#!/usr/bin/env python
"""Test the attractive seating approach with SOFA."""

import sys
from pathlib import Path

# Set up paths
_project_root = Path(__file__).parent.parent
sys.path.insert(0, str(_project_root / "src"))
sys.path.insert(0, str(_project_root))

import os
_sofa_root = os.environ.get("SOFA_ROOT", "/home/coreyt/sofa/SOFA_v24.06.00_Linux")
_sofa_python = f"{_sofa_root}/plugins/SofaPython3/lib/python3/site-packages"
if os.path.exists(_sofa_python) and _sofa_python not in sys.path:
    sys.path.insert(0, _sofa_python)

import argparse
import signal
import time
import numpy as np
import trimesh


# Timeout handler for detecting true hangs
class TimeoutHandler:
    """Context manager for detecting hangs with periodic status."""

    def __init__(self, timeout_sec=60, status_interval=5):
        self.timeout_sec = timeout_sec
        self.status_interval = status_interval
        self.start_time = None
        self.phase = "unknown"

    def set_phase(self, phase):
        """Update current phase for status reporting."""
        self.phase = phase

    def _handler(self, signum, frame):
        elapsed = time.time() - self.start_time
        if elapsed >= self.timeout_sec:
            print(f"\n[TIMEOUT] Hard timeout after {elapsed:.1f}s in phase: {self.phase}")
            print("[TIMEOUT] Stack trace:")
            import traceback
            traceback.print_stack(frame)
            sys.exit(1)
        else:
            print(f"[STATUS] {elapsed:.1f}s elapsed, currently in: {self.phase}")
            signal.alarm(self.status_interval)

    def __enter__(self):
        self.start_time = time.time()
        signal.signal(signal.SIGALRM, self._handler)
        signal.alarm(self.status_interval)
        return self

    def __exit__(self, *args):
        signal.alarm(0)  # Cancel the alarm

from scenes.attractive_seating import run_attractive_seating, identify_mating_points

# Known-good pose: CASE4 STL positions = designed seating pose
KNOWN_GOOD_POSITION = np.array([-21.85, 0.29, -132.14])
KNOWN_GOOD_QUATERNION = np.array([1.0, 0.0, 0.0, 0.0])  # Identity rotation


def run_diagnostics(guide, tibia, mating_pts_local, target_pts, guide_centroid, n_points, seed):
    """Diagnose mating point selection quality.

    Checks:
    1. Distance from mating points to FULL bone mesh (not 5000-point sample)
    2. Normal orientation of mating points (toward/away from bone)
    3. Net spring force at the known-good position
    4. Error vector between Phase 3b equilibrium and known-good pose
    """
    from scipy.spatial.transform import Rotation
    from trimesh.proximity import closest_point

    print("\n" + "=" * 60)
    print("MATING POINT DIAGNOSTICS")
    print("=" * 60)

    mating_pts_world = mating_pts_local + guide_centroid

    # --- 1. Distance to full bone mesh ---
    print("\n--- 1. Distance to Full Bone Mesh ---")
    full_closest, full_distances, full_face_ids = closest_point(tibia, mating_pts_world)
    print(f"  Mating points: {len(mating_pts_world)}")
    print(f"  Distance to full mesh: min={full_distances.min():.3f}, "
          f"max={full_distances.max():.3f}, mean={full_distances.mean():.3f} mm")
    print(f"  Median: {np.median(full_distances):.3f} mm")
    print(f"  Std:    {np.std(full_distances):.3f} mm")

    # Compare to KD-tree target distances
    from scipy.spatial import cKDTree
    target_tree = cKDTree(target_pts)
    kdtree_dists, _ = target_tree.query(mating_pts_world)
    print(f"\n  KD-tree target distances: min={kdtree_dists.min():.3f}, "
          f"max={kdtree_dists.max():.3f}, mean={kdtree_dists.mean():.3f} mm")
    discrepancy = full_distances - kdtree_dists
    print(f"  Full mesh vs KD-tree discrepancy: min={discrepancy.min():.3f}, "
          f"max={discrepancy.max():.3f}, mean={discrepancy.mean():.3f} mm")
    n_closer_full = np.sum(full_distances < kdtree_dists)
    print(f"  Points where full mesh is closer: {n_closer_full}/{len(mating_pts_world)} "
          f"({100*n_closer_full/len(mating_pts_world):.0f}%)")

    # --- 2. Normal orientation check ---
    print("\n--- 2. Normal Orientation Check ---")
    # Re-sample to get face indices for mating points
    rng = np.random.RandomState(seed)
    all_samples, all_face_indices = trimesh.sample.sample_surface(guide, n_points * 3, seed=rng)
    all_normals = guide.face_normals[all_face_indices]

    # Re-run the same filtering as identify_mating_points to get matching normals
    bone_samples, _ = trimesh.sample.sample_surface(tibia, 5000, seed=rng)
    bone_tree = cKDTree(bone_samples)
    distances_all, indices_all = bone_tree.query(all_samples, k=1)
    distance_threshold = np.percentile(distances_all, 40)
    mating_mask = distances_all < distance_threshold
    mating_normals = all_normals[mating_mask][:n_points]
    mating_samples = all_samples[mating_mask][:n_points]
    mating_bone_pts = bone_samples[indices_all[mating_mask]][:n_points]

    # Direction from guide surface point toward bone
    toward_bone = mating_bone_pts - mating_samples
    toward_bone_norm = toward_bone / (np.linalg.norm(toward_bone, axis=1, keepdims=True) + 1e-12)

    # Dot product: positive = normal points toward bone (concave/inner surface)
    #              negative = normal points away from bone (convex/outer surface)
    dots = np.sum(mating_normals * toward_bone_norm, axis=1)
    n_toward = np.sum(dots > 0)
    n_away = np.sum(dots <= 0)
    print(f"  Normals pointing toward bone (concave): {n_toward}/{len(dots)} "
          f"({100*n_toward/len(dots):.0f}%)")
    print(f"  Normals pointing away from bone (convex): {n_away}/{len(dots)} "
          f"({100*n_away/len(dots):.0f}%)")
    print(f"  Dot product stats: min={dots.min():.3f}, max={dots.max():.3f}, "
          f"mean={dots.mean():.3f}")

    # --- 3. Net spring force at known-good position ---
    print("\n--- 3. Net Spring Force at Known-Good Position ---")
    known_pos = KNOWN_GOOD_POSITION
    known_quat = KNOWN_GOOD_QUATERNION
    print(f"  Known-good position: {known_pos}")
    print(f"  Known-good quaternion: {known_quat} (identity)")

    # At the known-good position with identity rotation, the mating points
    # in world frame are simply: local + known_pos
    # (Identity rotation means R = I, so R @ local = local)
    R_known = Rotation.from_quat([known_quat[1], known_quat[2], known_quat[3], known_quat[0]])
    mating_at_known = R_known.apply(mating_pts_local) + known_pos

    # Spring forces: F = k * (target - current) for each point
    # Using stiffness = 1.0 to show the raw displacement-based force direction
    spring_displacements = target_pts - mating_at_known
    point_forces = spring_displacements  # k=1 to show displacement direction
    net_force = point_forces.sum(axis=0)
    net_force_mag = np.linalg.norm(net_force)
    net_force_dir = net_force / (net_force_mag + 1e-12)

    print(f"  Net spring displacement sum: [{net_force[0]:.3f}, {net_force[1]:.3f}, {net_force[2]:.3f}] mm")
    print(f"  Magnitude: {net_force_mag:.3f} mm (× stiffness = force)")
    print(f"  Direction: [{net_force_dir[0]:.3f}, {net_force_dir[1]:.3f}, {net_force_dir[2]:.3f}]")

    # Per-component analysis
    print(f"\n  Per-axis displacement analysis:")
    for axis, name in enumerate(["X", "Y", "Z"]):
        disps = spring_displacements[:, axis]
        n_pos = np.sum(disps > 0)
        n_neg = np.sum(disps < 0)
        print(f"    {name}: mean={disps.mean():.3f}, std={disps.std():.3f}, "
              f"positive={n_pos}, negative={n_neg}")

    # Distance from each mating point to its target AT the known-good pose
    dists_at_known = np.linalg.norm(spring_displacements, axis=1)
    print(f"\n  Mating-to-target distances at known-good pose:")
    print(f"    min={dists_at_known.min():.3f}, max={dists_at_known.max():.3f}, "
          f"mean={dists_at_known.mean():.3f} mm")

    # Net torque at known-good position
    r_vectors = mating_at_known - known_pos
    torques = np.cross(r_vectors, point_forces)
    net_torque = torques.sum(axis=0)
    print(f"\n  Net torque at known-good: [{net_torque[0]:.1f}, {net_torque[1]:.1f}, {net_torque[2]:.1f}]")

    # If net force is nonzero at the designed pose, the springs are biased
    if net_force_mag > 0.1:
        print(f"\n  ** WARNING: Net force at known-good pose is NONZERO ({net_force_mag:.1f})")
        print(f"  ** This means the spring equilibrium does NOT coincide with the designed pose.")
        print(f"  ** The springs will pull the guide away from its correct position.")
        print(f"  ** Primary bias direction: {name} = {net_force_dir}")
    else:
        print(f"\n  Net force is near zero — spring equilibrium matches designed pose.")

    # --- 4. Error vector ---
    print("\n--- 4. Error Vector Analysis ---")
    phase3b_equilibrium = np.array([-21.86, 3.05, -132.48])
    error_vec = phase3b_equilibrium - known_pos
    error_mag = np.linalg.norm(error_vec)
    print(f"  Known-good:        {known_pos}")
    print(f"  Phase 3b equil:    {phase3b_equilibrium}")
    print(f"  Error vector:      [{error_vec[0]:.3f}, {error_vec[1]:.3f}, {error_vec[2]:.3f}] mm")
    print(f"  Error magnitude:   {error_mag:.3f} mm")
    print(f"  Error direction:   [{error_vec[0]/error_mag:.3f}, {error_vec[1]/error_mag:.3f}, {error_vec[2]/error_mag:.3f}]")

    # Check if net force direction aligns with error direction
    alignment = np.dot(net_force_dir, error_vec / (error_mag + 1e-12))
    print(f"\n  Net force / error alignment: {alignment:.3f} "
          f"({'ALIGNED — force drives error' if alignment > 0.5 else 'NOT aligned' if alignment > -0.5 else 'OPPOSED'})")

    print("\n" + "=" * 60)

    return {
        'full_mesh_distances': full_distances,
        'kdtree_distances': kdtree_dists,
        'normal_dots': dots,
        'net_force': net_force,
        'net_torque': net_torque,
        'error_vec': error_vec,
        'alignment': alignment,
    }


def run_validation_tests(guide, tibia, args):
    """Run V1-V6 validation tests from known-good pose with offsets.

    Each test starts from the known-good position + an offset and checks
    whether the simulation converges back near the known-good pose.
    """
    from scipy.spatial.transform import Rotation

    print("\n" + "=" * 60)
    print("VALIDATION TESTS (V1-V6)")
    print("=" * 60)

    known_pos = KNOWN_GOOD_POSITION
    success_threshold = 0.5  # mm

    tests = [
        ("V1", [0, 5, 0],    None,  "5mm +Y offset"),
        ("V2", [0, -5, 0],   None,  "5mm -Y offset"),
        ("V3", [5, 0, 0],    None,  "5mm +X offset"),
        ("V4", [0, 0, 5],    None,  "5mm +Z offset"),
        ("V5", [0, 10, 0],   None,  "10mm +Y offset"),
        ("V6", [0, 5, 0],    10.0,  "5mm +Y + 10deg rotation"),
    ]

    results = []

    for test_name, offset, rot_deg, description in tests:
        print(f"\n--- {test_name}: {description} ---")

        # Initial position = known-good + offset
        initial_pos = known_pos + np.array(offset, dtype=float)
        initial_quat = KNOWN_GOOD_QUATERNION.copy()

        # Apply rotation offset if specified (rotate around X axis)
        if rot_deg is not None:
            angle_rad = np.radians(rot_deg)
            r = Rotation.from_rotvec([angle_rad, 0, 0])
            # Combine with identity: just the rotation itself
            initial_quat_scipy = r.as_quat()  # [x, y, z, w]
            initial_quat = np.array([initial_quat_scipy[3], initial_quat_scipy[0],
                                     initial_quat_scipy[1], initial_quat_scipy[2]])

        print(f"  Start position: {initial_pos}")
        print(f"  Start quaternion: {initial_quat.round(4)}")

        result = run_attractive_seating(
            tibia_mesh=tibia,
            guide_mesh=guide,
            initial_position=initial_pos,
            initial_quaternion=initial_quat,
            spring_stiffness=args.stiffness,
            spring_damping=args.damping,
            rotation_stiffness=args.rotation_stiffness,
            max_angular_velocity=args.max_angular_velocity,
            n_points=args.n_points,
            seed=args.seed,
            max_time=args.max_time,
            gui=False,
            enable_collision=not args.no_collision,
            use_controller=args.controller,
            gravity=[0, 0, 0] if args.no_gravity else None,
            collision_faces=args.collision_faces,
            collision_faces_coarse=args.collision_faces_coarse,
            collision_faces_fine=args.collision_faces_fine,
        )

        final_pos = result['pose'].position
        error_to_known = final_pos - known_pos
        error_mag = np.linalg.norm(error_to_known)
        passed = error_mag < success_threshold

        print(f"  Final position: [{final_pos[0]:.3f}, {final_pos[1]:.3f}, {final_pos[2]:.3f}]")
        print(f"  Error from known-good: [{error_to_known[0]:.3f}, {error_to_known[1]:.3f}, {error_to_known[2]:.3f}] mm")
        print(f"  Error magnitude: {error_mag:.3f} mm")
        print(f"  Converged: {result['converged']} ({result['steps']} steps)")
        print(f"  PASS: {'YES' if passed else 'NO'} (threshold: {success_threshold} mm)")

        results.append({
            'name': test_name,
            'description': description,
            'offset': offset,
            'rot_deg': rot_deg,
            'final_pos': final_pos,
            'error_vec': error_to_known,
            'error_mag': error_mag,
            'converged': result['converged'],
            'steps': result['steps'],
            'quaternion': result['pose'].quaternion,
            'passed': passed,
        })

    # Summary table
    print("\n" + "=" * 60)
    print("VALIDATION SUMMARY")
    print("=" * 60)
    print(f"{'Test':<6} {'Offset':<18} {'Final Position':<35} {'Error (mm)':<12} {'Pass'}")
    print("-" * 85)
    for r in results:
        offset_str = str(r['offset'])
        if r['rot_deg']:
            offset_str += f" +{r['rot_deg']}°"
        pos = r['final_pos']
        pos_str = f"[{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}]"
        print(f"{r['name']:<6} {offset_str:<18} {pos_str:<35} {r['error_mag']:<12.3f} {'YES' if r['passed'] else 'NO'}")

    n_passed = sum(1 for r in results if r['passed'])
    print(f"\nPassed: {n_passed}/{len(results)}")

    # Check if all converge to the same position (systematic error)
    final_positions = np.array([r['final_pos'] for r in results])
    spread = final_positions.max(axis=0) - final_positions.min(axis=0)
    mean_pos = final_positions.mean(axis=0)
    print(f"\nPosition spread across all tests: [{spread[0]:.3f}, {spread[1]:.3f}, {spread[2]:.3f}] mm")
    print(f"Mean final position: [{mean_pos[0]:.3f}, {mean_pos[1]:.3f}, {mean_pos[2]:.3f}]")
    mean_error = np.linalg.norm(mean_pos - known_pos)
    print(f"Mean error from known-good: {mean_error:.3f} mm")

    if spread.max() < 0.1 and mean_error > success_threshold:
        print(f"\n** SYSTEMATIC ERROR: All tests converge to the same wrong position.")
        print(f"** The spring model equilibrium is {mean_error:.2f}mm from the designed pose.")
        print(f"** This is NOT a convergence issue — the equilibrium itself is biased.")

    return results


def main():
    parser = argparse.ArgumentParser(
        description="Test attractive seating with SOFA",
        epilog="See docs/settings-reference.md for detailed parameter documentation."
    )
    parser.add_argument("tibia_mesh", help="Path to tibia STL")
    parser.add_argument("guide_mesh", help="Path to guide STL")

    # Settings file
    parser.add_argument("--settings", type=str, default=None,
                        help="Path to settings.yaml (default: auto-detect)")

    # Spring parameters (override settings.yaml)
    parser.add_argument("--stiffness", type=float, default=None,
                        help="Spring stiffness per spring (default: from settings.yaml)")
    parser.add_argument("--damping", type=float, default=None,
                        help="Spring damping coefficient (default: from settings.yaml)")

    # Simulation parameters
    parser.add_argument("--max-time", type=float, default=None,
                        help="Maximum simulation time (s) (default: from settings.yaml)")
    parser.add_argument("--gui", action="store_true",
                        help="Run with SOFA GUI")
    parser.add_argument("--screenshot", type=str,
                        help="Save result visualization")
    parser.add_argument("--timeout", type=int, default=120,
                        help="Hard timeout in seconds (default: 120)")
    parser.add_argument("--status-interval", type=int, default=10,
                        help="Status print interval in seconds (default: 10)")

    # Initial position
    parser.add_argument("--offset", type=float, nargs=3, default=None,
                        metavar=("X", "Y", "Z"),
                        help="Initial offset to move guide away from bone (mm) (default: from settings.yaml)")

    # Collision options
    parser.add_argument("--no-collision", action="store_true",
                        help="Disable collision detection (test springs only)")
    parser.add_argument("--controller", action="store_true",
                        help="Use Python controller for forces (bypasses RigidMapping issues)")

    # Mating point options (override settings.yaml)
    parser.add_argument("--n-points", type=int, default=None,
                        help="Number of mating points (default: from settings.yaml)")
    parser.add_argument("--seed", type=int, default=None,
                        help="Random seed for mating point sampling (default: from settings.yaml)")
    parser.add_argument("--distance-threshold", type=float, default=None,
                        help="Mating point distance threshold in mm (default: from settings.yaml)")

    # Rotation options (override settings.yaml)
    parser.add_argument("--rotation-stiffness", type=float, default=None,
                        help="Rotation constraint stiffness (default: from settings.yaml)")
    parser.add_argument("--max-angular-velocity", type=float, default=None,
                        help="Max angular velocity in rad/s (default: from settings.yaml)")

    # Other options
    parser.add_argument("--no-gravity", action="store_true",
                        help="Disable gravity (for testing)")
    parser.add_argument("--collision-faces", type=int, default=None,
                        help="Number of faces for decimated collision meshes (legacy single-mesh mode)")
    parser.add_argument("--collision-faces-coarse", type=int, default=None,
                        help="Coarse collision mesh faces for multi-phase (default: from settings.yaml)")
    parser.add_argument("--collision-faces-fine", type=int, default=None,
                        help="Fine collision mesh faces for multi-phase (default: from settings.yaml)")
    parser.add_argument("--multi-phase", action="store_true",
                        help="Enable multi-phase collision (uses settings.yaml values)")
    parser.add_argument("--diagnose", action="store_true",
                        help="Run mating point diagnostics (no simulation)")
    parser.add_argument("--validate", action="store_true",
                        help="Run V1-V6 validation tests from known-good pose")
    args = parser.parse_args()

    # Load settings from file
    from scenes.attractive_seating import load_settings
    settings = load_settings(args.settings)

    # Apply defaults from settings.yaml for any unspecified arguments
    if args.stiffness is None:
        args.stiffness = settings['springs']['stiffness']
    if args.damping is None:
        args.damping = settings['springs']['damping']
    if args.max_time is None:
        args.max_time = settings['simulation']['max_time']
    if args.n_points is None:
        args.n_points = settings['mating_points']['n_points']
    if args.seed is None:
        args.seed = settings['mating_points']['seed']
    if args.rotation_stiffness is None:
        args.rotation_stiffness = settings['rotation']['stiffness']
    if args.max_angular_velocity is None:
        args.max_angular_velocity = settings['rotation']['max_angular_velocity']
    if args.offset is None:
        args.offset = settings['initial_offset']['offset_mm']
    if args.collision_faces is None:
        args.collision_faces = settings['collision']['single_phase']['faces']
    if args.collision_faces_coarse is None:
        args.collision_faces_coarse = settings['collision']['multi_phase']['coarse_faces'] if settings['collision']['multi_phase']['enabled'] else 0
    if args.collision_faces_fine is None:
        args.collision_faces_fine = settings['collision']['multi_phase']['fine_faces'] if settings['collision']['multi_phase']['enabled'] else 0

    # Handle --multi-phase flag (enables multi-phase with settings values)
    if args.multi_phase:
        if args.collision_faces_coarse == 0:
            args.collision_faces_coarse = settings['collision']['multi_phase']['coarse_faces']
        if args.collision_faces_fine == 0:
            args.collision_faces_fine = settings['collision']['multi_phase']['fine_faces']

    # Get distance threshold from settings or CLI
    if args.distance_threshold is None:
        args.distance_threshold = settings['mating_points']['distance_threshold_mm']

    print("=" * 60)
    print("Attractive Seating Test")
    print(f"  Timeout: {args.timeout}s, Status interval: {args.status_interval}s")
    print(f"  Mating point distance threshold: {args.distance_threshold}mm")
    print(f"  Spring stiffness: {args.stiffness}, damping: {args.damping}")
    if args.collision_faces_coarse > 0 and args.collision_faces_fine > 0:
        print(f"  Multi-phase collision: coarse={args.collision_faces_coarse}, fine={args.collision_faces_fine}")
    elif not args.no_collision:
        print(f"  Single-phase collision: {args.collision_faces} faces")
    print("=" * 60)

    with TimeoutHandler(timeout_sec=args.timeout, status_interval=args.status_interval) as timeout:
        # Load meshes
        timeout.set_phase("loading meshes")
        print("\nLoading meshes...")
        tibia = trimesh.load(args.tibia_mesh)
        guide = trimesh.load(args.guide_mesh)
        print(f"  Tibia: {len(tibia.vertices)} vertices")
        print(f"  Guide: {len(guide.vertices)} vertices")

        # Show initial state
        print(f"\nInitial guide centroid: {guide.centroid}")
        print(f"Tibia centroid: {tibia.centroid}")

        # Check mating points
        timeout.set_phase("identifying mating points")
        print("\nIdentifying mating points...")
        mating_pts_local, target_pts, guide_centroid = identify_mating_points(
            guide, tibia,
            n_points=args.n_points,
            seed=args.seed,
            distance_threshold=args.distance_threshold
        )
        mating_pts = mating_pts_local + guide_centroid  # Convert to world for display
        print(f"  Found {len(mating_pts)} mating points")

        # Calculate initial distances
        from scipy.spatial import cKDTree
        bone_tree = cKDTree(target_pts)
        distances, _ = bone_tree.query(mating_pts)
        print(f"  Initial mating point distances: min={distances.min():.2f}, "
              f"max={distances.max():.2f}, mean={distances.mean():.2f} mm")

        # Run diagnostics if requested
        if args.diagnose or args.validate:
            timeout.set_phase("running diagnostics")
            diag_results = run_diagnostics(guide, tibia, mating_pts_local, target_pts,
                                           guide_centroid, args.n_points, args.seed)

        if args.validate:
            timeout.set_phase("running validation tests")
            val_results = run_validation_tests(guide, tibia, args)
            return 0

        if args.diagnose:
            return 0

        # Run simulation
        timeout.set_phase("SOFA simulation (init + loop)")
        print(f"\nRunning simulation (stiffness={args.stiffness}, max_time={args.max_time}s)...")
        t_total_start = time.time()
        result = run_attractive_seating(
            tibia_mesh=tibia,
            guide_mesh=guide,
            initial_offset=args.offset,
            spring_stiffness=args.stiffness,
            spring_damping=args.damping,
            rotation_stiffness=args.rotation_stiffness,
            max_angular_velocity=args.max_angular_velocity,
            n_points=args.n_points,
            seed=args.seed,
            max_time=args.max_time,
            gui=args.gui,
            enable_collision=not args.no_collision,
            use_controller=args.controller,
            gravity=[0, 0, 0] if args.no_gravity else None,
            collision_faces=args.collision_faces,
            collision_faces_coarse=args.collision_faces_coarse,
            collision_faces_fine=args.collision_faces_fine,
        )
        t_total_end = time.time()
        print(f"\n[TIMING] Total simulation time: {t_total_end - t_total_start:.2f}s")

    print("\n" + "=" * 60)
    print("RESULT")
    print("=" * 60)
    print(f"Converged: {result['converged']}")
    print(f"Steps: {result['steps']}")
    print(f"Final position: {result['pose'].position}")
    print(f"Final quaternion: {result['pose'].quaternion}")

    # Calculate movement
    initial_pos = guide.centroid
    final_pos = result['pose'].position
    movement = final_pos - initial_pos
    print(f"\nMovement from initial: [{movement[0]:.2f}, {movement[1]:.2f}, {movement[2]:.2f}] mm")
    print(f"Total displacement: {np.linalg.norm(movement):.2f} mm")

    # Visualization
    if args.screenshot:
        try:
            import pyvista as pv

            def mesh_to_pv(mesh):
                faces = np.column_stack([np.full(len(mesh.faces), 3), mesh.faces]).ravel()
                return pv.PolyData(mesh.vertices, faces)

            # Transform guide to final position
            from scipy.spatial.transform import Rotation
            guide_final = guide.copy()

            # Apply rotation
            q = result['pose'].quaternion  # [w, x, y, z]
            r = Rotation.from_quat([q[1], q[2], q[3], q[0]])
            R = r.as_matrix()

            # Transform: rotate around centroid, then translate
            guide_final.vertices = (R @ (guide_final.vertices - guide.centroid).T).T
            guide_final.vertices += result['pose'].position

            plotter = pv.Plotter(off_screen=True, window_size=(1200, 800))
            plotter.set_background("white")
            plotter.add_mesh(mesh_to_pv(tibia), color="ivory", opacity=0.7, label="Tibia")
            plotter.add_mesh(mesh_to_pv(guide), color="lightcoral", opacity=0.3, label="Guide (initial)")
            plotter.add_mesh(mesh_to_pv(guide_final), color="steelblue", opacity=1.0, label="Guide (seated)")

            # Add mating point visualization
            plotter.add_points(mating_pts + guide.centroid, color="red", point_size=5, label="Mating points")
            plotter.add_points(target_pts, color="green", point_size=5, label="Target points")

            plotter.add_axes()
            plotter.add_legend()
            plotter.camera_position = "iso"
            plotter.screenshot(args.screenshot)
            print(f"\nScreenshot saved: {args.screenshot}")

        except ImportError:
            print("\nPyVista not available for visualization")

    return 0


if __name__ == "__main__":
    sys.exit(main())
