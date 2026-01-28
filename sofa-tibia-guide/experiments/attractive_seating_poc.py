#!/usr/bin/env python
"""Proof-of-concept: Attractive Contact Physics for Guide Seating.

This implements Option 2 from seating-approaches.md:
- Attractive forces pull guide mating surface toward bone
- Repulsive forces prevent interpenetration
- Multi-start search finds global and local optima
- Perturbation analysis measures slop

Usage:
    python experiments/attractive_seating_poc.py <tibia.stl> <guide.stl>
"""

import argparse
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np
import trimesh
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation


@dataclass
class RigidState:
    """State of a rigid body."""
    position: np.ndarray  # [x, y, z] center of mass
    quaternion: np.ndarray  # [w, x, y, z] orientation
    velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))
    angular_velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))

    def copy(self) -> "RigidState":
        return RigidState(
            position=self.position.copy(),
            quaternion=self.quaternion.copy(),
            velocity=self.velocity.copy(),
            angular_velocity=self.angular_velocity.copy(),
        )

    def rotation_matrix(self) -> np.ndarray:
        """Get 3x3 rotation matrix from quaternion."""
        r = Rotation.from_quat([self.quaternion[1], self.quaternion[2],
                                self.quaternion[3], self.quaternion[0]])
        return r.as_matrix()

    def transform_points(self, points: np.ndarray) -> np.ndarray:
        """Transform points from body frame to world frame."""
        R = self.rotation_matrix()
        return (R @ points.T).T + self.position

    def transform_normals(self, normals: np.ndarray) -> np.ndarray:
        """Transform normals from body frame to world frame."""
        R = self.rotation_matrix()
        return (R @ normals.T).T


@dataclass
class SeatingConfig:
    """Configuration for seating simulation."""
    # Force parameters
    sigma: float = 1.0  # Equilibrium distance (mm)
    epsilon: float = 100.0  # Attraction strength
    cutoff_distance: float = 20.0  # Max distance for force computation (mm)

    # Sampling
    sample_density: float = 0.1  # Points per mm² of surface
    max_sample_points: int = 500  # Cap on sample points

    # Dynamics
    mass: float = 0.1  # kg
    inertia_scale: float = 1.0  # Scale factor for moment of inertia
    damping: float = 0.9  # Velocity damping per step
    angular_damping: float = 0.9  # Angular velocity damping
    dt: float = 0.001  # Time step (s)

    # Convergence
    max_steps: int = 10000
    velocity_threshold: float = 0.01  # mm/s
    angular_threshold: float = 0.001  # rad/s
    convergence_window: int = 100  # Steps to check convergence


@dataclass
class SeatingResult:
    """Result of a single seating simulation."""
    final_state: RigidState
    energy: float
    converged: bool
    steps: int
    trajectory: Optional[List[RigidState]] = None


def identify_mating_surface(
    guide_mesh: trimesh.Trimesh,
    bone_mesh: trimesh.Trimesh,
    config: SeatingConfig,
) -> Tuple[np.ndarray, np.ndarray]:
    """Identify points on guide that should mate with bone.

    Strategy: Sample guide surface, keep points whose normals point
    toward the bone centroid (these are the "inner" mating surfaces).

    Returns:
        points: (N, 3) sample points in guide's local frame
        normals: (N, 3) outward normals at those points
    """
    # Sample points on guide surface
    n_samples = min(
        int(guide_mesh.area * config.sample_density),
        config.max_sample_points
    )
    points, face_indices = trimesh.sample.sample_surface(guide_mesh, n_samples)
    normals = guide_mesh.face_normals[face_indices]

    # Filter to points facing toward bone centroid
    # (mating surface normals point "inward" toward where bone would be)
    bone_centroid = bone_mesh.centroid
    guide_centroid = guide_mesh.centroid

    # Vector from guide center toward bone center (rough "inward" direction)
    inward_dir = bone_centroid - guide_centroid
    inward_dir = inward_dir / np.linalg.norm(inward_dir)

    # Keep points whose normals have negative dot product with inward direction
    # (normal points away from surface, so mating surface normals point toward bone)
    # Actually, we want normals pointing TOWARD bone, which means normal · inward > 0
    # But surface normals point outward, so we want normal · inward < 0
    # Let me reconsider...

    # The guide's mating surface faces the bone. If we're looking at the guide,
    # the mating surface normals point "into" the guide (toward the bone when seated).
    # trimesh gives outward normals, so mating surface normals point away from bone.
    #
    # When the guide is in its designed position relative to bone:
    # - Mating surface normals point outward from guide surface
    # - But the mating surface is the concave inner surface
    # - So these normals actually point toward where the bone is
    #
    # Simpler approach: keep points that are closer to bone centroid than guide centroid

    to_bone = bone_centroid - points
    to_guide_center = guide_centroid - points

    dist_to_bone = np.linalg.norm(to_bone, axis=1)
    dist_to_guide_center = np.linalg.norm(to_guide_center, axis=1)

    # Mating surface points are generally closer to bone than to guide's own center
    # This is a heuristic that works for guides that "wrap around" the bone
    mating_mask = dist_to_bone < dist_to_guide_center * 1.5

    # Also filter by normal direction - mating surface normals should generally
    # point toward bone (dot product with to_bone should be positive)
    normal_dot = np.sum(normals * to_bone, axis=1) / (dist_to_bone + 1e-10)
    mating_mask &= normal_dot > 0

    if mating_mask.sum() < 10:
        # Fallback: just use all points
        print(f"Warning: Only {mating_mask.sum()} mating points found, using all {len(points)}")
        mating_mask = np.ones(len(points), dtype=bool)

    return points[mating_mask], normals[mating_mask]


def compute_attractive_force(
    guide_state: RigidState,
    guide_sample_points: np.ndarray,  # In guide's local frame
    guide_sample_normals: np.ndarray,  # In guide's local frame
    bone_tree: cKDTree,  # KD-tree of bone surface points
    bone_points: np.ndarray,  # Bone surface points
    bone_normals: np.ndarray,  # Bone surface normals
    config: SeatingConfig,
) -> Tuple[np.ndarray, np.ndarray]:
    """Compute attractive/repulsive force and torque on guide.

    Uses Lennard-Jones-like potential between guide sample points and
    nearest bone surface points.

    Returns:
        force: (3,) total force vector
        torque: (3,) total torque vector about guide center
    """
    # Transform guide points to world frame
    world_points = guide_state.transform_points(guide_sample_points)
    world_normals = guide_state.transform_normals(guide_sample_normals)

    # Find nearest bone points
    distances, indices = bone_tree.query(world_points, k=1)
    nearest_bone_points = bone_points[indices]
    nearest_bone_normals = bone_normals[indices]

    # Compute forces using modified Lennard-Jones potential
    # U(r) = epsilon * [(sigma/r)^12 - 2*(sigma/r)^6]
    # F(r) = -dU/dr = epsilon * [12*sigma^12/r^13 - 12*sigma^6/r^7]
    #      = 12 * epsilon / r * [(sigma/r)^12 - (sigma/r)^6]
    #
    # For numerical stability, clamp distances

    sigma = config.sigma
    epsilon = config.epsilon

    # Direction from bone to guide point (force direction for attraction)
    directions = world_points - nearest_bone_points
    r = np.maximum(distances, 0.1)[:, np.newaxis]  # Clamp minimum distance
    unit_directions = directions / r

    # Only apply forces within cutoff
    within_cutoff = distances < config.cutoff_distance

    # Lennard-Jones force magnitude
    # Positive = repulsive (pointing away from bone)
    # Negative = attractive (pointing toward bone)
    sr6 = (sigma / r.flatten()) ** 6
    sr12 = sr6 ** 2

    # Force magnitude: F = 12 * epsilon / r * (sr12 - sr6)
    # When r < sigma: sr12 > sr6, F > 0 (repulsive)
    # When r > sigma: sr12 < sr6, F < 0 (attractive)
    force_magnitudes = 12 * epsilon / r.flatten() * (sr12 - sr6)

    # Apply cutoff - smoothly taper to zero
    taper = np.maximum(0, 1 - distances / config.cutoff_distance) ** 2
    force_magnitudes *= taper
    force_magnitudes[~within_cutoff] = 0

    # Force vectors (pointing from guide toward bone when attractive)
    point_forces = -force_magnitudes[:, np.newaxis] * unit_directions

    # Total force
    total_force = point_forces.sum(axis=0)

    # Torque about guide center
    # τ = Σ (r_i × F_i) where r_i is position relative to guide center
    r_vectors = world_points - guide_state.position
    point_torques = np.cross(r_vectors, point_forces)
    total_torque = point_torques.sum(axis=0)

    return total_force, total_torque


def compute_energy(
    guide_state: RigidState,
    guide_sample_points: np.ndarray,
    bone_tree: cKDTree,
    config: SeatingConfig,
) -> float:
    """Compute potential energy of current configuration."""
    world_points = guide_state.transform_points(guide_sample_points)
    distances, _ = bone_tree.query(world_points, k=1)

    sigma = config.sigma
    epsilon = config.epsilon

    r = np.maximum(distances, 0.1)
    sr6 = (sigma / r) ** 6
    sr12 = sr6 ** 2

    # Lennard-Jones potential
    U = epsilon * (sr12 - 2 * sr6)

    # Apply cutoff taper
    within_cutoff = distances < config.cutoff_distance
    taper = np.maximum(0, 1 - distances / config.cutoff_distance) ** 2
    U *= taper
    U[~within_cutoff] = 0

    return U.sum()


def integrate_step(
    state: RigidState,
    force: np.ndarray,
    torque: np.ndarray,
    config: SeatingConfig,
) -> RigidState:
    """Integrate one timestep of rigid body dynamics."""
    dt = config.dt
    mass = config.mass

    # Simple Euler integration with damping
    # Linear motion
    acceleration = force / mass
    new_velocity = state.velocity * config.damping + acceleration * dt
    new_position = state.position + new_velocity * dt

    # Angular motion (simplified - assumes spherical inertia)
    # For a more accurate simulation, we'd compute proper inertia tensor
    inertia = mass * config.inertia_scale * 100  # Rough approximation
    angular_acceleration = torque / inertia
    new_angular_velocity = state.angular_velocity * config.angular_damping + angular_acceleration * dt

    # Update quaternion
    # dq/dt = 0.5 * q * ω (where ω is angular velocity as quaternion [0, ωx, ωy, ωz])
    omega_quat = np.array([0, new_angular_velocity[0], new_angular_velocity[1], new_angular_velocity[2]])
    q = state.quaternion
    dq = 0.5 * quaternion_multiply(q, omega_quat)
    new_quaternion = q + dq * dt
    new_quaternion = new_quaternion / np.linalg.norm(new_quaternion)  # Normalize

    return RigidState(
        position=new_position,
        quaternion=new_quaternion,
        velocity=new_velocity,
        angular_velocity=new_angular_velocity,
    )


def quaternion_multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """Multiply two quaternions [w, x, y, z]."""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    ])


def run_seating_simulation(
    bone_mesh: trimesh.Trimesh,
    guide_mesh: trimesh.Trimesh,
    initial_state: RigidState,
    config: SeatingConfig,
    record_trajectory: bool = False,
) -> SeatingResult:
    """Run a single seating simulation from initial state."""

    # Identify mating surface
    sample_points, sample_normals = identify_mating_surface(guide_mesh, bone_mesh, config)
    print(f"  Using {len(sample_points)} mating surface sample points")

    # Build KD-tree for bone surface
    bone_samples, bone_face_indices = trimesh.sample.sample_surface(bone_mesh, 5000)
    bone_normals = bone_mesh.face_normals[bone_face_indices]
    bone_tree = cKDTree(bone_samples)

    # Initialize
    state = initial_state.copy()
    trajectory = [state.copy()] if record_trajectory else None

    # Convergence tracking
    recent_velocities = []

    for step in range(config.max_steps):
        # Compute forces
        force, torque = compute_attractive_force(
            state, sample_points, sample_normals,
            bone_tree, bone_samples, bone_normals, config
        )

        # Integrate
        state = integrate_step(state, force, torque, config)

        if record_trajectory and step % 100 == 0:
            trajectory.append(state.copy())

        # Check convergence
        speed = np.linalg.norm(state.velocity)
        angular_speed = np.linalg.norm(state.angular_velocity)
        recent_velocities.append((speed, angular_speed))

        if len(recent_velocities) > config.convergence_window:
            recent_velocities.pop(0)
            avg_speed = np.mean([v[0] for v in recent_velocities])
            avg_angular = np.mean([v[1] for v in recent_velocities])

            if avg_speed < config.velocity_threshold and avg_angular < config.angular_threshold:
                energy = compute_energy(state, sample_points, bone_tree, config)
                return SeatingResult(
                    final_state=state,
                    energy=energy,
                    converged=True,
                    steps=step,
                    trajectory=trajectory,
                )

        # Progress
        if step % 1000 == 0:
            energy = compute_energy(state, sample_points, bone_tree, config)
            print(f"  Step {step}: energy={energy:.1f}, speed={speed:.3f}, angular={angular_speed:.4f}")

    # Did not converge
    energy = compute_energy(state, sample_points, bone_tree, config)
    return SeatingResult(
        final_state=state,
        energy=energy,
        converged=False,
        steps=config.max_steps,
        trajectory=trajectory,
    )


def generate_initial_poses(
    bone_mesh: trimesh.Trimesh,
    guide_mesh: trimesh.Trimesh,
    n_poses: int,
    offset_distance: float = 30.0,
) -> List[RigidState]:
    """Generate diverse initial poses for multi-start search."""
    poses = []

    bone_centroid = bone_mesh.centroid
    guide_centroid = guide_mesh.centroid

    # Original relative position
    original_offset = guide_centroid - bone_centroid

    for i in range(n_poses):
        # Random rotation
        random_rotation = Rotation.random()
        quat_xyzw = random_rotation.as_quat()
        quaternion = np.array([quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]])

        # Position: bone centroid plus offset in random direction
        if i == 0:
            # First pose: original position
            position = guide_centroid.copy()
            quaternion = np.array([1.0, 0.0, 0.0, 0.0])
        else:
            # Random direction offset from bone
            direction = np.random.randn(3)
            direction = direction / np.linalg.norm(direction)
            position = bone_centroid + direction * offset_distance

        poses.append(RigidState(position=position, quaternion=quaternion))

    return poses


def measure_slop(
    bone_mesh: trimesh.Trimesh,
    guide_mesh: trimesh.Trimesh,
    seated_state: RigidState,
    config: SeatingConfig,
    perturbation_force: float = 1.0,  # N
    perturbation_torque: float = 10.0,  # N·mm
) -> dict:
    """Measure slop by applying perturbations and measuring displacement.

    Returns dict with stiffness measurements for each DOF.
    """
    # Identify mating surface
    sample_points, sample_normals = identify_mating_surface(guide_mesh, bone_mesh, config)

    # Build KD-tree for bone
    bone_samples, bone_face_indices = trimesh.sample.sample_surface(bone_mesh, 5000)
    bone_normals_arr = bone_mesh.face_normals[bone_face_indices]
    bone_tree = cKDTree(bone_samples)

    results = {}

    # Test each DOF
    directions = [
        ("tx", np.array([perturbation_force, 0, 0]), np.zeros(3)),
        ("ty", np.array([0, perturbation_force, 0]), np.zeros(3)),
        ("tz", np.array([0, 0, perturbation_force]), np.zeros(3)),
        ("rx", np.zeros(3), np.array([perturbation_torque, 0, 0])),
        ("ry", np.zeros(3), np.array([0, perturbation_torque, 0])),
        ("rz", np.zeros(3), np.array([0, 0, perturbation_torque])),
    ]

    for dof_name, ext_force, ext_torque in directions:
        # Start from seated state
        state = seated_state.copy()

        # Apply perturbation and let settle
        max_displacement = 0.0
        max_rotation = 0.0
        initial_pos = state.position.copy()
        initial_quat = state.quaternion.copy()

        # Short simulation with external force
        for step in range(500):
            # Compute restoring force from attractive potential
            force, torque = compute_attractive_force(
                state, sample_points, sample_normals,
                bone_tree, bone_samples, bone_normals_arr, config
            )

            # Add external perturbation (only for first part)
            if step < 100:
                force = force + ext_force
                torque = torque + ext_torque

            state = integrate_step(state, force, torque, config)

            # Track maximum displacement
            displacement = np.linalg.norm(state.position - initial_pos)
            max_displacement = max(max_displacement, displacement)

            # Track rotation (simplified)
            quat_diff = np.abs(state.quaternion - initial_quat).sum()
            max_rotation = max(max_rotation, quat_diff)

        # Compute stiffness (force / displacement)
        if dof_name.startswith("t"):
            stiffness = perturbation_force / max(max_displacement, 0.001)
        else:
            stiffness = perturbation_torque / max(max_rotation, 0.0001)

        results[dof_name] = {
            "max_displacement": max_displacement,
            "max_rotation": max_rotation,
            "stiffness": stiffness,
        }

    return results


def main():
    parser = argparse.ArgumentParser(description="Attractive Contact Physics POC")
    parser.add_argument("tibia_mesh", help="Path to tibia STL")
    parser.add_argument("guide_mesh", help="Path to guide STL")
    parser.add_argument("--n-starts", type=int, default=5, help="Number of initial poses")
    parser.add_argument("--visualize", action="store_true", help="Show visualization")
    parser.add_argument("--screenshot", type=str, help="Save screenshot to file")
    args = parser.parse_args()

    # Load meshes
    print("Loading meshes...")
    bone_mesh = trimesh.load(args.tibia_mesh)
    guide_mesh = trimesh.load(args.guide_mesh)
    print(f"  Bone: {len(bone_mesh.vertices)} vertices, {len(bone_mesh.faces)} faces")
    print(f"  Guide: {len(guide_mesh.vertices)} vertices, {len(guide_mesh.faces)} faces")

    # Configuration
    config = SeatingConfig(
        sigma=2.0,  # 2mm equilibrium distance
        epsilon=50.0,  # Attraction strength
        cutoff_distance=30.0,  # 30mm cutoff
        sample_density=0.05,  # Sparser sampling for speed
        max_sample_points=300,
        damping=0.95,
        angular_damping=0.95,
        dt=0.0005,
        max_steps=20000,
    )

    # Generate initial poses
    print(f"\nGenerating {args.n_starts} initial poses...")
    initial_poses = generate_initial_poses(bone_mesh, guide_mesh, args.n_starts)

    # Run simulations
    results = []
    for i, initial_pose in enumerate(initial_poses):
        print(f"\n=== Simulation {i+1}/{args.n_starts} ===")
        print(f"  Initial position: {initial_pose.position}")

        result = run_seating_simulation(
            bone_mesh, guide_mesh, initial_pose, config,
            record_trajectory=(i == 0),  # Only record first trajectory
        )
        results.append(result)

        print(f"  Converged: {result.converged} in {result.steps} steps")
        print(f"  Final energy: {result.energy:.1f}")
        print(f"  Final position: {result.final_state.position}")

    # Sort by energy
    results.sort(key=lambda r: r.energy)

    print("\n" + "=" * 60)
    print("RESULTS (sorted by energy)")
    print("=" * 60)
    for i, result in enumerate(results):
        label = "GLOBAL OPTIMUM" if i == 0 else f"Local optimum {i}"
        print(f"\n{label}:")
        print(f"  Energy: {result.energy:.1f}")
        print(f"  Position: {result.final_state.position}")
        print(f"  Converged: {result.converged}")

    # Measure slop for best result
    print("\n" + "=" * 60)
    print("SLOP MEASUREMENT (best result)")
    print("=" * 60)
    best_result = results[0]
    slop = measure_slop(bone_mesh, guide_mesh, best_result.final_state, config)

    for dof, metrics in slop.items():
        print(f"  {dof}: displacement={metrics['max_displacement']:.3f}mm, stiffness={metrics['stiffness']:.1f}")

    # Visualization
    if args.visualize or args.screenshot:
        try:
            import pyvista as pv

            def mesh_to_pv(mesh):
                faces = np.column_stack([np.full(len(mesh.faces), 3), mesh.faces]).ravel()
                return pv.PolyData(mesh.vertices, faces)

            # Transform guide to final position
            best_state = best_result.final_state
            guide_transformed = guide_mesh.copy()

            # Apply rotation
            R = best_state.rotation_matrix()
            guide_transformed.vertices = (R @ (guide_transformed.vertices - guide_mesh.centroid).T).T
            guide_transformed.vertices += best_state.position

            bone_pv = mesh_to_pv(bone_mesh)
            guide_pv = mesh_to_pv(guide_transformed)
            guide_original_pv = mesh_to_pv(guide_mesh)

            plotter = pv.Plotter(off_screen=(args.screenshot is not None))
            plotter.set_background("white")
            plotter.add_mesh(bone_pv, color="ivory", opacity=0.7, label="Bone")
            plotter.add_mesh(guide_original_pv, color="lightcoral", opacity=0.3, label="Guide (initial)")
            plotter.add_mesh(guide_pv, color="steelblue", opacity=1.0, label="Guide (seated)")
            plotter.add_axes()
            plotter.add_legend()

            if args.screenshot:
                plotter.screenshot(args.screenshot)
                print(f"\nScreenshot saved: {args.screenshot}")
            else:
                plotter.show()

        except ImportError:
            print("\nPyVista not available for visualization")

    return 0


if __name__ == "__main__":
    sys.exit(main())
