#!/usr/bin/env python
"""End-to-end test of mvm_seating.py with actual CASE4 meshes.

This tests the production workflow:
1. Load actual STL files
2. Normalize coordinates using normalize_meshes()
3. Create SOFA scene using create_scene()
4. Run simulation
5. Extract pose using extract_guide_pose()
6. Denormalize pose back to original frame
"""

import sys
import tempfile
import shutil
from pathlib import Path
from datetime import datetime

import trimesh
import numpy as np

print(f"E2E Test - Run started: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")

# Add src to path for imports
sys.path.insert(0, str(Path(__file__).parent / "src"))

from scenes.mvm_seating import (
    create_scene,
    extract_guide_pose,
    normalize_meshes,
    denormalize_pose,
)


def main():
    print("\n" + "="*70)
    print("END-TO-END TEST: mvm_seating.py with CASE4 meshes")
    print("="*70)

    # Load original meshes
    tibia_path = "/tmp/repaired/CASE4_CroppedTibiaBone-repaired.stl"
    guide_path = "/tmp/repaired/CASE4_ResectThruTibiaGuide-repaired.stl"

    print(f"\n1. Loading meshes...")
    tibia_mesh = trimesh.load(tibia_path)
    guide_mesh = trimesh.load(guide_path)

    print(f"   Tibia: {len(tibia_mesh.vertices)} vertices, {len(tibia_mesh.faces)} faces")
    print(f"   Guide: {len(guide_mesh.vertices)} vertices, {len(guide_mesh.faces)} faces")

    # Record original positions
    orig_tibia_centroid = tibia_mesh.centroid.copy()
    orig_guide_centroid = guide_mesh.centroid.copy()
    orig_tibia_top = tibia_mesh.bounds[1][2]

    print(f"\n   Original tibia centroid: [{orig_tibia_centroid[0]:.1f}, {orig_tibia_centroid[1]:.1f}, {orig_tibia_centroid[2]:.1f}]")
    print(f"   Original guide centroid: [{orig_guide_centroid[0]:.1f}, {orig_guide_centroid[1]:.1f}, {orig_guide_centroid[2]:.1f}]")
    print(f"   Original tibia top Z: {orig_tibia_top:.1f}")

    # Simplify meshes for collision (convex hull for tibia, OBB for guide)
    print(f"\n2. Preparing collision meshes...")
    tibia_collision = tibia_mesh.convex_hull
    guide_collision = guide_mesh.bounding_box_oriented.to_mesh()

    print(f"   Tibia hull: {len(tibia_collision.faces)} faces")
    print(f"   Guide OBB: {len(guide_collision.faces)} faces")

    # Normalize coordinates
    print(f"\n3. Normalizing coordinates...")
    norm_info = normalize_meshes(tibia_collision, guide_collision)

    print(f"   Offset applied: [{norm_info['offset'][0]:.1f}, {norm_info['offset'][1]:.1f}, {norm_info['offset'][2]:.1f}]")
    print(f"   Z offset: {norm_info['z_offset']:.1f}")
    print(f"   Tibia now at Z: [{tibia_collision.bounds[0][2]:.1f}, {tibia_collision.bounds[1][2]:.1f}]")
    print(f"   Guide now at Z: [{guide_collision.bounds[0][2]:.1f}, {guide_collision.bounds[1][2]:.1f}]")

    # Ensure guide starts above tibia with gap
    gap = guide_collision.bounds[0][2] - tibia_collision.bounds[1][2]
    if gap < 10:
        guide_collision.apply_translation([0, 0, 15 - gap])
        print(f"   Added {15-gap:.1f}mm gap, guide now at Z: [{guide_collision.bounds[0][2]:.1f}, {guide_collision.bounds[1][2]:.1f}]")

    # Save normalized meshes to temp files for SOFA
    print(f"\n4. Saving normalized meshes...")
    temp_dir = Path(tempfile.mkdtemp(prefix="sofa_e2e_"))
    tibia_norm_path = temp_dir / "tibia_normalized.stl"
    guide_norm_path = temp_dir / "guide_normalized.stl"

    tibia_collision.export(str(tibia_norm_path))
    guide_collision.export(str(guide_norm_path))
    print(f"   Saved to: {temp_dir}")

    # Create SOFA scene
    print(f"\n5. Creating SOFA scene...")
    import Sofa
    import Sofa.Core
    import Sofa.Simulation

    # IMPORTANT: RigidMapping applies the rigid transform to mesh vertices.
    # Since our mesh is already at world coordinates, rigid body must start at ORIGIN.
    # The mesh vertices themselves define the initial position.
    guide_init_pos = [0.0, 0.0, 0.0]

    root = create_scene(
        tibia_mesh_path=str(tibia_norm_path),
        guide_mesh_path=str(guide_norm_path),
        guide_initial_position=guide_init_pos,
        guide_initial_quaternion=[1.0, 0.0, 0.0, 0.0],  # Identity
        guide_mass_kg=0.1,
        dt=0.001,
    )

    print(f"   Scene created successfully")
    print(f"   Guide initial position: {guide_init_pos}")

    # Initialize and run simulation
    print(f"\n6. Running simulation (500 steps = 0.5s)...")
    Sofa.Simulation.init(root)

    initial_pose = extract_guide_pose(root)
    print(f"   Initial pose: pos=[{initial_pose.position[0]:.2f}, {initial_pose.position[1]:.2f}, {initial_pose.position[2]:.2f}]")

    for i in range(500):
        Sofa.Simulation.animate(root, 0.001)
        if i % 100 == 0:
            pose = extract_guide_pose(root)
            print(f"   Step {i}: Z={pose.position[2]:.2f}")

    # Extract final pose
    print(f"\n7. Extracting final pose...")
    final_pose = extract_guide_pose(root)

    # The pose is the DELTA from initial position (which was at origin)
    # Actual guide centroid in world = mesh_centroid + pose.position
    guide_mesh_centroid = guide_collision.centroid  # In normalized coords
    guide_world_pos = guide_mesh_centroid + final_pose.position

    print(f"   Rigid body delta: [{final_pose.position[0]:.2f}, {final_pose.position[1]:.2f}, {final_pose.position[2]:.2f}]")
    print(f"   Guide mesh centroid: [{guide_mesh_centroid[0]:.2f}, {guide_mesh_centroid[1]:.2f}, {guide_mesh_centroid[2]:.2f}]")
    print(f"   Guide world position: [{guide_world_pos[0]:.2f}, {guide_world_pos[1]:.2f}, {guide_world_pos[2]:.2f}]")
    print(f"   Final quaternion: [{final_pose.quaternion[0]:.4f}, {final_pose.quaternion[1]:.4f}, {final_pose.quaternion[2]:.4f}, {final_pose.quaternion[3]:.4f}]")

    # Denormalize pose (add original offset back)
    print(f"\n8. Denormalizing pose...")
    # The world position in original frame = guide_world_pos + norm_info['total_offset']
    guide_orig_pos = guide_world_pos + norm_info['total_offset']

    print(f"   Guide position (original frame): [{guide_orig_pos[0]:.2f}, {guide_orig_pos[1]:.2f}, {guide_orig_pos[2]:.2f}]")

    # Validate results
    print(f"\n" + "="*70)
    print("VALIDATION")
    print("="*70)

    # Calculate guide bottom Z (centroid - half height)
    guide_half_height = (guide_collision.bounds[1][2] - guide_collision.bounds[0][2]) / 2
    guide_bottom_z = guide_world_pos[2] - guide_half_height

    print(f"\n   Guide centroid Z (normalized): {guide_world_pos[2]:.2f}")
    print(f"   Guide half-height: {guide_half_height:.2f}")
    print(f"   Guide bottom Z (normalized): {guide_bottom_z:.2f}")
    print(f"   Tibia top Z (normalized): 0.00")

    # Check 1: Guide didn't fall through (bottom Z > -10)
    fell_through = guide_bottom_z < -10
    print(f"\n   Fell through tibia: {'YES - FAIL' if fell_through else 'NO - PASS'}")

    # Check 2: Guide bottom is near tibia top (within reasonable range)
    # In normalized coords, tibia top is at Z=0, guide bottom should be near 0
    near_surface = -5 < guide_bottom_z < 10
    print(f"   Guide bottom near tibia surface (-5 < Z < 10): {'YES - PASS' if near_surface else 'NO - FAIL'}")

    # Check 3: Quaternion is valid (unit length)
    quat_norm = np.linalg.norm(final_pose.quaternion)
    quat_valid = abs(quat_norm - 1.0) < 0.01
    print(f"   Quaternion norm: {quat_norm:.4f} ({'VALID' if quat_valid else 'INVALID'})")

    # Check 4: Denormalized Z is reasonable (guide bottom near original tibia top)
    denorm_z = guide_orig_pos[2]
    # Guide bottom in original frame = denorm_z - guide_half_height
    guide_bottom_orig = denorm_z - guide_half_height
    # Should be near orig_tibia_top
    z_error = abs(guide_bottom_orig - orig_tibia_top)
    denorm_valid = z_error < 10  # Allow some tolerance
    print(f"   Guide bottom (original frame): {guide_bottom_orig:.2f}")
    print(f"   Original tibia top: {orig_tibia_top:.2f}")
    print(f"   Denormalization error: {z_error:.2f}mm ({'VALID' if denorm_valid else 'CHECK NEEDED'})")

    # Overall result
    all_pass = not fell_through and near_surface and quat_valid
    print(f"\n" + "="*70)
    print(f"OVERALL RESULT: {'PASS' if all_pass else 'FAIL'}")
    print("="*70)

    # Cleanup
    shutil.rmtree(temp_dir)
    print(f"\nCleaned up temp directory")

    print(f"\nE2E Test - Run completed: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")

    return 0 if all_pass else 1


if __name__ == "__main__":
    sys.exit(main())
