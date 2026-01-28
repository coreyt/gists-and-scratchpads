#!/usr/bin/env python
"""Test script: Validate seating API with CASE4 meshes.

This script tests the find_seating_pose API using the CASE4 tibia and guide
meshes. It validates that the guide seats correctly on the tibia surface.

Expected results:
    - Guide seats with ~0.5mm gap above tibia surface
    - Simulation converges within 2 seconds
    - Quaternion stays near identity (minimal tilt)

Requirements:
    - SOFA with SofaPython3 installed
    - CASE4 meshes in /tmp/repaired/

Usage:
    export SOFA_ROOT=/path/to/SOFA
    export LD_LIBRARY_PATH="$SOFA_ROOT/lib:$LD_LIBRARY_PATH"
    python examples/test_case4.py
"""

import os
import sys
from pathlib import Path

# Set up SOFA path before any imports that might need it
_sofa_root = os.environ.get("SOFA_ROOT", "/home/coreyt/sofa/SOFA_v24.06.00_Linux")
_sofa_python = f"{_sofa_root}/plugins/SofaPython3/lib/python3/site-packages"
if os.path.exists(_sofa_python) and _sofa_python not in sys.path:
    sys.path.insert(0, _sofa_python)

# Add project paths if running from repo root
_project_root = Path(__file__).parent.parent
sys.path.insert(0, str(_project_root / "src"))
sys.path.insert(0, str(_project_root))  # For scenes module

import numpy as np
from tibia_guide import find_seating_pose


def main():
    # CASE4 mesh paths
    tibia_path = "/tmp/repaired/CASE4_CroppedTibiaBone-repaired.stl"
    guide_path = "/tmp/repaired/CASE4_ResectThruTibiaGuide-repaired.stl"

    # Check meshes exist
    if not Path(tibia_path).exists():
        print(f"Error: Tibia mesh not found: {tibia_path}")
        print("Please ensure CASE4 meshes are available in /tmp/repaired/")
        return 1
    if not Path(guide_path).exists():
        print(f"Error: Guide mesh not found: {guide_path}")
        print("Please ensure CASE4 meshes are available in /tmp/repaired/")
        return 1

    print("=" * 60)
    print("Testing find_seating_pose with CASE4 meshes")
    print("=" * 60)

    # Run simulation
    result = find_seating_pose(
        tibia_mesh=tibia_path,
        guide_mesh=guide_path,
        max_simulation_time=2.0,
        convergence_threshold=0.01,
    )

    # Print results
    print(f"\nResult:")
    print(f"  Position: [{result.pose.position[0]:.2f}, {result.pose.position[1]:.2f}, {result.pose.position[2]:.2f}] mm")
    print(f"  Quaternion: [{result.pose.quaternion[0]:.4f}, {result.pose.quaternion[1]:.4f}, {result.pose.quaternion[2]:.4f}, {result.pose.quaternion[3]:.4f}]")
    print(f"  Converged: {result.converged}")
    print(f"  Steps: {result.steps}")
    print(f"  Final gap: {result.final_gap_mm:.2f} mm")

    # Validation
    print("\n" + "=" * 60)
    print("Validation")
    print("=" * 60)

    passed = True

    # Check convergence
    if result.converged:
        print("  [PASS] Simulation converged")
    else:
        print("  [FAIL] Simulation did not converge")
        passed = False

    # Check gap is reasonable (guide resting on surface, not penetrating)
    if -1 < result.final_gap_mm < 5:
        print(f"  [PASS] Gap is reasonable: {result.final_gap_mm:.2f}mm")
    else:
        print(f"  [FAIL] Gap is out of range: {result.final_gap_mm:.2f}mm")
        passed = False

    # Check quaternion is near identity (no major tipping)
    quat_identity_dist = np.linalg.norm(result.pose.quaternion - np.array([1, 0, 0, 0]))
    if quat_identity_dist < 0.1:
        print(f"  [PASS] Orientation near upright: {quat_identity_dist:.4f} from identity")
    else:
        print(f"  [WARN] Orientation tilted: {quat_identity_dist:.4f} from identity")

    print("=" * 60)
    print(f"Overall: {'PASS' if passed else 'FAIL'}")
    print("=" * 60)

    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
