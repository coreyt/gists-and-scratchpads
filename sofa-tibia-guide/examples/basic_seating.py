#!/usr/bin/env python
"""Example: Find guide seating position on tibia.

This example demonstrates the simplest usage of the find_seating_pose API.

Requirements:
    - SOFA with SofaPython3 installed
    - SOFA_ROOT environment variable set (or edit default path below)

Usage:
    export SOFA_ROOT=/path/to/SOFA
    export LD_LIBRARY_PATH="$SOFA_ROOT/lib:$LD_LIBRARY_PATH"
    python examples/basic_seating.py <tibia.stl> <guide.stl>
"""

import argparse
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

from tibia_guide import find_seating_pose


def main():
    parser = argparse.ArgumentParser(
        description="Find guide seating position on tibia using physics simulation."
    )
    parser.add_argument("tibia_mesh", help="Path to tibia STL file")
    parser.add_argument("guide_mesh", help="Path to guide STL file")
    parser.add_argument(
        "--max-time",
        type=float,
        default=2.0,
        help="Maximum simulation time in seconds (default: 2.0)",
    )
    parser.add_argument(
        "--convergence",
        type=float,
        default=0.01,
        help="Convergence threshold in mm (default: 0.01)",
    )
    args = parser.parse_args()

    # Validate inputs
    if not Path(args.tibia_mesh).exists():
        print(f"Error: Tibia mesh not found: {args.tibia_mesh}")
        return 1
    if not Path(args.guide_mesh).exists():
        print(f"Error: Guide mesh not found: {args.guide_mesh}")
        return 1

    print("=" * 60)
    print("Finding guide seating position")
    print("=" * 60)
    print(f"Tibia mesh: {args.tibia_mesh}")
    print(f"Guide mesh: {args.guide_mesh}")
    print(f"Max simulation time: {args.max_time}s")
    print(f"Convergence threshold: {args.convergence}mm")
    print()

    # Run simulation
    result = find_seating_pose(
        tibia_mesh=args.tibia_mesh,
        guide_mesh=args.guide_mesh,
        max_simulation_time=args.max_time,
        convergence_threshold=args.convergence,
    )

    # Print results
    print("Result:")
    print(f"  Position: [{result.pose.position[0]:.2f}, {result.pose.position[1]:.2f}, {result.pose.position[2]:.2f}] mm")
    print(f"  Quaternion (w,x,y,z): [{result.pose.quaternion[0]:.4f}, {result.pose.quaternion[1]:.4f}, {result.pose.quaternion[2]:.4f}, {result.pose.quaternion[3]:.4f}]")
    print(f"  Converged: {result.converged}")
    print(f"  Simulation steps: {result.steps}")
    print(f"  Final gap to tibia: {result.final_gap_mm:.2f} mm")
    print("=" * 60)

    return 0


if __name__ == "__main__":
    sys.exit(main())
