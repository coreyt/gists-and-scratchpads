#!/usr/bin/env python
"""Headless experiment runner for disturbance simulations.

Usage:
    python run_disturbance.py --config config.yaml
    python run_disturbance.py --tibia path/to/tibia.stl --guide path/to/guide.stl
"""

import argparse
import json
import sys
from datetime import datetime
from pathlib import Path
from typing import List, Optional

import numpy as np
import pandas as pd

from tibia_guide.config import SimulationConfig
from tibia_guide.metrics.pose import Pose, compare_poses


def run_simulation(
    config: SimulationConfig,
    reference_pose: Optional[Pose] = None,
    verbose: bool = True,
) -> dict:
    """Run a single simulation and return results.

    Args:
        config: Simulation configuration.
        reference_pose: Optional reference pose for comparison.
        verbose: Print progress messages.

    Returns:
        Dictionary with simulation results.
    """
    try:
        import Sofa
        import Sofa.Simulation
    except ImportError:
        print("ERROR: SOFA is not installed.", file=sys.stderr)
        print("Install via: conda install -c conda-forge sofa", file=sys.stderr)
        sys.exit(1)

    from scenes.mvm_seating import create_scene, extract_guide_pose, add_constant_force

    if verbose:
        print(f"Creating scene...")
        print(f"  Tibia mesh: {config.tibia.collision_or_visual}")
        print(f"  Guide mesh: {config.guide.collision_or_visual}")

    # Create scene
    root = create_scene(
        tibia_mesh_path=config.tibia.collision_or_visual,
        guide_mesh_path=config.guide.collision_or_visual,
        guide_initial_position=config.guide_initial_pose.position,
        guide_initial_quaternion=config.guide_initial_pose.quaternion,
        guide_mass_kg=config.guide_mass_kg,
        friction_coefficient=config.solver.friction_coefficient,
        gravity=config.solver.gravity,
        dt=config.solver.dt,
    )

    # Add disturbance if configured
    if any(f != 0 for f in config.disturbance.force):
        add_constant_force(
            root,
            force=config.disturbance.force,
            torque=config.disturbance.torque,
        )

    # Initialize
    Sofa.Simulation.init(root)

    # Calculate number of steps
    num_steps = int(config.duration_seconds / config.solver.dt)
    sample_interval = config.output.sample_interval

    if verbose:
        print(f"Running simulation: {num_steps} steps, dt={config.solver.dt}s")

    # Storage for time series
    times: List[float] = []
    positions: List[List[float]] = []
    quaternions: List[List[float]] = []

    # Run simulation
    for step in range(num_steps):
        Sofa.Simulation.animate(root, config.solver.dt)

        # Sample pose at interval
        if step % sample_interval == 0:
            pose = extract_guide_pose(root)
            t = step * config.solver.dt

            times.append(t)
            positions.append(pose.position.tolist())
            quaternions.append(pose.quaternion.tolist())

            if verbose and step % (num_steps // 10) == 0:
                print(f"  Step {step}/{num_steps} ({100*step/num_steps:.0f}%)")

    # Extract final pose
    final_pose = extract_guide_pose(root)

    if verbose:
        print(f"Simulation complete.")
        print(f"  Final position: {final_pose.position}")
        print(f"  Final quaternion: {final_pose.quaternion}")

    # Build results
    results = {
        "config": config.to_dict(),
        "timestamp": datetime.now().isoformat(),
        "num_steps": num_steps,
        "final_pose": final_pose.to_dict(),
        "timeseries": {
            "times": times,
            "positions": positions,
            "quaternions": quaternions,
        },
    }

    # Compare to reference if provided
    if reference_pose is not None:
        comparison = compare_poses(final_pose, reference_pose)
        results["comparison"] = {
            "reference_pose": reference_pose.to_dict(),
            "translation_error_mm": comparison["translation_error_mm"],
            "rotation_error_deg": comparison["rotation_error_deg"],
        }
        if verbose:
            print(f"  Comparison to reference:")
            print(f"    Translation error: {comparison['translation_error_mm']:.4f} mm")
            print(f"    Rotation error: {comparison['rotation_error_deg']:.4f} deg")

    return results


def save_results(results: dict, config: SimulationConfig) -> None:
    """Save simulation results to files.

    Args:
        results: Simulation results dictionary.
        config: Simulation configuration.
    """
    output_dir = Path(config.output.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    # Save pose time series to CSV
    csv_path = output_dir / config.output.pose_csv
    ts = results["timeseries"]

    df = pd.DataFrame({
        "time_s": ts["times"],
        "x_mm": [p[0] for p in ts["positions"]],
        "y_mm": [p[1] for p in ts["positions"]],
        "z_mm": [p[2] for p in ts["positions"]],
        "qw": [q[0] for q in ts["quaternions"]],
        "qx": [q[1] for q in ts["quaternions"]],
        "qy": [q[2] for q in ts["quaternions"]],
        "qz": [q[3] for q in ts["quaternions"]],
    })
    df.to_csv(csv_path, index=False)
    print(f"Saved pose time series to: {csv_path}")

    # Save full results to JSON
    json_path = output_dir / config.output.pose_json
    with open(json_path, "w") as f:
        json.dump(results, f, indent=2)
    print(f"Saved full results to: {json_path}")


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description="Run disturbance simulation experiment"
    )
    parser.add_argument(
        "--config",
        type=str,
        help="Path to YAML configuration file",
    )
    parser.add_argument(
        "--tibia",
        type=str,
        help="Path to tibia mesh (overrides config)",
    )
    parser.add_argument(
        "--guide",
        type=str,
        help="Path to guide mesh (overrides config)",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=2.0,
        help="Simulation duration in seconds (default: 2.0)",
    )
    parser.add_argument(
        "--output-dir",
        type=str,
        default="outputs",
        help="Output directory (default: outputs)",
    )
    parser.add_argument(
        "--reference-pose",
        type=str,
        help="Path to JSON file with reference pose for comparison",
    )
    parser.add_argument(
        "--quiet",
        action="store_true",
        help="Suppress progress output",
    )

    args = parser.parse_args()

    # Load or create configuration
    if args.config:
        config = SimulationConfig.from_yaml(args.config)
    else:
        config = SimulationConfig()

    # Override from command line
    if args.tibia:
        config.tibia.visual_path = args.tibia
    if args.guide:
        config.guide.visual_path = args.guide
    if args.duration:
        config.duration_seconds = args.duration
    if args.output_dir:
        config.output.output_dir = args.output_dir

    # Validate required inputs
    if not config.tibia.visual_path:
        print("ERROR: Tibia mesh path required (--tibia or in config)", file=sys.stderr)
        sys.exit(1)
    if not config.guide.visual_path:
        print("ERROR: Guide mesh path required (--guide or in config)", file=sys.stderr)
        sys.exit(1)

    # Load reference pose if provided
    reference_pose = None
    if args.reference_pose:
        with open(args.reference_pose) as f:
            ref_data = json.load(f)
        reference_pose = Pose.from_dict(ref_data)

    # Run simulation
    results = run_simulation(
        config,
        reference_pose=reference_pose,
        verbose=not args.quiet,
    )

    # Save results
    save_results(results, config)

    # Exit with error if comparison failed tolerance
    if "comparison" in results:
        trans_err = results["comparison"]["translation_error_mm"]
        rot_err = results["comparison"]["rotation_error_deg"]
        if trans_err > 0.1 or rot_err > 0.1:
            print(f"WARNING: Pose difference exceeds tolerance", file=sys.stderr)
            sys.exit(2)

    sys.exit(0)


if __name__ == "__main__":
    main()
