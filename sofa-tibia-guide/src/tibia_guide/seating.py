"""High-level API for finding guide seating position.

This module provides a simple interface for external applications to find
where a surgical cutting guide seats on a tibia using physics simulation.

Example:
    from tibia_guide import find_seating_pose

    result = find_seating_pose(
        tibia_mesh="path/to/tibia.stl",
        guide_mesh="path/to/guide.stl",
    )
    print(f"Guide seated at: {result.pose.position}")
"""

import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Union

import numpy as np
import trimesh

from tibia_guide.metrics.pose import Pose


@dataclass
class SeatingResult:
    """Result of seating simulation.

    Attributes:
        pose: Final guide pose in ORIGINAL coordinate frame.
        converged: Whether simulation reached equilibrium.
        steps: Number of simulation steps taken.
        final_gap_mm: Distance from guide bottom to tibia top surface (mm).
            Positive means guide is above surface, negative means penetration.
    """

    pose: Pose
    converged: bool
    steps: int
    final_gap_mm: float


def find_seating_pose(
    tibia_mesh: Union[trimesh.Trimesh, str, Path],
    guide_mesh: Union[trimesh.Trimesh, str, Path],
    max_simulation_time: float = 2.0,
    convergence_threshold: float = 0.01,
    convergence_check_interval: int = 100,
    dt: float = 0.001,
    guide_mass_kg: float = 0.1,
    collision_simplification: bool = True,
    initial_gap_mm: float = 15.0,
    gui: bool = False,
) -> SeatingResult:
    """Find guide seating position on tibia using physics simulation.

    Runs a SOFA rigid body simulation with gravity and frictional contact
    until the guide reaches equilibrium on the tibia surface.

    Args:
        tibia_mesh: Tibia mesh as trimesh.Trimesh object or path to STL file.
        guide_mesh: Guide mesh as trimesh.Trimesh object or path to STL file.
        max_simulation_time: Maximum simulation time in seconds. Default 2.0.
        convergence_threshold: Position change threshold for convergence in mm.
            Simulation stops when position changes less than this between checks.
            Default 0.01 mm.
        convergence_check_interval: Check convergence every N steps. Default 100.
        dt: Simulation timestep in seconds. Default 0.001 (1ms).
        guide_mass_kg: Guide mass in kilograms. Default 0.1 kg.
        collision_simplification: If True, use convex hull for tibia and OBB
            for guide to improve simulation stability. Default True.
        initial_gap_mm: Minimum initial gap between guide and tibia in mm.
            Guide is raised if needed to ensure this gap. Default 15.0 mm.
        gui: If True, run with SOFA's Qt GUI for interactive visualization.
            The simulation runs interactively and returns when the window is closed.
            Default False.

    Returns:
        SeatingResult with pose in original coordinate frame.

    Raises:
        ImportError: If SOFA is not installed.
        FileNotFoundError: If mesh file paths don't exist.
        ValueError: If meshes are invalid.

    Example:
        >>> result = find_seating_pose("tibia.stl", "guide.stl")
        >>> print(f"Position: {result.pose.position}")
        >>> print(f"Converged: {result.converged} in {result.steps} steps")
    """
    # Import SOFA (may raise ImportError if not installed)
    try:
        import Sofa
        import Sofa.Simulation
    except ImportError as e:
        raise ImportError(
            "SOFA is not installed. Install it with: conda install -c sofa-framework sofa"
        ) from e

    # Import scene creation functions
    from scenes.mvm_seating import (
        create_scene,
        extract_guide_pose,
        normalize_meshes,
    )

    # Load meshes if paths provided
    tibia_trimesh = _load_mesh(tibia_mesh, "tibia")
    guide_trimesh = _load_mesh(guide_mesh, "guide")

    # Prepare collision meshes
    if collision_simplification:
        tibia_collision = tibia_trimesh.convex_hull
        guide_collision = guide_trimesh.bounding_box_oriented.to_mesh()
    else:
        tibia_collision = tibia_trimesh.copy()
        guide_collision = guide_trimesh.copy()

    # Store original guide centroid for reference
    original_guide_centroid = guide_trimesh.centroid.copy()

    # Normalize coordinates (SOFA has numerical issues with large values)
    norm_info = normalize_meshes(tibia_collision, guide_collision)

    # Ensure guide starts above tibia with sufficient gap
    current_gap = guide_collision.bounds[0][2] - tibia_collision.bounds[1][2]
    if current_gap < initial_gap_mm:
        lift = initial_gap_mm - current_gap
        guide_collision.apply_translation([0, 0, lift])

    # Save to temp files (SOFA's MeshSTLLoader requires file paths)
    with tempfile.TemporaryDirectory(prefix="sofa_seating_") as temp_dir:
        temp_path = Path(temp_dir)
        tibia_path = temp_path / "tibia.stl"
        guide_path = temp_path / "guide.stl"

        tibia_collision.export(str(tibia_path))
        guide_collision.export(str(guide_path))

        # Create and initialize SOFA scene
        # Note: guide_initial_position must be [0,0,0] because RigidMapping
        # adds the rigid body position to mesh vertices, and our mesh
        # is already at world coordinates.
        root = create_scene(
            tibia_mesh_path=str(tibia_path),
            guide_mesh_path=str(guide_path),
            guide_initial_position=[0.0, 0.0, 0.0],
            guide_initial_quaternion=[1.0, 0.0, 0.0, 0.0],
            guide_mass_kg=guide_mass_kg,
            dt=dt,
            gui=gui,
        )
        Sofa.Simulation.init(root)

        # Run simulation
        if gui:
            # Run with interactive GUI
            import Sofa.Gui

            Sofa.Gui.GUIManager.Init("main", "qglviewer")
            Sofa.Gui.GUIManager.createGUI(root, __file__)
            Sofa.Gui.GUIManager.SetDimension(1200, 800)
            Sofa.Gui.GUIManager.MainLoop(root)
            Sofa.Gui.GUIManager.closeGUI()

            # After GUI closes, extract final state
            final_pose = extract_guide_pose(root)
            # In GUI mode, we don't track convergence
            converged = True
            final_step = -1  # Unknown number of steps in GUI mode
        else:
            # Run headless until convergence or timeout
            max_steps = int(max_simulation_time / dt)
            converged = False
            prev_pos = np.array([np.inf, np.inf, np.inf])
            final_step = 0

            for step in range(max_steps):
                Sofa.Simulation.animate(root, dt)
                final_step = step

                # Check convergence periodically
                if step > 0 and step % convergence_check_interval == 0:
                    pose = extract_guide_pose(root)
                    pos_change = np.linalg.norm(pose.position - prev_pos)
                    if pos_change < convergence_threshold:
                        converged = True
                        break
                    prev_pos = pose.position.copy()

            # Extract final pose
            final_pose = extract_guide_pose(root)

        # Convert to world coordinates
        # pose.position is the DELTA from initial position (which was [0,0,0])
        # World position = mesh_centroid + pose.position
        guide_centroid_normalized = guide_collision.centroid
        guide_world_pos_normalized = guide_centroid_normalized + final_pose.position

        # Convert back to original coordinate frame
        original_pos = guide_world_pos_normalized + norm_info["total_offset"]

        # Calculate final gap (guide bottom to tibia top)
        guide_half_height = (
            guide_collision.bounds[1][2] - guide_collision.bounds[0][2]
        ) / 2
        guide_bottom_z_normalized = guide_world_pos_normalized[2] - guide_half_height
        # In normalized coords, tibia top is at Z=0
        final_gap = guide_bottom_z_normalized

        return SeatingResult(
            pose=Pose(position=original_pos, quaternion=final_pose.quaternion),
            converged=converged,
            steps=final_step + 1,
            final_gap_mm=float(final_gap),
        )


def _load_mesh(
    mesh: Union[trimesh.Trimesh, str, Path], name: str
) -> trimesh.Trimesh:
    """Load mesh from path or return if already a Trimesh.

    Args:
        mesh: Trimesh object or path to STL file.
        name: Name for error messages.

    Returns:
        trimesh.Trimesh object.

    Raises:
        FileNotFoundError: If path doesn't exist.
        ValueError: If mesh is invalid.
    """
    if isinstance(mesh, trimesh.Trimesh):
        return mesh

    path = Path(mesh)
    if not path.exists():
        raise FileNotFoundError(f"{name} mesh not found: {path}")

    loaded = trimesh.load(str(path))
    if not isinstance(loaded, trimesh.Trimesh):
        raise ValueError(f"{name} mesh is not a valid Trimesh: {type(loaded)}")

    return loaded
