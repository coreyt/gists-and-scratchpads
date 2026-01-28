"""Visualization utilities for seating simulation results.

This module provides PyVista-based visualization for viewing tibia and guide
meshes before and after seating simulation.

Example:
    from tibia_guide.visualization import visualize_seating_result

    result = find_seating_pose(tibia_mesh, guide_mesh)
    visualize_seating_result(
        tibia_mesh=tibia_mesh,
        guide_mesh=guide_mesh,
        result=result,
    )
"""

from pathlib import Path
from typing import Optional, Union

import numpy as np
import trimesh

try:
    import pyvista as pv
    PYVISTA_AVAILABLE = True
except ImportError:
    PYVISTA_AVAILABLE = False


def _check_pyvista():
    """Raise ImportError if PyVista is not available."""
    if not PYVISTA_AVAILABLE:
        raise ImportError(
            "PyVista is required for visualization. "
            "Install it with: pip install pyvista"
        )


def _load_mesh(mesh: Union[trimesh.Trimesh, str, Path]) -> trimesh.Trimesh:
    """Load mesh from path or return if already a Trimesh."""
    if isinstance(mesh, trimesh.Trimesh):
        return mesh.copy()
    return trimesh.load(str(mesh))


def _trimesh_to_pyvista(mesh: trimesh.Trimesh) -> "pv.PolyData":
    """Convert trimesh to PyVista PolyData."""
    _check_pyvista()
    faces = np.column_stack([
        np.full(len(mesh.faces), 3),
        mesh.faces
    ]).ravel()
    return pv.PolyData(mesh.vertices, faces)


def visualize_meshes(
    tibia_mesh: Union[trimesh.Trimesh, str, Path],
    guide_mesh: Union[trimesh.Trimesh, str, Path],
    guide_position: Optional[np.ndarray] = None,
    guide_quaternion: Optional[np.ndarray] = None,
    tibia_color: str = "ivory",
    guide_color: str = "steelblue",
    tibia_opacity: float = 0.7,
    guide_opacity: float = 1.0,
    window_size: tuple = (1200, 800),
    title: str = "Tibia Guide Visualization",
    show_axes: bool = True,
    background_color: str = "white",
) -> None:
    """Visualize tibia and guide meshes interactively.

    Args:
        tibia_mesh: Tibia mesh (Trimesh or path to STL).
        guide_mesh: Guide mesh (Trimesh or path to STL).
        guide_position: Optional position offset [x, y, z] to apply to guide.
        guide_quaternion: Optional rotation quaternion [w, x, y, z] for guide.
        tibia_color: Color for tibia mesh.
        guide_color: Color for guide mesh.
        tibia_opacity: Opacity for tibia (0-1).
        guide_opacity: Opacity for guide (0-1).
        window_size: Window dimensions (width, height).
        title: Window title.
        show_axes: Whether to show coordinate axes.
        background_color: Background color.
    """
    _check_pyvista()

    # Load meshes
    tibia = _load_mesh(tibia_mesh)
    guide = _load_mesh(guide_mesh)

    # Apply transform to guide if provided
    if guide_position is not None or guide_quaternion is not None:
        transform = np.eye(4)

        if guide_quaternion is not None:
            # Convert quaternion [w, x, y, z] to rotation matrix
            w, x, y, z = guide_quaternion
            rot = trimesh.transformations.quaternion_matrix([w, x, y, z])
            transform[:3, :3] = rot[:3, :3]

        if guide_position is not None:
            transform[:3, 3] = guide_position

        guide.apply_transform(transform)

    # Convert to PyVista
    tibia_pv = _trimesh_to_pyvista(tibia)
    guide_pv = _trimesh_to_pyvista(guide)

    # Create plotter
    plotter = pv.Plotter(window_size=window_size, title=title)
    plotter.set_background(background_color)

    # Add meshes
    plotter.add_mesh(
        tibia_pv,
        color=tibia_color,
        opacity=tibia_opacity,
        label="Tibia",
    )
    plotter.add_mesh(
        guide_pv,
        color=guide_color,
        opacity=guide_opacity,
        label="Guide",
    )

    if show_axes:
        plotter.add_axes()

    plotter.add_legend()
    plotter.show()


def visualize_seating_result(
    tibia_mesh: Union[trimesh.Trimesh, str, Path],
    guide_mesh: Union[trimesh.Trimesh, str, Path],
    result: "SeatingResult",
    show_initial: bool = True,
    initial_guide_color: str = "lightcoral",
    final_guide_color: str = "steelblue",
    tibia_color: str = "ivory",
    window_size: tuple = (1200, 800),
) -> None:
    """Visualize seating simulation result with before/after comparison.

    Args:
        tibia_mesh: Tibia mesh (Trimesh or path to STL).
        guide_mesh: Guide mesh (Trimesh or path to STL).
        result: SeatingResult from find_seating_pose().
        show_initial: If True, show initial guide position (semi-transparent).
        initial_guide_color: Color for initial guide position.
        final_guide_color: Color for final guide position.
        tibia_color: Color for tibia mesh.
        window_size: Window dimensions (width, height).
    """
    _check_pyvista()

    # Load meshes
    tibia = _load_mesh(tibia_mesh)
    guide_initial = _load_mesh(guide_mesh)
    guide_final = _load_mesh(guide_mesh)

    # Apply final pose to guide
    # The result.pose.position is the final centroid position
    # We need to translate the guide from its initial centroid to the final position
    initial_centroid = guide_final.centroid.copy()
    translation = result.pose.position - initial_centroid

    # Apply rotation if not identity
    if result.pose.quaternion is not None:
        w, x, y, z = result.pose.quaternion
        # Check if rotation is significant (not identity)
        if abs(w - 1.0) > 1e-6 or abs(x) > 1e-6 or abs(y) > 1e-6 or abs(z) > 1e-6:
            rot_matrix = trimesh.transformations.quaternion_matrix([w, x, y, z])
            # Rotate around centroid
            guide_final.apply_transform(
                trimesh.transformations.translation_matrix(-initial_centroid)
            )
            guide_final.apply_transform(rot_matrix)
            guide_final.apply_transform(
                trimesh.transformations.translation_matrix(result.pose.position)
            )
        else:
            guide_final.apply_translation(translation)
    else:
        guide_final.apply_translation(translation)

    # Convert to PyVista
    tibia_pv = _trimesh_to_pyvista(tibia)
    guide_initial_pv = _trimesh_to_pyvista(guide_initial)
    guide_final_pv = _trimesh_to_pyvista(guide_final)

    # Create plotter
    plotter = pv.Plotter(window_size=window_size, title="Seating Simulation Result")
    plotter.set_background("white")

    # Add tibia
    plotter.add_mesh(
        tibia_pv,
        color=tibia_color,
        opacity=0.7,
        label="Tibia",
    )

    # Add initial guide (semi-transparent)
    if show_initial:
        plotter.add_mesh(
            guide_initial_pv,
            color=initial_guide_color,
            opacity=0.3,
            label="Guide (initial)",
        )

    # Add final guide
    plotter.add_mesh(
        guide_final_pv,
        color=final_guide_color,
        opacity=1.0,
        label="Guide (final)",
    )

    # Add info text
    info_text = (
        f"Converged: {result.converged}\n"
        f"Steps: {result.steps}\n"
        f"Final gap: {result.final_gap_mm:.2f} mm"
    )
    plotter.add_text(info_text, position="upper_left", font_size=10)

    plotter.add_axes()
    plotter.add_legend()
    plotter.show()


def visualize_side_by_side(
    tibia_mesh: Union[trimesh.Trimesh, str, Path],
    guide_mesh: Union[trimesh.Trimesh, str, Path],
    result: "SeatingResult",
    window_size: tuple = (1600, 800),
) -> None:
    """Show before and after views side by side.

    Args:
        tibia_mesh: Tibia mesh (Trimesh or path to STL).
        guide_mesh: Guide mesh (Trimesh or path to STL).
        result: SeatingResult from find_seating_pose().
        window_size: Window dimensions (width, height).
    """
    _check_pyvista()

    # Load meshes
    tibia = _load_mesh(tibia_mesh)
    guide_initial = _load_mesh(guide_mesh)
    guide_final = _load_mesh(guide_mesh)

    # Apply final pose to guide
    initial_centroid = guide_final.centroid.copy()
    translation = result.pose.position - initial_centroid
    guide_final.apply_translation(translation)

    # Convert to PyVista
    tibia_pv = _trimesh_to_pyvista(tibia)
    guide_initial_pv = _trimesh_to_pyvista(guide_initial)
    guide_final_pv = _trimesh_to_pyvista(guide_final)

    # Create side-by-side plotter
    plotter = pv.Plotter(
        shape=(1, 2),
        window_size=window_size,
        title="Seating Simulation: Before / After",
    )

    # Left: Before
    plotter.subplot(0, 0)
    plotter.set_background("white")
    plotter.add_text("BEFORE", position="upper_edge", font_size=14)
    plotter.add_mesh(tibia_pv.copy(), color="ivory", opacity=0.7)
    plotter.add_mesh(guide_initial_pv, color="lightcoral", opacity=1.0)
    plotter.add_axes()

    # Right: After
    plotter.subplot(0, 1)
    plotter.set_background("white")
    plotter.add_text("AFTER", position="upper_edge", font_size=14)
    plotter.add_mesh(tibia_pv, color="ivory", opacity=0.7)
    plotter.add_mesh(guide_final_pv, color="steelblue", opacity=1.0)
    plotter.add_axes()

    # Add result info
    info_text = (
        f"Converged: {result.converged} | "
        f"Steps: {result.steps} | "
        f"Gap: {result.final_gap_mm:.2f} mm"
    )
    plotter.add_text(info_text, position="lower_edge", font_size=10)

    # Link cameras so both views rotate together
    plotter.link_views()

    plotter.show()


def save_seating_screenshot(
    tibia_mesh: Union[trimesh.Trimesh, str, Path],
    guide_mesh: Union[trimesh.Trimesh, str, Path],
    result: "SeatingResult",
    output_path: Union[str, Path],
    show_initial: bool = True,
    camera_position: str = "iso",
    window_size: tuple = (1200, 800),
) -> None:
    """Save seating result visualization to an image file.

    Useful for headless environments (servers, CI/CD) where interactive
    display is not available.

    Args:
        tibia_mesh: Tibia mesh (Trimesh or path to STL).
        guide_mesh: Guide mesh (Trimesh or path to STL).
        result: SeatingResult from find_seating_pose().
        output_path: Path to save the image (PNG, JPG, etc.).
        show_initial: If True, show initial guide position (semi-transparent).
        camera_position: Camera angle ('iso', 'xy', 'xz', 'yz', or custom).
        window_size: Image dimensions (width, height).
    """
    _check_pyvista()

    # Load meshes
    tibia = _load_mesh(tibia_mesh)
    guide_initial = _load_mesh(guide_mesh)
    guide_final = _load_mesh(guide_mesh)

    # Apply final pose to guide
    initial_centroid = guide_final.centroid.copy()
    translation = result.pose.position - initial_centroid
    guide_final.apply_translation(translation)

    # Convert to PyVista
    tibia_pv = _trimesh_to_pyvista(tibia)
    guide_initial_pv = _trimesh_to_pyvista(guide_initial)
    guide_final_pv = _trimesh_to_pyvista(guide_final)

    # Create off-screen plotter
    plotter = pv.Plotter(off_screen=True, window_size=window_size)
    plotter.set_background("white")

    # Add tibia
    plotter.add_mesh(tibia_pv, color="ivory", opacity=0.7, label="Tibia")

    # Add initial guide (semi-transparent)
    if show_initial:
        plotter.add_mesh(
            guide_initial_pv,
            color="lightcoral",
            opacity=0.3,
            label="Guide (initial)",
        )

    # Add final guide
    plotter.add_mesh(
        guide_final_pv,
        color="steelblue",
        opacity=1.0,
        label="Guide (final)",
    )

    # Add info text
    info_text = (
        f"Converged: {result.converged}\n"
        f"Steps: {result.steps}\n"
        f"Gap: {result.final_gap_mm:.2f} mm"
    )
    plotter.add_text(info_text, position="upper_left", font_size=10)

    plotter.add_axes()
    plotter.add_legend()
    plotter.camera_position = camera_position

    # Save screenshot
    plotter.screenshot(str(output_path))
    plotter.close()
