"""SOFA-based physics simulation for surgical cutting guide placement."""

__version__ = "0.1.0"

from tibia_guide.seating import SeatingResult, find_seating_pose

__all__ = ["find_seating_pose", "SeatingResult"]

# Optional visualization exports (requires pyvista)
try:
    from tibia_guide.visualization import (
        visualize_meshes,
        visualize_seating_result,
        visualize_side_by_side,
        save_seating_screenshot,
    )

    __all__.extend([
        "visualize_meshes",
        "visualize_seating_result",
        "visualize_side_by_side",
        "save_seating_screenshot",
    ])
except ImportError:
    pass  # PyVista not installed
