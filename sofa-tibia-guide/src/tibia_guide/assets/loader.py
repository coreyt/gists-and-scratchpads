"""Mesh loading and validation utilities."""

from pathlib import Path
from typing import Union

import numpy as np
import trimesh


class MeshLoadError(Exception):
    """Raised when mesh loading fails."""

    pass


class MeshValidationError(Exception):
    """Raised when mesh validation fails."""

    pass


SUPPORTED_EXTENSIONS = {".stl", ".obj", ".ply"}


def load_mesh(
    path: Union[str, Path],
    validate: bool = False,
    require_watertight: bool = True,
) -> trimesh.Trimesh:
    """Load a mesh from file.

    Args:
        path: Path to mesh file (STL, OBJ, or PLY).
        validate: If True, validate mesh after loading.
        require_watertight: If validating, require mesh to be watertight.

    Returns:
        Loaded trimesh.Trimesh object.

    Raises:
        MeshLoadError: If file not found, unsupported format, or corrupt.
        MeshValidationError: If validation enabled and mesh is invalid.
    """
    path = Path(path)

    # Check file exists
    if not path.exists():
        raise MeshLoadError(f"Mesh file not found: {path}")

    # Check extension
    ext = path.suffix.lower()
    if ext not in SUPPORTED_EXTENSIONS:
        raise MeshLoadError(
            f"Unsupported mesh format '{ext}'. "
            f"Supported formats: {', '.join(SUPPORTED_EXTENSIONS)}"
        )

    # Load mesh
    try:
        mesh = trimesh.load(str(path), force="mesh")
    except Exception as e:
        raise MeshLoadError(f"Failed to load mesh from {path}: {e}") from e

    # Ensure we got a Trimesh, not a Scene
    if isinstance(mesh, trimesh.Scene):
        # Extract the first geometry
        geometries = list(mesh.geometry.values())
        if not geometries:
            raise MeshLoadError(f"No geometry found in {path}")
        mesh = geometries[0]

    if not isinstance(mesh, trimesh.Trimesh):
        raise MeshLoadError(f"Failed to load as triangle mesh: {path}")

    # Validate if requested
    if validate:
        validate_mesh(mesh, require_watertight=require_watertight)

    return mesh


def validate_mesh(
    mesh: trimesh.Trimesh,
    require_watertight: bool = True,
    check_degenerate: bool = True,
    return_stats: bool = False,
) -> Union[None, dict]:
    """Validate mesh integrity.

    Args:
        mesh: Trimesh object to validate.
        require_watertight: Require mesh to be watertight (closed).
        check_degenerate: Check for degenerate (zero-area) faces.
        return_stats: If True, return mesh statistics dict.

    Returns:
        None if return_stats is False, otherwise dict with mesh stats.

    Raises:
        MeshValidationError: If validation fails.
    """
    stats = {
        "num_vertices": len(mesh.vertices),
        "num_faces": len(mesh.faces),
        "is_watertight": mesh.is_watertight,
        "bounds": mesh.bounds.tolist() if mesh.bounds is not None else None,
    }

    # Check watertight
    if require_watertight and not mesh.is_watertight:
        raise MeshValidationError(
            "Mesh is not watertight. Use require_watertight=False to allow."
        )

    # Check for degenerate faces
    if check_degenerate:
        areas = mesh.area_faces
        degenerate_count = np.sum(areas < 1e-10)
        stats["degenerate_faces"] = int(degenerate_count)

        if degenerate_count > 0:
            raise MeshValidationError(
                f"Mesh has {degenerate_count} degenerate faces (zero area)."
            )

    if return_stats:
        return stats

    return None
