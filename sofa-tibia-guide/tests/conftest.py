"""Pytest fixtures for tibia guide simulation tests."""

import os
import sys
import tempfile
from pathlib import Path

# Ensure SOFA paths are set before any imports try to load Sofa
_sofa_root = os.environ.get("SOFA_ROOT", "/home/coreyt/sofa/SOFA_v24.06.00_Linux")
_sofa_python_path = f"{_sofa_root}/plugins/SofaPython3/lib/python3/site-packages"
if _sofa_python_path not in sys.path and os.path.exists(_sofa_python_path):
    sys.path.insert(0, _sofa_python_path)

import numpy as np
import pytest
import trimesh


@pytest.fixture
def simple_cube_stl(tmp_path: Path) -> Path:
    """Create a minimal valid STL cube for unit tests.

    Returns path to a 10x10x10mm cube centered at origin.
    """
    cube = trimesh.creation.box(extents=[10, 10, 10])
    stl_path = tmp_path / "cube.stl"
    cube.export(str(stl_path))
    return stl_path


@pytest.fixture
def simple_cylinder_stl(tmp_path: Path) -> Path:
    """Create a simple cylinder STL for testing.

    Returns path to a cylinder (radius=5mm, height=20mm).
    """
    cylinder = trimesh.creation.cylinder(radius=5, height=20)
    stl_path = tmp_path / "cylinder.stl"
    cylinder.export(str(stl_path))
    return stl_path


@pytest.fixture
def non_watertight_stl(tmp_path: Path) -> Path:
    """Create a non-watertight mesh (open box) for testing validation.

    Returns path to a box with one face removed.
    """
    # Create box and remove one face
    box = trimesh.creation.box(extents=[10, 10, 10])
    # Remove faces on one side (indices where normal points in +Z)
    normals = box.face_normals
    keep_mask = normals[:, 2] < 0.9  # Keep all but top faces
    box.update_faces(keep_mask)

    stl_path = tmp_path / "open_box.stl"
    box.export(str(stl_path))
    return stl_path


@pytest.fixture
def identity_transform() -> np.ndarray:
    """4x4 identity transformation matrix."""
    return np.eye(4)


@pytest.fixture
def sample_transform() -> np.ndarray:
    """Known rotation + translation for testing.

    90-degree rotation around Z-axis, translated by [10, 20, 30].
    """
    transform = np.array([
        [0, -1, 0, 10],
        [1, 0, 0, 20],
        [0, 0, 1, 30],
        [0, 0, 0, 1]
    ], dtype=np.float64)
    return transform


@pytest.fixture
def sample_quaternion() -> np.ndarray:
    """Quaternion corresponding to sample_transform rotation.

    90-degree rotation around Z-axis as (w, x, y, z).
    """
    # 90 degrees around Z: cos(45°), 0, 0, sin(45°)
    angle = np.pi / 2
    return np.array([
        np.cos(angle / 2),  # w
        0,                   # x
        0,                   # y
        np.sin(angle / 2)   # z
    ])


@pytest.fixture
def sample_position() -> np.ndarray:
    """Translation component of sample_transform."""
    return np.array([10.0, 20.0, 30.0])


@pytest.fixture
def reference_pose() -> dict:
    """Reference pose for comparison tests (simulating 6dof-mc ICP result)."""
    return {
        "position": np.array([5.0, 10.0, 15.0]),
        "quaternion": np.array([1.0, 0.0, 0.0, 0.0]),  # identity rotation
    }


def pytest_configure(config):
    """Register custom markers."""
    config.addinivalue_line(
        "markers", "sofa: marks tests as requiring SOFA installation"
    )
    config.addinivalue_line(
        "markers", "slow: marks tests as slow-running"
    )


def sofa_available() -> bool:
    """Check if SOFA and SofaPython3 are available."""
    try:
        import Sofa
        return True
    except ImportError:
        return False


@pytest.fixture
def skip_without_sofa():
    """Skip test if SOFA is not installed."""
    if not sofa_available():
        pytest.skip("SOFA not installed")
