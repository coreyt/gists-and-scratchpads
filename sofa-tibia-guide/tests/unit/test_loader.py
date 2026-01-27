"""Unit tests for mesh loading functionality."""

from pathlib import Path

import numpy as np
import pytest

from tibia_guide.assets.loader import (
    load_mesh,
    validate_mesh,
    MeshLoadError,
    MeshValidationError,
)


class TestLoadMesh:
    """Tests for load_mesh function."""

    def test_load_valid_stl(self, simple_cube_stl: Path):
        """Load a valid STL file successfully."""
        mesh = load_mesh(simple_cube_stl)

        assert mesh is not None
        assert len(mesh.vertices) > 0
        assert len(mesh.faces) > 0

    def test_load_returns_trimesh(self, simple_cube_stl: Path):
        """Returned object is a trimesh.Trimesh instance."""
        import trimesh

        mesh = load_mesh(simple_cube_stl)

        assert isinstance(mesh, trimesh.Trimesh)

    def test_load_missing_file_raises(self, tmp_path: Path):
        """Raise MeshLoadError for non-existent file."""
        missing = tmp_path / "does_not_exist.stl"

        with pytest.raises(MeshLoadError) as exc_info:
            load_mesh(missing)

        assert "not found" in str(exc_info.value).lower()

    def test_load_invalid_extension_raises(self, tmp_path: Path):
        """Raise MeshLoadError for unsupported file extension."""
        bad_file = tmp_path / "mesh.xyz"
        bad_file.write_text("not a mesh")

        with pytest.raises(MeshLoadError) as exc_info:
            load_mesh(bad_file)

        assert "unsupported" in str(exc_info.value).lower()

    def test_load_corrupt_file_raises(self, tmp_path: Path):
        """Raise MeshLoadError for corrupt STL file."""
        corrupt = tmp_path / "corrupt.stl"
        # Write invalid binary STL header with face count but no face data
        # Binary STL: 80-byte header + 4-byte face count + face data
        # Claim 1000 faces but provide no data -> should fail
        header = b"\x00" * 80
        face_count = (1000).to_bytes(4, "little")
        corrupt.write_bytes(header + face_count)

        with pytest.raises(MeshLoadError):
            load_mesh(corrupt)

    def test_load_accepts_string_path(self, simple_cube_stl: Path):
        """Accept string path as well as Path object."""
        mesh = load_mesh(str(simple_cube_stl))

        assert mesh is not None

    def test_load_cylinder(self, simple_cylinder_stl: Path):
        """Load a cylinder mesh successfully."""
        mesh = load_mesh(simple_cylinder_stl)

        assert mesh is not None
        assert mesh.is_watertight


class TestValidateMesh:
    """Tests for mesh validation."""

    def test_validate_watertight_mesh(self, simple_cube_stl: Path):
        """Valid watertight mesh passes validation."""
        mesh = load_mesh(simple_cube_stl)

        # Should not raise
        validate_mesh(mesh)

    def test_validate_non_watertight_raises(self, non_watertight_stl: Path):
        """Non-watertight mesh raises MeshValidationError."""
        mesh = load_mesh(non_watertight_stl)

        with pytest.raises(MeshValidationError) as exc_info:
            validate_mesh(mesh, require_watertight=True)

        assert "watertight" in str(exc_info.value).lower()

    def test_validate_non_watertight_allowed(self, non_watertight_stl: Path):
        """Non-watertight mesh passes when not required."""
        mesh = load_mesh(non_watertight_stl)

        # Should not raise when watertight not required
        validate_mesh(mesh, require_watertight=False)

    def test_validate_checks_degenerate_faces(self, tmp_path: Path):
        """Mesh with degenerate (zero-area) faces fails validation."""
        import trimesh

        # Create mesh with degenerate face (collinear vertices)
        vertices = np.array([
            [0, 0, 0],
            [1, 0, 0],
            [0, 1, 0],
            [2, 0, 0],  # Collinear with 0 and 1
        ])
        faces = np.array([
            [0, 1, 2],
            [0, 1, 3],  # Degenerate: all three vertices are collinear
        ])
        mesh = trimesh.Trimesh(vertices=vertices, faces=faces)

        with pytest.raises(MeshValidationError) as exc_info:
            # Disable watertight check to ensure we hit degenerate check
            validate_mesh(mesh, require_watertight=False, check_degenerate=True)

        assert "degenerate" in str(exc_info.value).lower()

    def test_validate_returns_stats(self, simple_cube_stl: Path):
        """Validation returns mesh statistics."""
        mesh = load_mesh(simple_cube_stl)

        stats = validate_mesh(mesh, return_stats=True)

        assert "num_vertices" in stats
        assert "num_faces" in stats
        assert "is_watertight" in stats
        assert stats["num_vertices"] > 0
        assert stats["num_faces"] > 0


class TestLoadMeshWithValidation:
    """Tests for combined load and validate."""

    def test_load_and_validate(self, simple_cube_stl: Path):
        """Load mesh with validation enabled."""
        mesh = load_mesh(simple_cube_stl, validate=True)

        assert mesh is not None
        assert mesh.is_watertight

    def test_load_invalid_mesh_with_validation_raises(
        self, non_watertight_stl: Path
    ):
        """Loading non-watertight mesh with validation raises."""
        with pytest.raises(MeshValidationError):
            load_mesh(non_watertight_stl, validate=True, require_watertight=True)
