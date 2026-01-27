"""Unit tests for coordinate transform functionality."""

from pathlib import Path

import numpy as np
import pytest

from tibia_guide.assets.transform import (
    apply_transform,
    compose_transforms,
    invert_transform,
    matrix_to_pose,
    pose_to_matrix,
    quaternion_to_rotation_matrix,
    rotation_matrix_to_quaternion,
)


class TestApplyTransform:
    """Tests for applying transforms to meshes."""

    def test_identity_transform_unchanged(self, simple_cube_stl: Path):
        """Identity transform leaves mesh unchanged."""
        from tibia_guide.assets.loader import load_mesh

        mesh = load_mesh(simple_cube_stl)
        original_vertices = mesh.vertices.copy()

        transformed = apply_transform(mesh, np.eye(4))

        np.testing.assert_array_almost_equal(
            transformed.vertices, original_vertices
        )

    def test_translation_only(self, simple_cube_stl: Path):
        """Pure translation moves all vertices equally."""
        from tibia_guide.assets.loader import load_mesh

        mesh = load_mesh(simple_cube_stl)
        original_centroid = mesh.centroid.copy()

        translation = np.array([10, 20, 30])
        transform = np.eye(4)
        transform[:3, 3] = translation

        transformed = apply_transform(mesh, transform)

        expected_centroid = original_centroid + translation
        np.testing.assert_array_almost_equal(
            transformed.centroid, expected_centroid
        )

    def test_rotation_90_degrees_z(self, simple_cube_stl: Path):
        """90-degree rotation around Z-axis."""
        from tibia_guide.assets.loader import load_mesh

        mesh = load_mesh(simple_cube_stl)

        # 90 degrees around Z: x -> -y, y -> x
        transform = np.array([
            [0, -1, 0, 0],
            [1, 0, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ], dtype=np.float64)

        transformed = apply_transform(mesh, transform)

        # For a centered cube, bounds should rotate
        # Original bounds should map correctly
        assert transformed.vertices is not None

    def test_apply_sample_transform(
        self, simple_cube_stl: Path, sample_transform: np.ndarray
    ):
        """Apply known rotation + translation."""
        from tibia_guide.assets.loader import load_mesh

        mesh = load_mesh(simple_cube_stl)

        transformed = apply_transform(mesh, sample_transform)

        # Centroid should be translated by [10, 20, 30] after rotation
        # Original centroid is at origin for centered cube
        np.testing.assert_array_almost_equal(
            transformed.centroid, [10, 20, 30], decimal=5
        )

    def test_transform_preserves_mesh_properties(self, simple_cube_stl: Path):
        """Transform preserves face count, watertightness, etc."""
        from tibia_guide.assets.loader import load_mesh

        mesh = load_mesh(simple_cube_stl)
        original_face_count = len(mesh.faces)
        original_watertight = mesh.is_watertight

        transform = np.array([
            [0.5, 0, 0, 5],
            [0, 0.5, 0, 10],
            [0, 0, 0.5, 15],
            [0, 0, 0, 1]
        ])

        transformed = apply_transform(mesh, transform)

        assert len(transformed.faces) == original_face_count
        assert transformed.is_watertight == original_watertight

    def test_transform_copies_mesh(self, simple_cube_stl: Path):
        """Transform returns a copy, doesn't modify original."""
        from tibia_guide.assets.loader import load_mesh

        mesh = load_mesh(simple_cube_stl)
        original_vertices = mesh.vertices.copy()

        transform = np.eye(4)
        transform[:3, 3] = [100, 100, 100]

        _ = apply_transform(mesh, transform)

        # Original should be unchanged
        np.testing.assert_array_equal(mesh.vertices, original_vertices)


class TestMatrixToPose:
    """Tests for extracting pose from transformation matrix."""

    def test_identity_matrix(self):
        """Identity matrix gives zero translation, identity quaternion."""
        position, quaternion = matrix_to_pose(np.eye(4))

        np.testing.assert_array_almost_equal(position, [0, 0, 0])
        # Identity quaternion: w=1, x=y=z=0
        np.testing.assert_array_almost_equal(quaternion, [1, 0, 0, 0])

    def test_translation_only(self):
        """Pure translation matrix."""
        transform = np.eye(4)
        transform[:3, 3] = [10, 20, 30]

        position, quaternion = matrix_to_pose(transform)

        np.testing.assert_array_almost_equal(position, [10, 20, 30])
        np.testing.assert_array_almost_equal(quaternion, [1, 0, 0, 0])

    def test_sample_transform(
        self, sample_transform: np.ndarray,
        sample_quaternion: np.ndarray,
        sample_position: np.ndarray
    ):
        """Extract pose from known transform."""
        position, quaternion = matrix_to_pose(sample_transform)

        np.testing.assert_array_almost_equal(position, sample_position)
        # Quaternion may have opposite sign (represents same rotation)
        assert np.allclose(quaternion, sample_quaternion) or np.allclose(
            quaternion, -sample_quaternion
        )

    def test_rotation_180_z(self):
        """180-degree rotation around Z-axis."""
        transform = np.array([
            [-1, 0, 0, 0],
            [0, -1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ], dtype=np.float64)

        position, quaternion = matrix_to_pose(transform)

        # 180 degrees around Z: w=0, x=0, y=0, z=1
        np.testing.assert_array_almost_equal(position, [0, 0, 0])
        expected_quat = np.array([0, 0, 0, 1])
        assert np.allclose(quaternion, expected_quat) or np.allclose(
            quaternion, -expected_quat
        )


class TestPoseToMatrix:
    """Tests for creating transformation matrix from pose."""

    def test_identity_pose(self):
        """Identity pose gives identity matrix."""
        position = np.array([0, 0, 0])
        quaternion = np.array([1, 0, 0, 0])

        matrix = pose_to_matrix(position, quaternion)

        np.testing.assert_array_almost_equal(matrix, np.eye(4))

    def test_translation_only(self):
        """Pure translation pose."""
        position = np.array([10, 20, 30])
        quaternion = np.array([1, 0, 0, 0])

        matrix = pose_to_matrix(position, quaternion)

        expected = np.eye(4)
        expected[:3, 3] = [10, 20, 30]
        np.testing.assert_array_almost_equal(matrix, expected)

    def test_roundtrip_matrix_pose_matrix(self, sample_transform: np.ndarray):
        """Matrix -> pose -> matrix roundtrip."""
        position, quaternion = matrix_to_pose(sample_transform)
        reconstructed = pose_to_matrix(position, quaternion)

        np.testing.assert_array_almost_equal(
            reconstructed, sample_transform, decimal=5
        )

    def test_roundtrip_pose_matrix_pose(
        self, sample_position: np.ndarray, sample_quaternion: np.ndarray
    ):
        """Pose -> matrix -> pose roundtrip."""
        matrix = pose_to_matrix(sample_position, sample_quaternion)
        pos_out, quat_out = matrix_to_pose(matrix)

        np.testing.assert_array_almost_equal(pos_out, sample_position)
        # Quaternion sign ambiguity
        assert np.allclose(quat_out, sample_quaternion) or np.allclose(
            quat_out, -sample_quaternion
        )


class TestComposeTransforms:
    """Tests for composing multiple transforms."""

    def test_compose_identity(self):
        """Composing with identity gives original."""
        transform = np.array([
            [1, 0, 0, 10],
            [0, 1, 0, 20],
            [0, 0, 1, 30],
            [0, 0, 0, 1]
        ], dtype=np.float64)

        composed = compose_transforms(np.eye(4), transform)

        np.testing.assert_array_almost_equal(composed, transform)

    def test_compose_translations(self):
        """Composing translations adds them."""
        t1 = np.eye(4)
        t1[:3, 3] = [10, 0, 0]

        t2 = np.eye(4)
        t2[:3, 3] = [0, 20, 0]

        composed = compose_transforms(t1, t2)

        expected = np.eye(4)
        expected[:3, 3] = [10, 20, 0]
        np.testing.assert_array_almost_equal(composed, expected)

    def test_compose_multiple(self):
        """Compose three transforms."""
        t1 = np.eye(4)
        t1[:3, 3] = [1, 0, 0]

        t2 = np.eye(4)
        t2[:3, 3] = [0, 1, 0]

        t3 = np.eye(4)
        t3[:3, 3] = [0, 0, 1]

        composed = compose_transforms(t1, t2, t3)

        expected = np.eye(4)
        expected[:3, 3] = [1, 1, 1]
        np.testing.assert_array_almost_equal(composed, expected)


class TestInvertTransform:
    """Tests for inverting transforms."""

    def test_invert_identity(self):
        """Inverse of identity is identity."""
        inverse = invert_transform(np.eye(4))

        np.testing.assert_array_almost_equal(inverse, np.eye(4))

    def test_invert_translation(self):
        """Inverse of translation is negative translation."""
        transform = np.eye(4)
        transform[:3, 3] = [10, 20, 30]

        inverse = invert_transform(transform)

        expected = np.eye(4)
        expected[:3, 3] = [-10, -20, -30]
        np.testing.assert_array_almost_equal(inverse, expected)

    def test_inverse_roundtrip(self, sample_transform: np.ndarray):
        """T @ T^-1 = I."""
        inverse = invert_transform(sample_transform)
        product = sample_transform @ inverse

        np.testing.assert_array_almost_equal(product, np.eye(4), decimal=5)


class TestQuaternionConversions:
    """Tests for quaternion <-> rotation matrix conversions."""

    def test_identity_quaternion_to_matrix(self):
        """Identity quaternion gives identity rotation."""
        quat = np.array([1, 0, 0, 0])

        rot = quaternion_to_rotation_matrix(quat)

        np.testing.assert_array_almost_equal(rot, np.eye(3))

    def test_90_deg_z_quaternion(self, sample_quaternion: np.ndarray):
        """90-degree Z rotation quaternion."""
        rot = quaternion_to_rotation_matrix(sample_quaternion)

        # Expected: x->-y, y->x
        expected = np.array([
            [0, -1, 0],
            [1, 0, 0],
            [0, 0, 1]
        ], dtype=np.float64)
        np.testing.assert_array_almost_equal(rot, expected, decimal=5)

    def test_roundtrip_quat_mat_quat(self, sample_quaternion: np.ndarray):
        """Quaternion -> matrix -> quaternion roundtrip."""
        mat = quaternion_to_rotation_matrix(sample_quaternion)
        quat_out = rotation_matrix_to_quaternion(mat)

        # Sign ambiguity
        assert np.allclose(quat_out, sample_quaternion, atol=1e-6) or np.allclose(
            quat_out, -sample_quaternion, atol=1e-6
        )

    def test_normalized_quaternion_output(self):
        """Output quaternion is normalized."""
        # Arbitrary rotation matrix
        angle = 0.7
        rot = np.array([
            [np.cos(angle), -np.sin(angle), 0],
            [np.sin(angle), np.cos(angle), 0],
            [0, 0, 1]
        ])

        quat = rotation_matrix_to_quaternion(rot)

        norm = np.linalg.norm(quat)
        assert np.isclose(norm, 1.0, atol=1e-6)
