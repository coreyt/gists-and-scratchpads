"""Unit tests for pose extraction and comparison metrics."""

import numpy as np
import pytest

from tibia_guide.metrics.pose import (
    Pose,
    compute_translation_error,
    compute_rotation_error,
    compare_poses,
    poses_within_tolerance,
)


class TestPoseDataclass:
    """Tests for Pose dataclass."""

    def test_pose_creation(self):
        """Create a pose with position and quaternion."""
        pose = Pose(
            position=np.array([1, 2, 3]),
            quaternion=np.array([1, 0, 0, 0])
        )

        np.testing.assert_array_equal(pose.position, [1, 2, 3])
        np.testing.assert_array_equal(pose.quaternion, [1, 0, 0, 0])

    def test_pose_from_matrix(self):
        """Create pose from 4x4 transformation matrix."""
        matrix = np.eye(4)
        matrix[:3, 3] = [10, 20, 30]

        pose = Pose.from_matrix(matrix)

        np.testing.assert_array_almost_equal(pose.position, [10, 20, 30])
        np.testing.assert_array_almost_equal(pose.quaternion, [1, 0, 0, 0])

    def test_pose_to_matrix(self):
        """Convert pose to 4x4 transformation matrix."""
        pose = Pose(
            position=np.array([10, 20, 30]),
            quaternion=np.array([1, 0, 0, 0])
        )

        matrix = pose.to_matrix()

        expected = np.eye(4)
        expected[:3, 3] = [10, 20, 30]
        np.testing.assert_array_almost_equal(matrix, expected)

    def test_pose_identity(self):
        """Create identity pose."""
        pose = Pose.identity()

        np.testing.assert_array_equal(pose.position, [0, 0, 0])
        np.testing.assert_array_equal(pose.quaternion, [1, 0, 0, 0])

    def test_pose_to_dict(self):
        """Convert pose to dictionary for serialization."""
        pose = Pose(
            position=np.array([1.0, 2.0, 3.0]),
            quaternion=np.array([1.0, 0.0, 0.0, 0.0])
        )

        d = pose.to_dict()

        assert "position" in d
        assert "quaternion" in d
        assert d["position"] == [1.0, 2.0, 3.0]
        assert d["quaternion"] == [1.0, 0.0, 0.0, 0.0]

    def test_pose_from_dict(self):
        """Create pose from dictionary."""
        d = {
            "position": [1.0, 2.0, 3.0],
            "quaternion": [1.0, 0.0, 0.0, 0.0]
        }

        pose = Pose.from_dict(d)

        np.testing.assert_array_equal(pose.position, [1, 2, 3])
        np.testing.assert_array_equal(pose.quaternion, [1, 0, 0, 0])


class TestTranslationError:
    """Tests for translation error computation."""

    def test_identical_positions(self):
        """Zero error for identical positions."""
        pose1 = Pose(
            position=np.array([1, 2, 3]),
            quaternion=np.array([1, 0, 0, 0])
        )
        pose2 = Pose(
            position=np.array([1, 2, 3]),
            quaternion=np.array([1, 0, 0, 0])
        )

        error = compute_translation_error(pose1, pose2)

        assert error == 0.0

    def test_unit_displacement(self):
        """Unit displacement gives error of 1."""
        pose1 = Pose(
            position=np.array([0, 0, 0]),
            quaternion=np.array([1, 0, 0, 0])
        )
        pose2 = Pose(
            position=np.array([1, 0, 0]),
            quaternion=np.array([1, 0, 0, 0])
        )

        error = compute_translation_error(pose1, pose2)

        assert np.isclose(error, 1.0)

    def test_3d_displacement(self):
        """3D displacement uses Euclidean distance."""
        pose1 = Pose(
            position=np.array([0, 0, 0]),
            quaternion=np.array([1, 0, 0, 0])
        )
        pose2 = Pose(
            position=np.array([3, 4, 0]),
            quaternion=np.array([1, 0, 0, 0])
        )

        error = compute_translation_error(pose1, pose2)

        assert np.isclose(error, 5.0)  # 3-4-5 triangle

    def test_symmetric(self):
        """Error is symmetric."""
        pose1 = Pose(
            position=np.array([0, 0, 0]),
            quaternion=np.array([1, 0, 0, 0])
        )
        pose2 = Pose(
            position=np.array([1, 2, 3]),
            quaternion=np.array([1, 0, 0, 0])
        )

        error_12 = compute_translation_error(pose1, pose2)
        error_21 = compute_translation_error(pose2, pose1)

        assert np.isclose(error_12, error_21)


class TestRotationError:
    """Tests for rotation error computation."""

    def test_identical_rotations(self):
        """Zero error for identical rotations."""
        pose1 = Pose(
            position=np.array([0, 0, 0]),
            quaternion=np.array([1, 0, 0, 0])
        )
        pose2 = Pose(
            position=np.array([0, 0, 0]),
            quaternion=np.array([1, 0, 0, 0])
        )

        error = compute_rotation_error(pose1, pose2)

        assert np.isclose(error, 0.0, atol=1e-10)

    def test_90_degree_rotation(self):
        """90-degree rotation gives 90 degrees error."""
        pose1 = Pose(
            position=np.array([0, 0, 0]),
            quaternion=np.array([1, 0, 0, 0])  # identity
        )
        # 90 degrees around Z
        angle = np.pi / 2
        pose2 = Pose(
            position=np.array([0, 0, 0]),
            quaternion=np.array([np.cos(angle/2), 0, 0, np.sin(angle/2)])
        )

        error = compute_rotation_error(pose1, pose2)

        assert np.isclose(error, 90.0, atol=1e-5)

    def test_180_degree_rotation(self):
        """180-degree rotation gives 180 degrees error."""
        pose1 = Pose(
            position=np.array([0, 0, 0]),
            quaternion=np.array([1, 0, 0, 0])
        )
        pose2 = Pose(
            position=np.array([0, 0, 0]),
            quaternion=np.array([0, 0, 0, 1])  # 180 around Z
        )

        error = compute_rotation_error(pose1, pose2)

        assert np.isclose(error, 180.0, atol=1e-5)

    def test_symmetric(self):
        """Rotation error is symmetric."""
        pose1 = Pose(
            position=np.array([0, 0, 0]),
            quaternion=np.array([1, 0, 0, 0])
        )
        angle = np.pi / 4  # 45 degrees
        pose2 = Pose(
            position=np.array([0, 0, 0]),
            quaternion=np.array([np.cos(angle/2), 0, 0, np.sin(angle/2)])
        )

        error_12 = compute_rotation_error(pose1, pose2)
        error_21 = compute_rotation_error(pose2, pose1)

        assert np.isclose(error_12, error_21)

    def test_opposite_quaternions_same_rotation(self):
        """Quaternion sign ambiguity: q and -q represent same rotation."""
        pose1 = Pose(
            position=np.array([0, 0, 0]),
            quaternion=np.array([0.5, 0.5, 0.5, 0.5])
        )
        pose2 = Pose(
            position=np.array([0, 0, 0]),
            quaternion=np.array([-0.5, -0.5, -0.5, -0.5])  # same rotation
        )

        error = compute_rotation_error(pose1, pose2)

        assert np.isclose(error, 0.0, atol=1e-5)


class TestComparePoses:
    """Tests for full pose comparison."""

    def test_identical_poses(self):
        """Identical poses have zero error."""
        pose1 = Pose(
            position=np.array([1, 2, 3]),
            quaternion=np.array([1, 0, 0, 0])
        )
        pose2 = Pose(
            position=np.array([1, 2, 3]),
            quaternion=np.array([1, 0, 0, 0])
        )

        result = compare_poses(pose1, pose2)

        assert np.isclose(result["translation_error_mm"], 0.0)
        assert np.isclose(result["rotation_error_deg"], 0.0)

    def test_compare_returns_dict(self):
        """Comparison returns dictionary with expected keys."""
        pose1 = Pose.identity()
        pose2 = Pose(
            position=np.array([1, 0, 0]),
            quaternion=np.array([1, 0, 0, 0])
        )

        result = compare_poses(pose1, pose2)

        assert "translation_error_mm" in result
        assert "rotation_error_deg" in result

    def test_compare_with_reference(self, reference_pose: dict):
        """Compare against reference pose from fixture."""
        actual = Pose(
            position=np.array([5.1, 10.05, 14.95]),  # Small offset
            quaternion=np.array([1, 0, 0, 0])
        )
        reference = Pose(
            position=reference_pose["position"],
            quaternion=reference_pose["quaternion"]
        )

        result = compare_poses(actual, reference)

        # Should be close but not zero
        assert result["translation_error_mm"] < 0.2  # ~0.12mm expected
        assert result["translation_error_mm"] > 0.0


class TestPosesWithinTolerance:
    """Tests for tolerance checking."""

    def test_within_tolerance(self):
        """Poses within tolerance return True."""
        pose1 = Pose.identity()
        pose2 = Pose(
            position=np.array([0.05, 0, 0]),  # 0.05mm offset
            quaternion=np.array([1, 0, 0, 0])
        )

        result = poses_within_tolerance(
            pose1, pose2,
            translation_tol_mm=0.1,
            rotation_tol_deg=0.1
        )

        assert result is True

    def test_exceeds_translation_tolerance(self):
        """Pose exceeding translation tolerance returns False."""
        pose1 = Pose.identity()
        pose2 = Pose(
            position=np.array([1, 0, 0]),  # 1mm offset
            quaternion=np.array([1, 0, 0, 0])
        )

        result = poses_within_tolerance(
            pose1, pose2,
            translation_tol_mm=0.1,
            rotation_tol_deg=1.0
        )

        assert result is False

    def test_exceeds_rotation_tolerance(self):
        """Pose exceeding rotation tolerance returns False."""
        pose1 = Pose.identity()
        angle = np.radians(5)  # 5 degrees
        pose2 = Pose(
            position=np.array([0, 0, 0]),
            quaternion=np.array([np.cos(angle/2), 0, 0, np.sin(angle/2)])
        )

        result = poses_within_tolerance(
            pose1, pose2,
            translation_tol_mm=1.0,
            rotation_tol_deg=1.0  # tighter than 5 deg
        )

        assert result is False

    def test_default_tolerances(self):
        """Default tolerances are reasonable for ICP comparison."""
        pose1 = Pose.identity()
        pose2 = Pose(
            position=np.array([0.05, 0.05, 0.05]),  # ~0.087mm
            quaternion=np.array([1, 0, 0, 0])
        )

        # Default: 0.1mm translation, 0.1deg rotation
        result = poses_within_tolerance(pose1, pose2)

        assert result is True
