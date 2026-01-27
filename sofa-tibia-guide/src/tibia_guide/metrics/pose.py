"""Pose representation and comparison metrics."""

from dataclasses import dataclass
from typing import Dict, List, Union

import numpy as np
from scipy.spatial.transform import Rotation


@dataclass
class Pose:
    """Represents a 6-DOF pose (position + orientation).

    Attributes:
        position: [x, y, z] translation in millimeters.
        quaternion: [w, x, y, z] rotation (scalar-first convention).
    """

    position: np.ndarray
    quaternion: np.ndarray

    def __post_init__(self):
        """Ensure arrays are numpy arrays."""
        self.position = np.asarray(self.position, dtype=np.float64)
        self.quaternion = np.asarray(self.quaternion, dtype=np.float64)

        # Normalize quaternion
        norm = np.linalg.norm(self.quaternion)
        if norm > 0:
            self.quaternion = self.quaternion / norm

    @classmethod
    def identity(cls) -> "Pose":
        """Create an identity pose (origin, no rotation)."""
        return cls(
            position=np.array([0.0, 0.0, 0.0]),
            quaternion=np.array([1.0, 0.0, 0.0, 0.0]),
        )

    @classmethod
    def from_matrix(cls, matrix: np.ndarray) -> "Pose":
        """Create pose from 4x4 transformation matrix.

        Args:
            matrix: 4x4 homogeneous transformation matrix.

        Returns:
            Pose instance.
        """
        from tibia_guide.assets.transform import matrix_to_pose

        position, quaternion = matrix_to_pose(matrix)
        return cls(position=position, quaternion=quaternion)

    def to_matrix(self) -> np.ndarray:
        """Convert pose to 4x4 transformation matrix.

        Returns:
            4x4 homogeneous transformation matrix.
        """
        from tibia_guide.assets.transform import pose_to_matrix

        return pose_to_matrix(self.position, self.quaternion)

    def to_dict(self) -> Dict[str, List[float]]:
        """Convert pose to dictionary for serialization.

        Returns:
            Dictionary with 'position' and 'quaternion' keys.
        """
        return {
            "position": self.position.tolist(),
            "quaternion": self.quaternion.tolist(),
        }

    @classmethod
    def from_dict(cls, data: Dict[str, List[float]]) -> "Pose":
        """Create pose from dictionary.

        Args:
            data: Dictionary with 'position' and 'quaternion' keys.

        Returns:
            Pose instance.
        """
        return cls(
            position=np.array(data["position"]),
            quaternion=np.array(data["quaternion"]),
        )


def compute_translation_error(pose1: Pose, pose2: Pose) -> float:
    """Compute translation error (Euclidean distance) between two poses.

    Args:
        pose1: First pose.
        pose2: Second pose.

    Returns:
        Translation error in millimeters.
    """
    return float(np.linalg.norm(pose1.position - pose2.position))


def compute_rotation_error(pose1: Pose, pose2: Pose) -> float:
    """Compute rotation error (geodesic distance) between two poses.

    The geodesic distance on SO(3) is the angle of the relative rotation.

    Args:
        pose1: First pose.
        pose2: Second pose.

    Returns:
        Rotation error in degrees.
    """
    # Convert to scipy Rotation objects (scalar-last convention)
    w1, x1, y1, z1 = pose1.quaternion
    w2, x2, y2, z2 = pose2.quaternion

    r1 = Rotation.from_quat([x1, y1, z1, w1])
    r2 = Rotation.from_quat([x2, y2, z2, w2])

    # Compute relative rotation: r_rel = r1.inv() * r2
    r_rel = r1.inv() * r2

    # Get the angle of the relative rotation
    # magnitude() returns the rotation angle in radians
    angle_rad = r_rel.magnitude()

    # Convert to degrees
    angle_deg = np.degrees(angle_rad)

    return float(angle_deg)


def compare_poses(pose1: Pose, pose2: Pose) -> Dict[str, float]:
    """Compare two poses and return error metrics.

    Args:
        pose1: First pose (e.g., measured).
        pose2: Second pose (e.g., reference).

    Returns:
        Dictionary with:
        - translation_error_mm: Euclidean distance in mm
        - rotation_error_deg: Geodesic distance in degrees
    """
    return {
        "translation_error_mm": compute_translation_error(pose1, pose2),
        "rotation_error_deg": compute_rotation_error(pose1, pose2),
    }


def poses_within_tolerance(
    pose1: Pose,
    pose2: Pose,
    translation_tol_mm: float = 0.1,
    rotation_tol_deg: float = 0.1,
) -> bool:
    """Check if two poses are within specified tolerances.

    Args:
        pose1: First pose.
        pose2: Second pose.
        translation_tol_mm: Maximum allowed translation error in mm.
        rotation_tol_deg: Maximum allowed rotation error in degrees.

    Returns:
        True if both errors are within tolerance.
    """
    comparison = compare_poses(pose1, pose2)

    return (
        comparison["translation_error_mm"] <= translation_tol_mm
        and comparison["rotation_error_deg"] <= rotation_tol_deg
    )
