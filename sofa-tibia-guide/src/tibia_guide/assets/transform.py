"""Coordinate transformation utilities."""

from typing import Tuple

import numpy as np
import trimesh
from scipy.spatial.transform import Rotation


def apply_transform(
    mesh: trimesh.Trimesh,
    transform: np.ndarray,
    copy: bool = True,
) -> trimesh.Trimesh:
    """Apply a 4x4 homogeneous transformation to a mesh.

    Args:
        mesh: Input trimesh object.
        transform: 4x4 homogeneous transformation matrix.
        copy: If True, return a copy; if False, modify in place.

    Returns:
        Transformed mesh.
    """
    if copy:
        mesh = mesh.copy()

    mesh.apply_transform(transform)
    return mesh


def matrix_to_pose(transform: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """Extract position and quaternion from a 4x4 transformation matrix.

    Args:
        transform: 4x4 homogeneous transformation matrix.

    Returns:
        Tuple of (position, quaternion) where:
        - position is [x, y, z]
        - quaternion is [w, x, y, z] (scalar-first convention)
    """
    # Extract translation
    position = transform[:3, 3].copy()

    # Extract rotation matrix and convert to quaternion
    rotation_matrix = transform[:3, :3]
    quaternion = rotation_matrix_to_quaternion(rotation_matrix)

    return position, quaternion


def pose_to_matrix(
    position: np.ndarray,
    quaternion: np.ndarray,
) -> np.ndarray:
    """Create a 4x4 transformation matrix from position and quaternion.

    Args:
        position: [x, y, z] translation.
        quaternion: [w, x, y, z] rotation (scalar-first convention).

    Returns:
        4x4 homogeneous transformation matrix.
    """
    transform = np.eye(4)

    # Set rotation
    rotation_matrix = quaternion_to_rotation_matrix(quaternion)
    transform[:3, :3] = rotation_matrix

    # Set translation
    transform[:3, 3] = position

    return transform


def quaternion_to_rotation_matrix(quaternion: np.ndarray) -> np.ndarray:
    """Convert quaternion to 3x3 rotation matrix.

    Args:
        quaternion: [w, x, y, z] (scalar-first convention).

    Returns:
        3x3 rotation matrix.
    """
    # scipy uses [x, y, z, w] (scalar-last), so we need to reorder
    w, x, y, z = quaternion
    scipy_quat = np.array([x, y, z, w])

    rotation = Rotation.from_quat(scipy_quat)
    return rotation.as_matrix()


def rotation_matrix_to_quaternion(rotation_matrix: np.ndarray) -> np.ndarray:
    """Convert 3x3 rotation matrix to quaternion.

    Args:
        rotation_matrix: 3x3 rotation matrix.

    Returns:
        Quaternion [w, x, y, z] (scalar-first convention), normalized.
    """
    rotation = Rotation.from_matrix(rotation_matrix)
    scipy_quat = rotation.as_quat()  # [x, y, z, w]

    # Convert to scalar-first
    x, y, z, w = scipy_quat
    quaternion = np.array([w, x, y, z])

    # Ensure normalized
    quaternion = quaternion / np.linalg.norm(quaternion)

    return quaternion


def compose_transforms(*transforms: np.ndarray) -> np.ndarray:
    """Compose multiple 4x4 transformation matrices.

    Transforms are applied left-to-right: compose(A, B) means apply A, then B.

    Args:
        *transforms: Variable number of 4x4 transformation matrices.

    Returns:
        Composed 4x4 transformation matrix.
    """
    if not transforms:
        return np.eye(4)

    result = transforms[0].copy()
    for t in transforms[1:]:
        result = result @ t

    return result


def invert_transform(transform: np.ndarray) -> np.ndarray:
    """Compute the inverse of a 4x4 transformation matrix.

    Args:
        transform: 4x4 homogeneous transformation matrix.

    Returns:
        Inverse transformation matrix.
    """
    # For rigid transforms, we can use a more numerically stable approach
    # than general matrix inversion
    rotation = transform[:3, :3]
    translation = transform[:3, 3]

    inverse = np.eye(4)
    inverse[:3, :3] = rotation.T
    inverse[:3, 3] = -rotation.T @ translation

    return inverse
