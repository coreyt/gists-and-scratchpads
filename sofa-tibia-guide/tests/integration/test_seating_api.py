"""Integration tests for the high-level seating API.

These tests require SOFA to be installed.
"""

from pathlib import Path

import numpy as np
import pytest
import trimesh

from tests.conftest import sofa_available


pytestmark = pytest.mark.sofa


@pytest.fixture
def skip_if_no_sofa():
    """Skip test if SOFA is not available."""
    if not sofa_available():
        pytest.skip("SOFA not installed")


class TestFindSeatingPose:
    """Tests for find_seating_pose API."""

    def test_basic_usage_with_paths(
        self, skip_if_no_sofa, simple_cube_stl: Path, simple_cylinder_stl: Path
    ):
        """Basic usage with file paths."""
        from tibia_guide import find_seating_pose

        result = find_seating_pose(
            tibia_mesh=simple_cube_stl,
            guide_mesh=simple_cylinder_stl,
            max_simulation_time=0.5,  # Short for testing
        )

        assert result.pose is not None
        assert result.steps > 0
        assert isinstance(result.converged, bool)
        assert isinstance(result.final_gap_mm, float)

    def test_basic_usage_with_trimesh(
        self, skip_if_no_sofa, simple_cube_stl: Path, simple_cylinder_stl: Path
    ):
        """Basic usage with trimesh objects."""
        from tibia_guide import find_seating_pose

        tibia = trimesh.load(str(simple_cube_stl))
        guide = trimesh.load(str(simple_cylinder_stl))

        result = find_seating_pose(
            tibia_mesh=tibia,
            guide_mesh=guide,
            max_simulation_time=0.5,
        )

        assert result.pose is not None
        assert result.steps > 0

    def test_returns_seating_result(
        self, skip_if_no_sofa, simple_cube_stl: Path, simple_cylinder_stl: Path
    ):
        """Returns proper SeatingResult dataclass."""
        from tibia_guide import SeatingResult, find_seating_pose
        from tibia_guide.metrics.pose import Pose

        result = find_seating_pose(
            tibia_mesh=simple_cube_stl,
            guide_mesh=simple_cylinder_stl,
            max_simulation_time=0.5,
        )

        assert isinstance(result, SeatingResult)
        assert isinstance(result.pose, Pose)
        assert hasattr(result.pose, "position")
        assert hasattr(result.pose, "quaternion")

    def test_pose_has_valid_quaternion(
        self, skip_if_no_sofa, simple_cube_stl: Path, simple_cylinder_stl: Path
    ):
        """Returned pose has unit quaternion."""
        from tibia_guide import find_seating_pose

        result = find_seating_pose(
            tibia_mesh=simple_cube_stl,
            guide_mesh=simple_cylinder_stl,
            max_simulation_time=0.5,
        )

        quat_norm = np.linalg.norm(result.pose.quaternion)
        assert abs(quat_norm - 1.0) < 0.01, f"Quaternion not unit: {quat_norm}"

    def test_convergence_detection(
        self, skip_if_no_sofa, simple_cube_stl: Path, simple_cylinder_stl: Path
    ):
        """Convergence is detected when guide settles."""
        from tibia_guide import find_seating_pose

        result = find_seating_pose(
            tibia_mesh=simple_cube_stl,
            guide_mesh=simple_cylinder_stl,
            max_simulation_time=2.0,  # Longer to allow convergence
            convergence_threshold=0.1,  # Looser threshold
            convergence_check_interval=50,
        )

        # With simple test meshes, should converge quickly
        # (they don't overlap, so guide just falls and stops)
        assert result.steps < 2000  # Didn't run full 2 seconds

    def test_missing_file_raises_error(self, skip_if_no_sofa, simple_cube_stl: Path):
        """FileNotFoundError when mesh file doesn't exist."""
        from tibia_guide import find_seating_pose

        with pytest.raises(FileNotFoundError, match="guide mesh not found"):
            find_seating_pose(
                tibia_mesh=simple_cube_stl,
                guide_mesh="/nonexistent/guide.stl",
            )

    def test_custom_parameters(
        self, skip_if_no_sofa, simple_cube_stl: Path, simple_cylinder_stl: Path
    ):
        """Custom simulation parameters are respected."""
        from tibia_guide import find_seating_pose

        result = find_seating_pose(
            tibia_mesh=simple_cube_stl,
            guide_mesh=simple_cylinder_stl,
            max_simulation_time=0.2,
            dt=0.002,  # Larger timestep
            guide_mass_kg=0.05,
        )

        # With dt=0.002 and max_time=0.2, max steps would be 100
        assert result.steps <= 100


class TestSeatingResultDataclass:
    """Tests for SeatingResult dataclass."""

    def test_dataclass_fields(self):
        """SeatingResult has expected fields."""
        from tibia_guide import SeatingResult
        from tibia_guide.metrics.pose import Pose

        result = SeatingResult(
            pose=Pose(position=np.array([1, 2, 3]), quaternion=np.array([1, 0, 0, 0])),
            converged=True,
            steps=500,
            final_gap_mm=0.5,
        )

        assert result.pose.position[0] == 1
        assert result.converged is True
        assert result.steps == 500
        assert result.final_gap_mm == 0.5
