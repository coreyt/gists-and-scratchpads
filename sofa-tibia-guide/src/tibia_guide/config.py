"""Simulation configuration dataclasses."""

from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional, Union

import numpy as np
import yaml


@dataclass
class MeshConfig:
    """Configuration for a mesh asset."""

    visual_path: str
    collision_path: Optional[str] = None

    @property
    def collision_or_visual(self) -> str:
        """Return collision path if set, otherwise visual path."""
        return self.collision_path or self.visual_path


@dataclass
class PoseConfig:
    """Configuration for initial pose."""

    position: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    quaternion: List[float] = field(default_factory=lambda: [1.0, 0.0, 0.0, 0.0])

    @property
    def position_array(self) -> np.ndarray:
        """Position as numpy array."""
        return np.array(self.position)

    @property
    def quaternion_array(self) -> np.ndarray:
        """Quaternion as numpy array."""
        return np.array(self.quaternion)


@dataclass
class DisturbanceConfig:
    """Configuration for disturbance forces/torques."""

    force: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    torque: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    start_time: float = 0.0
    duration: float = float("inf")

    @property
    def force_array(self) -> np.ndarray:
        """Force as numpy array [N]."""
        return np.array(self.force)

    @property
    def torque_array(self) -> np.ndarray:
        """Torque as numpy array [N·mm]."""
        return np.array(self.torque)


@dataclass
class SolverConfig:
    """Configuration for SOFA solver."""

    dt: float = 0.001  # timestep in seconds
    gravity: List[float] = field(default_factory=lambda: [0.0, 0.0, -9810.0])  # mm/s^2
    friction_coefficient: float = 0.3
    constraint_iterations: int = 100
    constraint_tolerance: float = 1e-6


@dataclass
class OutputConfig:
    """Configuration for simulation outputs."""

    output_dir: str = "outputs"
    pose_csv: str = "pose_timeseries.csv"
    pose_json: str = "pose_final.json"
    sample_interval: int = 10  # Save pose every N steps


@dataclass
class SimulationConfig:
    """Complete simulation configuration."""

    # Mesh assets
    tibia: MeshConfig = field(default_factory=lambda: MeshConfig(visual_path=""))
    guide: MeshConfig = field(default_factory=lambda: MeshConfig(visual_path=""))

    # Initial conditions
    guide_initial_pose: PoseConfig = field(default_factory=PoseConfig)

    # Physics
    guide_mass_kg: float = 0.05  # 50 grams

    # Disturbance
    disturbance: DisturbanceConfig = field(default_factory=DisturbanceConfig)

    # Solver settings
    solver: SolverConfig = field(default_factory=SolverConfig)

    # Output settings
    output: OutputConfig = field(default_factory=OutputConfig)

    # Simulation duration
    duration_seconds: float = 2.0

    # Random seed for reproducibility
    seed: Optional[int] = None

    @classmethod
    def from_yaml(cls, path: Union[str, Path]) -> "SimulationConfig":
        """Load configuration from YAML file.

        Args:
            path: Path to YAML configuration file.

        Returns:
            SimulationConfig instance.
        """
        with open(path) as f:
            data = yaml.safe_load(f)

        return cls.from_dict(data)

    @classmethod
    def from_dict(cls, data: dict) -> "SimulationConfig":
        """Create configuration from dictionary.

        Args:
            data: Configuration dictionary.

        Returns:
            SimulationConfig instance.
        """
        config = cls()

        if "tibia" in data:
            config.tibia = MeshConfig(**data["tibia"])

        if "guide" in data:
            config.guide = MeshConfig(**data["guide"])

        if "guide_initial_pose" in data:
            config.guide_initial_pose = PoseConfig(**data["guide_initial_pose"])

        if "guide_mass_kg" in data:
            config.guide_mass_kg = data["guide_mass_kg"]

        if "disturbance" in data:
            config.disturbance = DisturbanceConfig(**data["disturbance"])

        if "solver" in data:
            config.solver = SolverConfig(**data["solver"])

        if "output" in data:
            config.output = OutputConfig(**data["output"])

        if "duration_seconds" in data:
            config.duration_seconds = data["duration_seconds"]

        if "seed" in data:
            config.seed = data["seed"]

        return config

    def to_dict(self) -> dict:
        """Convert configuration to dictionary.

        Returns:
            Configuration as dictionary.
        """
        return {
            "tibia": {
                "visual_path": self.tibia.visual_path,
                "collision_path": self.tibia.collision_path,
            },
            "guide": {
                "visual_path": self.guide.visual_path,
                "collision_path": self.guide.collision_path,
            },
            "guide_initial_pose": {
                "position": self.guide_initial_pose.position,
                "quaternion": self.guide_initial_pose.quaternion,
            },
            "guide_mass_kg": self.guide_mass_kg,
            "disturbance": {
                "force": self.disturbance.force,
                "torque": self.disturbance.torque,
                "start_time": self.disturbance.start_time,
                "duration": self.disturbance.duration,
            },
            "solver": {
                "dt": self.solver.dt,
                "gravity": self.solver.gravity,
                "friction_coefficient": self.solver.friction_coefficient,
                "constraint_iterations": self.solver.constraint_iterations,
                "constraint_tolerance": self.solver.constraint_tolerance,
            },
            "output": {
                "output_dir": self.output.output_dir,
                "pose_csv": self.output.pose_csv,
                "pose_json": self.output.pose_json,
                "sample_interval": self.output.sample_interval,
            },
            "duration_seconds": self.duration_seconds,
            "seed": self.seed,
        }

    def to_yaml(self, path: Union[str, Path]) -> None:
        """Save configuration to YAML file.

        Args:
            path: Output path for YAML file.
        """
        with open(path, "w") as f:
            yaml.dump(self.to_dict(), f, default_flow_style=False)
