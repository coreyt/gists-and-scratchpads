"""SOFA-based physics simulation for surgical cutting guide placement."""

__version__ = "0.1.0"

from tibia_guide.seating import SeatingResult, find_seating_pose

__all__ = ["find_seating_pose", "SeatingResult"]
