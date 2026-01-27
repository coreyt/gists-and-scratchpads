"""Contact metrics extraction (placeholder for future implementation)."""

from typing import Dict, List

import numpy as np


class ContactMetrics:
    """Container for contact-related metrics.

    Placeholder for future implementation. Will extract:
    - Number of contact points
    - Contact forces
    - Contact locations
    - Penetration depths
    """

    def __init__(self):
        """Initialize empty contact metrics."""
        self.num_contacts: int = 0
        self.contact_points: List[np.ndarray] = []
        self.contact_normals: List[np.ndarray] = []
        self.contact_forces: List[float] = []

    def to_dict(self) -> Dict:
        """Convert to dictionary for serialization."""
        return {
            "num_contacts": self.num_contacts,
            "contact_points": [p.tolist() for p in self.contact_points],
            "contact_normals": [n.tolist() for n in self.contact_normals],
            "contact_forces": self.contact_forces,
        }


def extract_contact_metrics(root) -> ContactMetrics:
    """Extract contact metrics from SOFA scene.

    Placeholder - actual implementation requires SOFA contact response
    introspection.

    Args:
        root: SOFA root node.

    Returns:
        ContactMetrics object.
    """
    # TODO: Implement contact extraction from SOFA collision response
    return ContactMetrics()
