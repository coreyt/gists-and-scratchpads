"""MVM (Minimum Viable Model) seating scene.

Rigid tibia (fixed) + rigid guide (dynamic) with frictional contact.

IMPORTANT: Mesh coordinates must be normalized before calling create_scene().
Large coordinate values (e.g., Z around -125mm) cause numerical issues in SOFA
collision detection. Use normalize_meshes() or manually:
  1. Translate both meshes so tibia centroid is at origin
  2. Translate both meshes so tibia top surface is at Z=0
  3. Store the inverse transform to convert results back to original frame

IMPORTANT: RigidMapping coordinate frame
When using MeshSTLLoader with meshes already positioned at world coordinates:
  - Set guide_initial_position to [0, 0, 0]
  - The mesh vertices define the starting position
  - The rigid body position becomes the DELTA from initial position
  - To get world position: mesh_centroid + rigid_body_position
"""

from typing import List, Optional

import numpy as np

from tibia_guide.metrics.pose import Pose


def create_scene(
    tibia_mesh_path: str,
    guide_mesh_path: str,
    guide_initial_position: Optional[List[float]] = None,
    guide_initial_quaternion: Optional[List[float]] = None,
    guide_mass_kg: float = 0.05,
    friction_coefficient: float = 0.3,
    gravity: Optional[List[float]] = None,
    dt: float = 0.001,
    gui: bool = False,
):
    """Create SOFA scene with tibia and guide.

    Args:
        tibia_mesh_path: Path to tibia collision mesh (STL).
        guide_mesh_path: Path to guide collision mesh (STL).
        guide_initial_position: Initial guide position [x, y, z] in mm.
        guide_initial_quaternion: Initial guide orientation [w, x, y, z].
        guide_mass_kg: Guide mass in kilograms.
        friction_coefficient: Friction coefficient for contact.
        gravity: Gravity vector [gx, gy, gz] in mm/s^2.
        dt: Simulation timestep in seconds.
        gui: If True, add visual models for GUI rendering.

    Returns:
        SOFA root node.
    """
    import Sofa
    import Sofa.Core

    # Defaults
    if guide_initial_position is None:
        guide_initial_position = [0.0, 0.0, 0.0]
    if guide_initial_quaternion is None:
        guide_initial_quaternion = [1.0, 0.0, 0.0, 0.0]
    if gravity is None:
        gravity = [0.0, 0.0, -9810.0]  # mm/s^2

    # Convert quaternion from [w, x, y, z] to SOFA's [x, y, z, w] format
    w, qx, qy, qz = guide_initial_quaternion
    sofa_quaternion = [qx, qy, qz, w]

    # SOFA rigid position format: [x, y, z, qx, qy, qz, qw]
    guide_rigid_position = guide_initial_position + sofa_quaternion

    # Create root node
    root = Sofa.Core.Node("root")
    root.gravity = gravity
    root.dt = dt

    # Required plugins
    plugins = [
        "Sofa.Component.AnimationLoop",
        "Sofa.Component.Collision.Detection.Algorithm",
        "Sofa.Component.Collision.Detection.Intersection",
        "Sofa.Component.Collision.Geometry",
        "Sofa.Component.Collision.Response.Contact",
        "Sofa.Component.Constraint.Lagrangian.Correction",
        "Sofa.Component.Constraint.Lagrangian.Solver",
        "Sofa.Component.IO.Mesh",
        "Sofa.Component.LinearSolver.Iterative",
        "Sofa.Component.Mapping.NonLinear",
        "Sofa.Component.Mass",
        "Sofa.Component.ODESolver.Backward",
        "Sofa.Component.StateContainer",
        "Sofa.Component.Topology.Container.Constant",
        "Sofa.Component.Constraint.Projective",
    ]

    # Add visual plugins for GUI mode
    if gui:
        plugins.extend([
            "Sofa.GL.Component.Rendering3D",
            "Sofa.GL.Component.Shader",
            "Sofa.Component.Mapping.Linear",
        ])

    root.addObject("RequiredPlugin", pluginName=plugins)

    # Collision pipeline with constraint-based contact
    root.addObject("FreeMotionAnimationLoop")
    root.addObject("CollisionPipeline")
    root.addObject("BruteForceBroadPhase")
    root.addObject("BVHNarrowPhase")
    root.addObject(
        "LocalMinDistance",
        alarmDistance=5.0,
        contactDistance=2.0,
        angleCone=0.1,  # Filter edge contacts to prevent ghost collisions
    )
    root.addObject(
        "CollisionResponse",
        response="FrictionContactConstraint",
    )
    # Use GenericConstraintSolver instead of LCPConstraintSolver
    # LCPConstraintSolver has known numerical issues and is deprecated
    root.addObject("GenericConstraintSolver", maxIterations=1000, tolerance=0.001)

    # Tibia node (fixed, static collision surface)
    # Note: For fixed objects, we don't need Rigid3d or RigidMapping
    # Just load the mesh directly as Vec3d with simulated=False
    tibia = root.addChild("Tibia")
    tibia.addObject(
        "MeshSTLLoader",
        name="loader",
        filename=tibia_mesh_path,
    )
    tibia.addObject(
        "MeshTopology",
        src="@loader",
    )
    tibia.addObject(
        "MechanicalObject",
        name="mstate",
        src="@loader",
    )
    tibia.addObject(
        "TriangleCollisionModel",
        simulated=False,
        moving=False,
    )
    tibia.addObject(
        "LineCollisionModel",
        simulated=False,
        moving=False,
    )
    tibia.addObject(
        "PointCollisionModel",
        simulated=False,
        moving=False,
    )

    # Tibia visual (for GUI only)
    if gui:
        tibia_visual = tibia.addChild("Visual")
        tibia_visual.addObject(
            "OglModel",
            name="VisualModel",
            src="@../loader",
            color="0.9 0.9 0.8 0.7",  # ivory, semi-transparent
        )
        tibia_visual.addObject(
            "IdentityMapping",
            input="@../mstate",
            output="@VisualModel",
        )

    # Guide node (dynamic)
    guide = root.addChild("Guide")
    guide.addObject(
        "EulerImplicitSolver",
        rayleighStiffness=0.05,
        rayleighMass=1.0,
    )
    guide.addObject(
        "CGLinearSolver",
        iterations=25,
        tolerance=1e-10,
        threshold=1e-10,
    )
    guide.addObject(
        "MechanicalObject",
        name="MechanicalObject",
        template="Rigid3d",
        position=guide_rigid_position,
    )
    guide.addObject("UniformMass", totalMass=guide_mass_kg)
    guide.addObject("UncoupledConstraintCorrection")

    # Guide collision
    guide_collision = guide.addChild("Collision")
    guide_collision.addObject(
        "MeshSTLLoader",
        name="loader",
        filename=guide_mesh_path,
    )
    guide_collision.addObject(
        "MeshTopology",
        src="@loader",
        name="GuideCollisionModel",
    )
    guide_collision.addObject(
        "MechanicalObject",
        name="CollisionMO",
        src="@loader",
    )
    guide_collision.addObject(
        "TriangleCollisionModel",
        name="guide_triangle",
        contactStiffness=5,
    )
    guide_collision.addObject(
        "LineCollisionModel",
        name="guide_line",
        contactStiffness=5,
    )
    guide_collision.addObject(
        "PointCollisionModel",
        name="guide_point",
        contactStiffness=5,
    )
    guide_collision.addObject(
        "RigidMapping",
        input="@../MechanicalObject",
        output="@CollisionMO",
    )

    # Guide visual (for GUI only)
    if gui:
        guide_visual = guide.addChild("Visual")
        guide_visual.addObject(
            "MeshSTLLoader",
            name="loader",
            filename=guide_mesh_path,
        )
        guide_visual.addObject(
            "OglModel",
            name="VisualModel",
            src="@loader",
            color="0.27 0.51 0.71 1.0",  # steel blue
        )
        guide_visual.addObject(
            "RigidMapping",
            input="@../MechanicalObject",
            output="@VisualModel",
        )

    return root


def extract_guide_pose(root) -> Pose:
    """Extract current guide pose from SOFA scene.

    Args:
        root: SOFA root node.

    Returns:
        Pose object with guide position and orientation.
    """
    guide = root.getChild("Guide")
    mech = guide.getObject("MechanicalObject")

    # SOFA rigid format: [x, y, z, qx, qy, qz, qw]
    position_data = mech.position.value[0]

    x, y, z = position_data[:3]
    qx, qy, qz, qw = position_data[3:7]

    # Convert to our [w, x, y, z] convention
    position = np.array([x, y, z])
    quaternion = np.array([qw, qx, qy, qz])

    return Pose(position=position, quaternion=quaternion)


def add_constant_force(
    root,
    force: List[float],
    torque: Optional[List[float]] = None,
):
    """Add constant force/torque to the guide.

    Args:
        root: SOFA root node.
        force: Force vector [fx, fy, fz] in N.
        torque: Torque vector [tx, ty, tz] in N·mm. Defaults to zero.
    """
    if torque is None:
        torque = [0.0, 0.0, 0.0]

    guide = root.getChild("Guide")

    # For rigid bodies, we need to apply force at center of mass
    # ConstantForceField for rigid takes [fx, fy, fz, tx, ty, tz]
    forces = [force + torque]

    guide.addObject(
        "ConstantForceField",
        name="Disturbance",
        forces=forces,
    )


def normalize_meshes(tibia_mesh, guide_mesh):
    """Normalize mesh coordinates to prevent numerical issues.

    SOFA collision detection has numerical issues with large coordinate values.
    This function translates both meshes so the tibia is centered at origin
    with its top surface at Z=0.

    Args:
        tibia_mesh: trimesh.Trimesh for tibia (modified in place).
        guide_mesh: trimesh.Trimesh for guide (modified in place).

    Returns:
        dict with 'offset' (XYZ translation) and 'z_offset' (Z translation)
        to convert results back to original coordinate frame.
    """
    # Move both meshes so tibia centroid is at origin
    offset = tibia_mesh.centroid.copy()
    tibia_mesh.apply_translation(-offset)
    guide_mesh.apply_translation(-offset)

    # Move so tibia top surface is at Z=0
    z_offset = tibia_mesh.bounds[1][2]
    tibia_mesh.apply_translation([0, 0, -z_offset])
    guide_mesh.apply_translation([0, 0, -z_offset])

    return {
        "offset": offset,
        "z_offset": z_offset,
        "total_offset": np.array([offset[0], offset[1], offset[2] + z_offset]),
    }


def denormalize_pose(pose: Pose, normalization_info: dict) -> Pose:
    """Convert pose from normalized coordinates back to original frame.

    Args:
        pose: Pose in normalized coordinates.
        normalization_info: Dict from normalize_meshes().

    Returns:
        Pose in original coordinate frame.
    """
    total_offset = normalization_info["total_offset"]
    denorm_position = pose.position + total_offset
    return Pose(position=denorm_position, quaternion=pose.quaternion)
