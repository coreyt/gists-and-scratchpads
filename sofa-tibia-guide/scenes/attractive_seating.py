"""Attractive Contact Physics seating scene using SOFA.

This implements the "attractive contact" approach where:
1. Virtual springs pull guide mating surface toward bone surface
2. Collision prevents interpenetration
3. Guide settles at equilibrium (best-fit position)

The key insight: instead of just gravity, we add attractive forces
between the guide's inner (mating) surface and the bone's outer surface.
This pulls the guide INTO the bone cavity until collision stops it.

Multi-Phase Collision (2026-01-28):
- Phase 1: Springs only (no collision) - fast convergence to approximate position
- Phase 2: Coarse collision (500 faces, Triangle-only) - prevent gross interpenetration
- Phase 3: Fine collision (2000 faces, Triangle+Line+Point) - accurate final seating
"""

from pathlib import Path
from typing import List, Optional, Tuple, Dict, Any

import numpy as np
import trimesh
from scipy.spatial import cKDTree

from tibia_guide.metrics.pose import Pose


# =============================================================================
# Settings Management
# =============================================================================

# Default settings (used if no settings.yaml is found)
DEFAULT_SETTINGS = {
    'mating_points': {
        'distance_threshold_mm': 1.0,
        'n_points': 150,
        'seed': 42,
    },
    'springs': {
        'stiffness': 50.0,
        'damping': 10.0,
    },
    'rotation': {
        'stiffness': 0.0,
        'max_angular_velocity': 5.0,
    },
    'collision': {
        'enabled': True,
        'multi_phase': {
            'enabled': True,
            'coarse_faces': 500,
            'fine_faces': 2000,
        },
        'single_phase': {
            'faces': 5000,
        },
        'distances': {
            'alarm_distance': 5.0,
            'contact_distance': 2.0,
        },
    },
    'convergence': {
        'thresholds': {
            'phase1': 0.2,
            'phase2': 0.05,
            'phase3': 0.01,
        },
        'settling_steps': 50,
    },
    'simulation': {
        'dt': 0.001,
        'max_time': 10.0,
        'gravity': [0.0, 0.0, -9810.0],
        'guide_mass_kg': 0.05,
    },
    'initial_offset': {
        'offset_mm': [0.0, 5.0, 0.0],
    },
}


def load_settings(settings_path: Optional[str] = None) -> Dict[str, Any]:
    """Load settings from YAML file, falling back to defaults.

    Args:
        settings_path: Path to settings.yaml. If None, searches for settings.yaml
                      in the current directory and parent directories.

    Returns:
        Dictionary of settings with defaults applied for missing values.
    """
    import copy

    settings = copy.deepcopy(DEFAULT_SETTINGS)

    # Find settings file
    if settings_path is None:
        # Search in current dir and parents
        search_paths = [
            Path.cwd() / 'settings.yaml',
            Path(__file__).parent.parent / 'settings.yaml',
        ]
        for path in search_paths:
            if path.exists():
                settings_path = str(path)
                break

    if settings_path and Path(settings_path).exists():
        try:
            import yaml
            with open(settings_path, 'r') as f:
                user_settings = yaml.safe_load(f)

            if user_settings:
                # Deep merge user settings into defaults
                _deep_merge(settings, user_settings)
                print(f"[SETTINGS] Loaded from {settings_path}")
        except ImportError:
            print("[SETTINGS] PyYAML not installed, using defaults")
        except Exception as e:
            print(f"[SETTINGS] Error loading {settings_path}: {e}, using defaults")
    else:
        print("[SETTINGS] No settings.yaml found, using defaults")

    return settings


def _deep_merge(base: dict, override: dict) -> None:
    """Recursively merge override into base dictionary."""
    for key, value in override.items():
        if key in base and isinstance(base[key], dict) and isinstance(value, dict):
            _deep_merge(base[key], value)
        else:
            base[key] = value


# Module-level settings cache
_cached_settings: Optional[Dict[str, Any]] = None


def get_settings(reload: bool = False) -> Dict[str, Any]:
    """Get cached settings, loading if necessary.

    Args:
        reload: If True, reload settings from disk.

    Returns:
        Settings dictionary.
    """
    global _cached_settings
    if _cached_settings is None or reload:
        _cached_settings = load_settings()
    return _cached_settings


# =============================================================================
# Phase configuration constants
PHASE_CONFIG = {
    1: {
        'name': 'Springs Only',
        'max_iterations': 10,
        'alarm_distance': 10.0,
        'contact_distance': 5.0,
        'position_threshold': 0.2,  # mm change over 100 steps to exit
        'settling_steps': 0,
    },
    2: {
        'name': 'Coarse Collision',
        'max_iterations': 100,
        'alarm_distance': 6.0,
        'contact_distance': 3.0,
        'position_threshold': 0.05,
        'max_constraints': 200,  # Exit if constraints exceed this
        'settling_steps': 50,
    },
    3: {
        'name': 'Fine Collision',
        'max_iterations': 500,
        'alarm_distance': 3.0,
        'contact_distance': 1.0,
        'position_threshold': 0.01,
        'settling_steps': 50,
    },
}


def create_attractive_force_controller(node, rigid_mo, mating_points_local, target_points,
                                        stiffness, damping, mass=0.05,
                                        rotation_stiffness=0.0,
                                        max_angular_velocity=0.0,
                                        debug=False):
    """Create a proper SOFA Python controller for attractive forces.

    This must be called AFTER Sofa.Simulation.init() since it requires
    the Sofa.Core module to be properly initialized.

    Args:
        node: SOFA node to attach controller to
        rigid_mo: SOFA MechanicalObject for the rigid body (Rigid3d)
        mating_points_local: (N, 3) array of mating points in rigid body local frame
        target_points: (N, 3) array of target points in world frame
        stiffness: Spring stiffness per point in SOFA units (kg/s² ≈ mN/mm).
            1.0 here ≈ 0.001 N/mm. Use values like 1-100 for typical simulations.
        damping: Damping coefficient in SOFA units (kg/s ≈ mN·s/mm).
        mass: Rigid body mass in kg (must match guide_mass_kg).
        rotation_stiffness: Torsional spring stiffness pulling toward identity
            rotation. 0 = no constraint.
            Angular damping is auto-computed for critical damping.
        max_angular_velocity: Maximum angular velocity magnitude (rad/s).
            0 = no limit. Prevents wild spins during transient oscillation
            while allowing gradual rotation to the correct seating pose.
            Recommended: 5-20 rad/s.
        debug: If True, print debug info for first few steps

    Returns:
        The controller object
    """
    import Sofa.Core
    from scipy.spatial.transform import Rotation

    class AttractiveForceController(Sofa.Core.Controller):
        """SOFA controller that applies attractive forces to a rigid body.

        This properly integrates with SOFA's animation loop via onAnimateBeginEvent,
        ensuring forces are applied at the correct time each simulation step.

        This bypasses issues with StiffSpringForceField not properly propagating
        forces through RigidMapping.
        """

        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            self.rigid_mo = kwargs.get('rigid_mo')
            self.mating_points_local = np.array(kwargs.get('mating_points_local'))
            self.target_points = np.array(kwargs.get('target_points'))
            self.stiffness = kwargs.get('stiffness', 1.0)
            self.damping = kwargs.get('damping', 1.0)
            self.mass = kwargs.get('mass', 0.05)
            self.rotation_stiffness = kwargs.get('rotation_stiffness', 0.0)
            self.max_angular_velocity = kwargs.get('max_angular_velocity', 0.0)
            # Auto-compute angular damping for critical damping of rotation spring
            if self.rotation_stiffness > 0:
                self.angular_damping = 2.0 * np.sqrt(self.rotation_stiffness * self.mass)
            else:
                self.angular_damping = self.damping * 10.0  # Legacy fallback
            self.debug = kwargs.get('debug', False)
            self.step_count = 0

        def compute_attractive_forces(self):
            """Compute net force and torque on rigid body from attractive springs."""
            # Get current rigid body state
            pos_data = self.rigid_mo.position.value[0]
            pos = np.array(pos_data[:3])
            quat_sofa = pos_data[3:7]  # SOFA: [x, y, z, w]

            # Transform local mating points to world frame
            R = Rotation.from_quat(quat_sofa)  # scipy uses same [x,y,z,w] order
            mating_world = R.apply(self.mating_points_local) + pos

            # Compute spring forces: F = k * (target - current)
            displacements = self.target_points - mating_world
            point_forces = self.stiffness * displacements

            # Get velocity for damping
            vel_data = self.rigid_mo.velocity.value[0]
            linear_vel = np.array(vel_data[:3])
            angular_vel = np.array(vel_data[3:6])

            # Net force = sum of all point forces + linear damping
            damping_force = -self.damping * linear_vel
            net_force = point_forces.sum(axis=0) + damping_force

            # Net torque = sum of r × F for each point (r = vector from COM to point)
            r_vectors = mating_world - pos
            torques = np.cross(r_vectors, point_forces)
            net_torque = torques.sum(axis=0)

            # Rotation constraint: restoring torque toward identity orientation
            if self.rotation_stiffness > 0:
                rotvec = R.as_rotvec()  # axis * angle (zero at identity)
                restoring_torque = -self.rotation_stiffness * rotvec
                net_torque += restoring_torque

            # Angular damping (critically damped if rotation_stiffness > 0)
            net_torque -= self.angular_damping * angular_vel

            return net_force, net_torque

        def onAnimateBeginEvent(self, event):
            """Called at the beginning of each simulation step - apply forces here."""
            net_force, net_torque = self.compute_attractive_forces()

            # Debug output for first few steps
            if self.debug and self.step_count < 10:
                pos_data = self.rigid_mo.position.value[0]
                vel_data = self.rigid_mo.velocity.value[0]
                print(f"[CTRL] Step {self.step_count}: pos={np.array(pos_data[:3]).round(2)}, "
                      f"vel={np.array(vel_data[:3]).round(2)}, force={net_force.round(1)}")

            # Compute acceleration: a = F/m
            # Forces are in SOFA native units (kg·mm/s²), so F/m gives mm/s² directly.
            # Stiffness is in kg/s² (≈ mN/mm), damping is in kg/s (≈ mN·s/mm).
            linear_accel = net_force / self.mass
            angular_accel = net_torque / self.mass

            # Get timestep from root node
            dt = self.rigid_mo.getContext().getDt()

            # Update velocity directly: v_new = v_old + a * dt
            with self.rigid_mo.velocity.writeable() as vel:
                if len(vel) > 0:
                    vel[0][:3] += linear_accel * dt
                    vel[0][3:6] += angular_accel * dt

                    # Clamp angular velocity magnitude
                    if self.max_angular_velocity > 0:
                        ang_vel = np.array(vel[0][3:6])
                        ang_speed = np.linalg.norm(ang_vel)
                        if ang_speed > self.max_angular_velocity:
                            vel[0][3:6] = ang_vel * (self.max_angular_velocity / ang_speed)

            self.step_count += 1

    # Create and add the controller to the node
    controller = AttractiveForceController(
        name="attractiveForceController",
        rigid_mo=rigid_mo,
        mating_points_local=mating_points_local,
        target_points=target_points,
        stiffness=stiffness,
        damping=damping,
        mass=mass,
        rotation_stiffness=rotation_stiffness,
        max_angular_velocity=max_angular_velocity,
        debug=debug,
    )
    node.addObject(controller)

    return controller


# Legacy wrapper for backwards compatibility
class AttractiveForceController:
    """Legacy controller class - use create_attractive_force_controller() instead."""

    def __init__(self, rigid_mo, mating_points_local, target_points, stiffness, damping):
        self.rigid_mo = rigid_mo
        self.mating_points_local = np.array(mating_points_local)
        self.target_points = np.array(target_points)
        self.stiffness = stiffness
        self.damping = damping
        self._sofa_controller = None

    def apply_forces(self, debug_step=None):
        """Legacy method - forces are now applied via SOFA controller."""
        pass  # No-op, SOFA controller handles this now


def create_multi_phase_controller(
    node,
    rigid_mo,
    solver,
    intersection,
    tibia_collision_coarse,
    tibia_collision_fine,
    guide_collision_coarse,
    guide_collision_fine,
    on_converged=None,
):
    """Create a controller that manages multi-phase collision activation.

    This controller monitors simulation convergence and progressively activates
    collision nodes:
    - Phase 1: Springs only (all collision deactivated)
    - Phase 2: Coarse collision (500 faces, Triangle-only)
    - Phase 3: Fine collision (2000 faces, Triangle+Line+Point)

    Args:
        node: SOFA node to attach controller to
        rigid_mo: Guide's rigid MechanicalObject
        solver: GenericConstraintSolver to tune maxIterations
        intersection: LocalMinDistance to tune alarm/contact distances
        tibia_collision_coarse: Tibia coarse collision node
        tibia_collision_fine: Tibia fine collision node
        guide_collision_coarse: Guide coarse collision node
        guide_collision_fine: Guide fine collision node
        on_converged: Optional callback when Phase 3 converges

    Returns:
        The controller object
    """
    import Sofa.Core

    class MultiPhaseCollisionController(Sofa.Core.Controller):
        """Controller for multi-phase progressive collision activation."""

        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            self.rigid_mo = kwargs.get('rigid_mo')
            self.solver = kwargs.get('solver')
            self.intersection = kwargs.get('intersection')
            self.tibia_collision_coarse = kwargs.get('tibia_collision_coarse')
            self.tibia_collision_fine = kwargs.get('tibia_collision_fine')
            self.guide_collision_coarse = kwargs.get('guide_collision_coarse')
            self.guide_collision_fine = kwargs.get('guide_collision_fine')
            self.on_converged = kwargs.get('on_converged')

            self.phase = 1
            self.step_count = 0
            self.phase_step_count = 0
            self.settling_steps_remaining = 0
            self.position_history = []
            self.converged = False
            self.initialized = False

            # Note: Collision nodes will be deactivated after SOFA init completes
            # (in onSimulationInitDoneEvent or first onAnimateBeginEvent)
            print(f"[PHASE] Controller created, will initialize to Phase 1 after SOFA init")

        def _deactivate_all_collision(self):
            """Deactivate all collision nodes."""
            for node in [self.tibia_collision_coarse, self.tibia_collision_fine,
                         self.guide_collision_coarse, self.guide_collision_fine]:
                if node is not None:
                    node.setActive(False)

        def _activate_coarse_collision(self):
            """Activate coarse collision nodes."""
            for node in [self.tibia_collision_coarse, self.guide_collision_coarse]:
                if node is not None:
                    node.setActive(True)
            print(f"[PHASE] Activated coarse collision nodes")

        def _activate_fine_collision(self):
            """Activate fine collision nodes, deactivate coarse."""
            for node in [self.tibia_collision_coarse, self.guide_collision_coarse]:
                if node is not None:
                    node.setActive(False)
            for node in [self.tibia_collision_fine, self.guide_collision_fine]:
                if node is not None:
                    node.setActive(True)
            print(f"[PHASE] Activated fine collision nodes")

        def _apply_phase_config(self, phase):
            """Apply solver/intersection settings for a phase."""
            config = PHASE_CONFIG[phase]
            if self.solver is not None:
                self.solver.maxIterations.value = config['max_iterations']
            if self.intersection is not None:
                self.intersection.alarmDistance.value = config['alarm_distance']
                self.intersection.contactDistance.value = config['contact_distance']

        def _get_position(self):
            """Get current guide position."""
            pos_data = self.rigid_mo.position.value[0]
            return np.array(pos_data[:3])

        def _position_change_over_n_steps(self, n):
            """Calculate position change over last n steps."""
            if len(self.position_history) < n + 1:
                return float('inf')
            old_pos = self.position_history[-(n + 1)]
            new_pos = self.position_history[-1]
            return np.linalg.norm(new_pos - old_pos)

        def _get_constraint_count(self):
            """Get current number of active constraints."""
            if self.solver is not None:
                try:
                    return self.solver.currentNumConstraints.value
                except:
                    return 0
            return 0

        def _transition_to_phase(self, new_phase):
            """Transition to a new phase."""
            old_phase = self.phase
            self.phase = new_phase
            self.phase_step_count = 0
            config = PHASE_CONFIG[new_phase]

            print(f"[PHASE] Transitioning from Phase {old_phase} to Phase {new_phase}: {config['name']}")
            print(f"[PHASE]   After {self.step_count} total steps")

            # Activate appropriate collision nodes
            if new_phase == 2:
                self._activate_coarse_collision()
            elif new_phase == 3:
                self._activate_fine_collision()

            # Apply phase configuration
            self._apply_phase_config(new_phase)

            # Start settling period
            self.settling_steps_remaining = config['settling_steps']
            if self.settling_steps_remaining > 0:
                print(f"[PHASE]   Starting {self.settling_steps_remaining}-step settling period")

        def onAnimateBeginEvent(self, event):
            """Called at beginning of each simulation step."""
            # Initialize on first step (after SOFA init has completed)
            if not self.initialized:
                self.initialized = True
                self._deactivate_all_collision()
                self._apply_phase_config(1)
                print(f"[PHASE] Initialized in Phase 1: {PHASE_CONFIG[1]['name']}")

        def onAnimateEndEvent(self, event):
            """Called at end of each simulation step."""
            self.step_count += 1
            self.phase_step_count += 1

            # Record position
            current_pos = self._get_position()
            self.position_history.append(current_pos.copy())

            # Keep history bounded
            if len(self.position_history) > 200:
                self.position_history = self.position_history[-200:]

            # Handle settling period
            if self.settling_steps_remaining > 0:
                self.settling_steps_remaining -= 1
                if self.settling_steps_remaining == 0:
                    print(f"[PHASE] Settling period complete for Phase {self.phase}")
                return

            # Check phase transition conditions (every 100 steps after settling)
            if self.phase_step_count % 100 != 0:
                return

            config = PHASE_CONFIG[self.phase]
            pos_change = self._position_change_over_n_steps(100)
            constraints = self._get_constraint_count()

            # Log status
            if self.step_count % 100 == 0:
                print(f"[PHASE] Step {self.step_count} (Phase {self.phase}): "
                      f"pos_change={pos_change:.4f}mm, constraints={constraints}")

            if self.phase == 1:
                # Exit Phase 1 when position change < threshold
                if pos_change < config['position_threshold']:
                    self._transition_to_phase(2)

            elif self.phase == 2:
                # Exit Phase 2 when position change < threshold AND constraints reasonable
                max_constraints = config.get('max_constraints', float('inf'))
                if pos_change < config['position_threshold'] and constraints < max_constraints:
                    self._transition_to_phase(3)

            elif self.phase == 3:
                # Exit Phase 3 (converged) when position change < threshold
                if pos_change < config['position_threshold']:
                    self.converged = True
                    print(f"[PHASE] CONVERGED at step {self.step_count} in Phase 3")
                    print(f"[PHASE]   Final position: {current_pos.round(3)}")
                    if self.on_converged:
                        self.on_converged()

    controller = MultiPhaseCollisionController(
        name="multiPhaseController",
        rigid_mo=rigid_mo,
        solver=solver,
        intersection=intersection,
        tibia_collision_coarse=tibia_collision_coarse,
        tibia_collision_fine=tibia_collision_fine,
        guide_collision_coarse=guide_collision_coarse,
        guide_collision_fine=guide_collision_fine,
        on_converged=on_converged,
    )
    node.addObject(controller)

    return controller


def identify_mating_points(
    guide_mesh: trimesh.Trimesh,
    bone_mesh: trimesh.Trimesh,
    n_points: int = 200,
    seed: int = 42,
    distance_threshold: Optional[float] = None,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Identify points on guide that should be attracted to bone.

    Returns points in guide's LOCAL coordinate frame (relative to centroid).

    The target points are computed using trimesh.proximity.closest_point() to find
    the actual closest point on the bone mesh surface, not just the closest point
    in a sampled point cloud. This eliminates the ~2.6mm bias that occurred when
    using KD-tree lookups on sparse bone samples.

    Args:
        guide_mesh: Guide mesh
        bone_mesh: Bone mesh for reference
        n_points: Number of sample points
        seed: Random seed for reproducible surface sampling
        distance_threshold: Maximum distance (mm) from bone surface for a point to
                           be considered a mating point. If None, uses value from
                           settings.yaml (default: 1.0mm). See docs/settings-reference.md.

    Returns:
        mating_points_local: (N, 3) points on guide mating surface IN LOCAL FRAME
        target_points_world: (N, 3) corresponding closest points on bone IN WORLD FRAME
        guide_centroid: (3,) guide centroid (for coordinate transforms)
    """
    from trimesh.proximity import closest_point

    # Load distance threshold from settings if not provided
    if distance_threshold is None:
        settings = get_settings()
        distance_threshold = settings['mating_points']['distance_threshold_mm']

    rng = np.random.RandomState(seed)
    guide_centroid = guide_mesh.centroid.copy()

    # Sample guide surface (world coordinates) — use seeded RNG for reproducibility
    samples_world, face_indices = trimesh.sample.sample_surface(guide_mesh, n_points * 3, seed=rng)
    normals = guide_mesh.face_normals[face_indices]

    # Find closest points on bone mesh surface (not sampled points!)
    # This is the key fix: closest_point returns the actual closest point on
    # the mesh surface, eliminating the bias from sparse sampling.
    closest_bone_points, distances, _ = closest_point(bone_mesh, samples_world)

    # Filter to mating surface: use an absolute distance threshold rather than
    # percentile. For a guide that should seat directly on the bone, we want
    # only points that are very close to contact. Using percentile-based
    # thresholds was including points 3-8mm away, creating spring bias.
    # See docs/settings-reference.md for tuning guidance.
    proximity_mask = distances < distance_threshold

    # Filter by normal orientation: keep only points whose normals point
    # toward the bone (concave/inner surface). Without this filter, both
    # inner and outer guide surfaces are selected, creating a systematic
    # bias that pulls the guide off the designed pose.
    toward_bone = closest_bone_points - samples_world
    toward_bone_norm = toward_bone / (np.linalg.norm(toward_bone, axis=1, keepdims=True) + 1e-12)
    normal_dots = np.sum(normals * toward_bone_norm, axis=1)
    normal_mask = normal_dots > 0  # Normal points toward bone = inner surface

    mating_mask = proximity_mask & normal_mask

    # Select mating points (world coordinates)
    mating_points_world = samples_world[mating_mask][:n_points]
    target_bone_points = closest_bone_points[mating_mask][:n_points]

    # Convert mating points to LOCAL frame (relative to guide centroid)
    # This is critical for RigidMapping to work correctly
    mating_points_local = mating_points_world - guide_centroid

    return mating_points_local, target_bone_points, guide_centroid


def create_attractive_scene(
    tibia_mesh_path: str,
    guide_mesh_path: str,
    mating_points: np.ndarray,
    target_points: np.ndarray,
    guide_initial_position: Optional[List[float]] = None,
    guide_initial_quaternion: Optional[List[float]] = None,
    guide_mass_kg: float = 0.05,
    spring_stiffness: float = 100.0,
    spring_damping: float = 1.0,
    rotation_stiffness: float = 0.0,
    max_angular_velocity: float = 0.0,
    gravity: Optional[List[float]] = None,
    dt: float = 0.001,
    gui: bool = False,
    enable_collision: bool = True,
    use_controller: bool = False,
    tibia_collision_path: Optional[str] = None,
    guide_collision_path: Optional[str] = None,
    # Multi-phase collision paths (new)
    tibia_collision_coarse_path: Optional[str] = None,
    tibia_collision_fine_path: Optional[str] = None,
    guide_collision_coarse_path: Optional[str] = None,
    guide_collision_fine_path: Optional[str] = None,
    multi_phase_collision: bool = False,
):
    """Create SOFA scene with attractive springs between guide and bone.

    Args:
        tibia_mesh_path: Path to tibia collision mesh (STL).
        guide_mesh_path: Path to guide collision mesh (STL).
        mating_points: (N, 3) points on guide surface (in guide local frame).
        target_points: (N, 3) target points on bone surface (world frame).
        guide_initial_position: Initial guide position [x, y, z] in mm.
        guide_initial_quaternion: Initial guide orientation [w, x, y, z].
        guide_mass_kg: Guide mass in kilograms.
        spring_stiffness: Stiffness of attractive springs (N/mm).
        spring_damping: Damping coefficient for springs.
        gravity: Gravity vector [gx, gy, gz] in mm/s^2.
        dt: Simulation timestep in seconds.
        gui: If True, add visual models for GUI rendering.

    Returns:
        SOFA root node.
    """
    import sys
    import time
    import Sofa
    import Sofa.Core

    def log(msg):
        print(msg, flush=True)

    log("[SCENE] Starting scene creation...")
    t0 = time.time()

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
        "Sofa.Component.Mapping.Linear",
        "Sofa.Component.Mass",
        "Sofa.Component.ODESolver.Backward",
        "Sofa.Component.StateContainer",
        "Sofa.Component.Topology.Container.Constant",
        "Sofa.Component.Constraint.Projective",
        "Sofa.Component.SolidMechanics.Spring",
    ]

    if gui:
        plugins.extend([
            "Sofa.GL.Component.Rendering3D",
            "Sofa.GL.Component.Shader",
        ])

    root.addObject("RequiredPlugin", pluginName=plugins)
    log(f"[SCENE] Plugins loaded: {time.time() - t0:.2f}s")

    # Animation loop and collision
    # Store references for multi-phase controller
    solver_ref = None
    intersection_ref = None

    if enable_collision:
        # Use parallel options for better performance
        root.addObject("FreeMotionAnimationLoop",
                       parallelCollisionDetectionAndFreeMotion=True)
        root.addObject("CollisionPipeline")
        root.addObject("BruteForceBroadPhase")
        root.addObject("BVHNarrowPhase")
        intersection_ref = root.addObject(
            "LocalMinDistance",
            name="intersection",
            alarmDistance=5.0,
            contactDistance=2.0,
            angleCone=0.1,
        )
        root.addObject("CollisionResponse", response="FrictionContactConstraint")
        # Use parallel solver with UnbuiltGaussSeidel for better performance
        solver_ref = root.addObject("GenericConstraintSolver",
                                    name="constraintSolver",
                                    maxIterations=1000,
                                    tolerance=0.001,
                                    multithreading=True,
                                    resolutionMethod="UnbuiltGaussSeidel")
        log(f"[SCENE] Collision pipeline configured (parallel): {time.time() - t0:.2f}s")
    else:
        root.addObject("DefaultAnimationLoop")
        log(f"[SCENE] Using DefaultAnimationLoop (no collision): {time.time() - t0:.2f}s")

    # ===================
    # Tibia (fixed) - only needed for collision, but load for visual reference
    # ===================
    log("[SCENE] Creating Tibia node...")
    tibia = root.addChild("Tibia")
    tibia.addObject("MeshSTLLoader", name="loader", filename=tibia_mesh_path)
    tibia.addObject("MeshTopology", src="@loader")
    tibia.addObject("MechanicalObject", name="mstate", src="@loader")

    # Collision node references for multi-phase controller
    tibia_collision_coarse_node = None
    tibia_collision_fine_node = None

    if enable_collision:
        if multi_phase_collision and tibia_collision_coarse_path and tibia_collision_fine_path:
            # Multi-phase: create dual collision nodes (both start DEACTIVATED)
            # Coarse collision: Triangle-only (fewer collision elements)
            # Note: Nodes must remain active during init(), deactivated after by controller
            tibia_collision_coarse_node = tibia.addChild("CollisionCoarse")
            tibia_collision_coarse_node.addObject("MeshSTLLoader", name="loader",
                                                   filename=tibia_collision_coarse_path)
            tibia_collision_coarse_node.addObject("MeshTopology", src="@loader")
            tibia_collision_coarse_node.addObject("MechanicalObject", name="collisionMO", src="@loader")
            tibia_collision_coarse_node.addObject("TriangleCollisionModel", simulated=False, moving=False)
            # No Line/Point for coarse - 3x fewer collision elements
            log(f"[SCENE] Tibia coarse collision node created (will deactivate after init)")

            # Fine collision: Triangle + Line + Point (full accuracy)
            tibia_collision_fine_node = tibia.addChild("CollisionFine")
            tibia_collision_fine_node.addObject("MeshSTLLoader", name="loader",
                                                 filename=tibia_collision_fine_path)
            tibia_collision_fine_node.addObject("MeshTopology", src="@loader")
            tibia_collision_fine_node.addObject("MechanicalObject", name="collisionMO", src="@loader")
            tibia_collision_fine_node.addObject("TriangleCollisionModel", simulated=False, moving=False)
            tibia_collision_fine_node.addObject("LineCollisionModel", simulated=False, moving=False)
            tibia_collision_fine_node.addObject("PointCollisionModel", simulated=False, moving=False)
            log(f"[SCENE] Tibia fine collision node created (will deactivate after init)")

        elif tibia_collision_path:
            # Single decimated collision mesh (legacy mode)
            tibia_collision = tibia.addChild("Collision")
            tibia_collision.addObject("MeshSTLLoader", name="loader", filename=tibia_collision_path)
            tibia_collision.addObject("MeshTopology", src="@loader")
            tibia_collision.addObject("MechanicalObject", name="collisionMO", src="@loader")
            tibia_collision.addObject("TriangleCollisionModel", simulated=False, moving=False)
            tibia_collision.addObject("LineCollisionModel", simulated=False, moving=False)
            tibia_collision.addObject("PointCollisionModel", simulated=False, moving=False)
        else:
            # Full mesh collision (original behavior)
            tibia.addObject("TriangleCollisionModel", simulated=False, moving=False)
            tibia.addObject("LineCollisionModel", simulated=False, moving=False)
            tibia.addObject("PointCollisionModel", simulated=False, moving=False)
    log(f"[SCENE] Tibia node created: {time.time() - t0:.2f}s")

    if gui:
        tibia_visual = tibia.addChild("Visual")
        tibia_visual.addObject("OglModel", name="VisualModel", src="@../loader",
                               color="0.9 0.9 0.8 0.7")
        tibia_visual.addObject("IdentityMapping", input="@../mstate", output="@VisualModel")

    # ===================
    # Target points (fixed points on bone that springs attach to)
    # ===================
    log("[SCENE] Creating Targets node...")
    targets = root.addChild("Targets")
    # Format positions as flat list
    target_positions = target_points.flatten().tolist()
    targets.addObject(
        "MechanicalObject",
        name="targetMO",
        template="Vec3d",
        position=target_positions,
    )
    targets.addObject("FixedProjectiveConstraint", indices=list(range(len(target_points))))
    log(f"[SCENE] Targets node created: {time.time() - t0:.2f}s")

    # ===================
    # Guide (dynamic rigid body)
    # ===================
    log("[SCENE] Creating Guide node...")
    guide = root.addChild("Guide")
    guide.addObject("EulerImplicitSolver", rayleighStiffness=0.1, rayleighMass=0.1)
    guide.addObject("CGLinearSolver", iterations=50, tolerance=1e-10, threshold=1e-10)
    guide.addObject(
        "MechanicalObject",
        name="rigidMO",
        template="Rigid3d",
        position=guide_rigid_position,
        externalForce=[0, 0, 0, 0, 0, 0],  # Initialize for controller to use
    )
    guide.addObject("UniformMass", totalMass=guide_mass_kg)
    if enable_collision:
        guide.addObject("UncoupledConstraintCorrection")

    # Guide collision mesh (only if collision enabled)
    guide_collision_coarse_node = None
    guide_collision_fine_node = None

    if enable_collision:
        if multi_phase_collision and guide_collision_coarse_path and guide_collision_fine_path:
            # Multi-phase: create dual collision nodes
            # Note: Nodes must remain active during init(), deactivated after by controller
            # Coarse collision: Triangle-only
            guide_collision_coarse_node = guide.addChild("CollisionCoarse")
            guide_collision_coarse_node.addObject("MeshSTLLoader", name="loader",
                                                   filename=guide_collision_coarse_path)
            guide_collision_coarse_node.addObject("MeshTopology", src="@loader")
            guide_collision_coarse_node.addObject("MechanicalObject", name="collisionMO", src="@loader")
            guide_collision_coarse_node.addObject("TriangleCollisionModel", contactStiffness=10)
            # No Line/Point for coarse
            guide_collision_coarse_node.addObject("RigidMapping", input="@../rigidMO", output="@collisionMO")
            log(f"[SCENE] Guide coarse collision node created (will deactivate after init)")

            # Fine collision: Triangle + Line + Point
            guide_collision_fine_node = guide.addChild("CollisionFine")
            guide_collision_fine_node.addObject("MeshSTLLoader", name="loader",
                                                 filename=guide_collision_fine_path)
            guide_collision_fine_node.addObject("MeshTopology", src="@loader")
            guide_collision_fine_node.addObject("MechanicalObject", name="collisionMO", src="@loader")
            guide_collision_fine_node.addObject("TriangleCollisionModel", contactStiffness=10)
            guide_collision_fine_node.addObject("LineCollisionModel", contactStiffness=10)
            guide_collision_fine_node.addObject("PointCollisionModel", contactStiffness=10)
            guide_collision_fine_node.addObject("RigidMapping", input="@../rigidMO", output="@collisionMO")
            log(f"[SCENE] Guide fine collision node created (will deactivate after init)")

        else:
            # Single collision mesh (legacy mode)
            collision_mesh_path = guide_collision_path if guide_collision_path else guide_mesh_path
            guide_collision = guide.addChild("Collision")
            guide_collision.addObject("MeshSTLLoader", name="loader", filename=collision_mesh_path)
            guide_collision.addObject("MeshTopology", src="@loader")
            guide_collision.addObject("MechanicalObject", name="collisionMO", src="@loader")
            guide_collision.addObject("TriangleCollisionModel", contactStiffness=10)
            guide_collision.addObject("LineCollisionModel", contactStiffness=10)
            guide_collision.addObject("PointCollisionModel", contactStiffness=10)
            guide_collision.addObject("RigidMapping", input="@../rigidMO", output="@collisionMO")
            log(f"[SCENE] Guide collision mesh created: {time.time() - t0:.2f}s")
    else:
        log(f"[SCENE] Skipped guide collision mesh (collision disabled): {time.time() - t0:.2f}s")

    # ===================
    # Mating surface points (attached to guide via rigid mapping)
    # These will be connected to target points via springs
    # ===================
    log("[SCENE] Creating MatingPoints node...")
    guide_mating = guide.addChild("MatingPoints")
    mating_positions = mating_points.flatten().tolist()
    guide_mating.addObject(
        "MechanicalObject",
        name="matingMO",
        template="Vec3d",
        position=mating_positions,
    )
    guide_mating.addObject("RigidMapping", input="@../rigidMO", output="@matingMO")
    log(f"[SCENE] Mating points created: {time.time() - t0:.2f}s")

    # ===================
    # Attractive forces - either via springs or controller
    # ===================
    # Store controller params - actual SOFA controller created after init()
    controller_params = None
    if use_controller:
        # Will use Python controller approach (bypasses mapping issues)
        # Controller must be created AFTER Sofa.Simulation.init()
        log(f"[SCENE] Will use AttractiveForceController for {len(mating_points)} points (created after init)...")
        controller_params = {
            'mating_points_local': mating_points,
            'target_points': target_points,
            'stiffness': spring_stiffness,
            'damping': spring_damping,
            'mass': guide_mass_kg,
            'rotation_stiffness': rotation_stiffness,
            'max_angular_velocity': max_angular_velocity,
        }
    else:
        # Use StiffSpringForceField (may have issues with RigidMapping)
        # Format: pairs of indices (index1_obj1, index1_obj2, stiffness, damping, restLength)
        log(f"[SCENE] Creating {len(mating_points)} springs...")
        n_springs = len(mating_points)
        springs_data = []
        for i in range(n_springs):
            # Spring from mating point i to target point i
            # Rest length of 0 means spring pulls to coincident
            springs_data.extend([i, i, spring_stiffness, spring_damping, 0.0])
        log(f"[SCENE] Springs data prepared: {time.time() - t0:.2f}s")

        # Add the spring force field
        # Note: StiffSpringForceField needs to be at root level to connect different objects
        log("[SCENE] Adding StiffSpringForceField...")
        root.addObject(
            "StiffSpringForceField",
            name="attractiveSprings",
            object1="@Guide/MatingPoints/matingMO",
            object2="@Targets/targetMO",
            spring=springs_data,
        )
        log(f"[SCENE] StiffSpringForceField added: {time.time() - t0:.2f}s")

    if gui:
        guide_visual = guide.addChild("Visual")
        guide_visual.addObject("MeshSTLLoader", name="loader", filename=guide_mesh_path)
        guide_visual.addObject("OglModel", name="VisualModel", src="@loader",
                               color="0.27 0.51 0.71 1.0")
        guide_visual.addObject("RigidMapping", input="@../rigidMO", output="@VisualModel")

    log(f"[SCENE] Scene creation complete: {time.time() - t0:.2f}s")

    # Build scene info dict with references needed for multi-phase controller
    scene_info = {
        'controller_params': controller_params,
        'solver': solver_ref,
        'intersection': intersection_ref,
        'tibia_collision_coarse': tibia_collision_coarse_node,
        'tibia_collision_fine': tibia_collision_fine_node,
        'guide_collision_coarse': guide_collision_coarse_node,
        'guide_collision_fine': guide_collision_fine_node,
        'multi_phase_collision': multi_phase_collision,
    }

    return root, scene_info


def extract_guide_pose(root) -> Pose:
    """Extract current guide pose from SOFA scene."""
    guide = root.getChild("Guide")
    mech = guide.getObject("rigidMO")

    position_data = mech.position.value[0]
    x, y, z = position_data[:3]
    qx, qy, qz, qw = position_data[3:7]

    return Pose(
        position=np.array([x, y, z]),
        quaternion=np.array([qw, qx, qy, qz]),
    )


def run_attractive_seating(
    tibia_mesh: trimesh.Trimesh,
    guide_mesh: trimesh.Trimesh,
    initial_position: Optional[np.ndarray] = None,
    initial_quaternion: Optional[np.ndarray] = None,
    initial_offset: Optional[np.ndarray] = None,
    spring_stiffness: float = 50.0,
    spring_damping: float = 1.0,
    rotation_stiffness: float = 0.0,
    max_angular_velocity: float = 0.0,
    n_points: int = 150,
    seed: int = 42,
    max_time: float = 5.0,
    dt: float = 0.001,
    convergence_threshold: float = 0.01,
    gui: bool = False,
    enable_collision: bool = True,
    use_controller: bool = False,
    gravity: Optional[List[float]] = None,
    collision_faces: int = 5000,  # Legacy single-mesh decimation
    collision_faces_coarse: int = 0,  # Multi-phase coarse (0 = use legacy)
    collision_faces_fine: int = 0,  # Multi-phase fine (0 = use legacy)
) -> dict:
    """Run attractive seating simulation.

    Args:
        tibia_mesh: Tibia mesh
        guide_mesh: Guide mesh
        initial_position: Starting position (if None, use mesh centroid)
        initial_quaternion: Starting orientation [w,x,y,z] (if None, identity)
        initial_offset: Offset vector [x,y,z] to move guide away from starting position.
                       Use this to prevent initial collision (e.g., [0, 0, 5] for 5mm up).
        spring_stiffness: Stiffness of attractive springs
        max_time: Maximum simulation time
        dt: Timestep
        convergence_threshold: Position change threshold for convergence
        gui: Run with SOFA GUI

    Returns:
        dict with 'pose', 'converged', 'steps', 'energy'
    """
    import tempfile
    from pathlib import Path

    import Sofa
    import Sofa.Simulation

    # Identify mating points (local frame) and targets (world frame)
    mating_points_local, target_points, guide_centroid = identify_mating_points(
        guide_mesh, tibia_mesh, n_points=n_points, seed=seed
    )
    print(f"Using {len(mating_points_local)} mating points")

    # Set up initial pose (rigid body starts at guide centroid)
    if initial_position is None:
        initial_position = guide_centroid.copy()
    if initial_quaternion is None:
        initial_quaternion = np.array([1.0, 0.0, 0.0, 0.0])

    # Apply offset to prevent initial collision
    if initial_offset is not None:
        initial_position = initial_position + np.array(initial_offset)
        print(f"Applied initial offset {initial_offset}, new position: {initial_position}")

    # Center the guide mesh at origin for SOFA (since rigid body position is the offset)
    guide_mesh_centered = guide_mesh.copy()
    guide_mesh_centered.vertices = guide_mesh_centered.vertices - guide_centroid

    # Save meshes to temp files
    with tempfile.TemporaryDirectory(prefix="sofa_attractive_") as temp_dir:
        temp_path = Path(temp_dir)
        tibia_path = temp_path / "tibia.stl"
        guide_path = temp_path / "guide.stl"

        tibia_mesh.export(str(tibia_path))
        guide_mesh_centered.export(str(guide_path))  # Use centered mesh

        # Decimate meshes for collision if collision is enabled
        tibia_collision_path = None
        guide_collision_path = None
        tibia_collision_coarse_path = None
        tibia_collision_fine_path = None
        guide_collision_coarse_path = None
        guide_collision_fine_path = None
        multi_phase_collision = False

        if enable_collision:
            if collision_faces_coarse > 0 and collision_faces_fine > 0:
                # Multi-phase collision: create coarse and fine decimations
                multi_phase_collision = True
                print(f"[DECIMATION] Multi-phase collision enabled")

                # Coarse decimation
                tibia_coarse = tibia_mesh.simplify_quadric_decimation(collision_faces_coarse)
                print(f"[DECIMATION] Tibia coarse: {len(tibia_mesh.faces)} -> {len(tibia_coarse.faces)} faces")
                tibia_coarse_path = temp_path / "tibia_collision_coarse.stl"
                tibia_coarse.export(str(tibia_coarse_path))
                tibia_collision_coarse_path = str(tibia_coarse_path)

                guide_coarse = guide_mesh_centered.simplify_quadric_decimation(collision_faces_coarse)
                print(f"[DECIMATION] Guide coarse: {len(guide_mesh_centered.faces)} -> {len(guide_coarse.faces)} faces")
                guide_coarse_path = temp_path / "guide_collision_coarse.stl"
                guide_coarse.export(str(guide_coarse_path))
                guide_collision_coarse_path = str(guide_coarse_path)

                # Fine decimation
                tibia_fine = tibia_mesh.simplify_quadric_decimation(collision_faces_fine)
                print(f"[DECIMATION] Tibia fine: {len(tibia_mesh.faces)} -> {len(tibia_fine.faces)} faces")
                tibia_fine_path = temp_path / "tibia_collision_fine.stl"
                tibia_fine.export(str(tibia_fine_path))
                tibia_collision_fine_path = str(tibia_fine_path)

                guide_fine = guide_mesh_centered.simplify_quadric_decimation(collision_faces_fine)
                print(f"[DECIMATION] Guide fine: {len(guide_mesh_centered.faces)} -> {len(guide_fine.faces)} faces")
                guide_fine_path = temp_path / "guide_collision_fine.stl"
                guide_fine.export(str(guide_fine_path))
                guide_collision_fine_path = str(guide_fine_path)

            elif collision_faces > 0:
                # Legacy single-mesh decimation
                tibia_decimated = tibia_mesh.simplify_quadric_decimation(collision_faces)
                print(f"Decimated tibia: {len(tibia_mesh.faces)} -> {len(tibia_decimated.faces)} faces")
                tibia_col_path = temp_path / "tibia_collision.stl"
                tibia_decimated.export(str(tibia_col_path))
                tibia_collision_path = str(tibia_col_path)

                guide_decimated = guide_mesh_centered.simplify_quadric_decimation(collision_faces)
                print(f"Decimated guide: {len(guide_mesh_centered.faces)} -> {len(guide_decimated.faces)} faces")
                guide_col_path = temp_path / "guide_collision.stl"
                guide_decimated.export(str(guide_col_path))
                guide_collision_path = str(guide_col_path)

        # Create scene
        root, scene_info = create_attractive_scene(
            tibia_mesh_path=str(tibia_path),
            guide_mesh_path=str(guide_path),
            mating_points=mating_points_local,  # Local frame points
            target_points=target_points,  # World frame targets
            guide_initial_position=initial_position.tolist(),
            guide_initial_quaternion=initial_quaternion.tolist(),
            spring_stiffness=spring_stiffness,
            spring_damping=spring_damping,
            rotation_stiffness=rotation_stiffness,
            max_angular_velocity=max_angular_velocity,
            dt=dt,
            gui=gui,
            enable_collision=enable_collision,
            use_controller=use_controller,
            gravity=gravity,
            tibia_collision_path=tibia_collision_path,
            guide_collision_path=guide_collision_path,
            tibia_collision_coarse_path=tibia_collision_coarse_path,
            tibia_collision_fine_path=tibia_collision_fine_path,
            guide_collision_coarse_path=guide_collision_coarse_path,
            guide_collision_fine_path=guide_collision_fine_path,
            multi_phase_collision=multi_phase_collision,
        )

        # Extract references from scene_info
        controller_params = scene_info['controller_params']

        import time
        print(f"[TIMING] Initializing SOFA scene...")
        t_init_start = time.time()
        Sofa.Simulation.init(root)
        t_init_end = time.time()
        print(f"[TIMING] Scene init completed in {t_init_end - t_init_start:.2f}s")

        # Create SOFA controller AFTER init (requires Sofa.Core to be initialized)
        if controller_params is not None:
            print(f"[TIMING] Creating SOFA AttractiveForceController...")
            guide_node = root.getChild("Guide")
            rigid_mo = guide_node.getObject("rigidMO")
            create_attractive_force_controller(
                node=guide_node,
                rigid_mo=rigid_mo,
                mating_points_local=controller_params['mating_points_local'],
                target_points=controller_params['target_points'],
                stiffness=controller_params['stiffness'],
                damping=controller_params['damping'],
                mass=controller_params['mass'],
                rotation_stiffness=controller_params['rotation_stiffness'],
                max_angular_velocity=controller_params['max_angular_velocity'],
                debug=True,
            )
            print(f"[TIMING] Controller attached to Guide node")

        # Create multi-phase collision controller if enabled
        multi_phase_controller = None
        if scene_info['multi_phase_collision']:
            print(f"[TIMING] Creating MultiPhaseCollisionController...")
            guide_node = root.getChild("Guide")
            rigid_mo = guide_node.getObject("rigidMO")
            multi_phase_controller = create_multi_phase_controller(
                node=root,
                rigid_mo=rigid_mo,
                solver=scene_info['solver'],
                intersection=scene_info['intersection'],
                tibia_collision_coarse=scene_info['tibia_collision_coarse'],
                tibia_collision_fine=scene_info['tibia_collision_fine'],
                guide_collision_coarse=scene_info['guide_collision_coarse'],
                guide_collision_fine=scene_info['guide_collision_fine'],
            )
            print(f"[TIMING] MultiPhaseCollisionController attached")

        if gui:
            import Sofa.Gui
            Sofa.Gui.GUIManager.Init("main", "qglviewer")
            Sofa.Gui.GUIManager.createGUI(root, __file__)
            Sofa.Gui.GUIManager.SetDimension(1200, 800)
            Sofa.Gui.GUIManager.MainLoop(root)
            Sofa.Gui.GUIManager.closeGUI()

            final_pose = extract_guide_pose(root)
            return {
                'pose': final_pose,
                'converged': True,
                'steps': -1,
            }

        # Run simulation
        max_steps = int(max_time / dt)
        prev_pos = np.array([np.inf, np.inf, np.inf])
        converged = False

        print(f"[TIMING] Starting simulation loop ({max_steps} steps max)...")
        t_sim_start = time.time()
        step_times = []
        phase_step_counts = {1: 0, 2: 0, 3: 0}

        for step in range(max_steps):
            # Note: If using controller, forces are applied automatically
            # via onAnimateBeginEvent in the SOFA controller

            t_step_start = time.time()
            Sofa.Simulation.animate(root, dt)
            t_step_end = time.time()
            step_time_ms = (t_step_end - t_step_start) * 1000
            step_times.append(step_time_ms)

            # Track phase step counts
            if multi_phase_controller is not None:
                current_phase = multi_phase_controller.phase
                phase_step_counts[current_phase] = phase_step_counts.get(current_phase, 0) + 1

            # Print every step for first 10, then every 100
            if step < 10 or step % 100 == 0:
                pose = extract_guide_pose(root)
                pos_change = np.linalg.norm(pose.position - prev_pos)
                avg_step_time = sum(step_times) / len(step_times)

                phase_info = ""
                if multi_phase_controller is not None:
                    phase_info = f" | phase={multi_phase_controller.phase}"

                print(f"[TIMING] Step {step}: {step_time_ms:.1f}ms (avg: {avg_step_time:.1f}ms) | "
                      f"pos={pose.position.round(2)} | change={pos_change:.4f}{phase_info}")

                # Check for convergence
                if multi_phase_controller is not None:
                    # Use multi-phase controller's convergence status
                    if multi_phase_controller.converged:
                        converged = True
                        print(f"[TIMING] Multi-phase converged at step {step}")
                        break
                elif pos_change < convergence_threshold and step > 100:
                    converged = True
                    print(f"[TIMING] Converged at step {step}")
                    break

                prev_pos = pose.position.copy()

        t_sim_end = time.time()
        print(f"[TIMING] Simulation loop completed in {t_sim_end - t_sim_start:.2f}s "
              f"({step + 1} steps, avg {sum(step_times)/len(step_times):.1f}ms/step)")

        if multi_phase_controller is not None:
            print(f"[TIMING] Phase step counts: {phase_step_counts}")

        final_pose = extract_guide_pose(root)

        result = {
            'pose': final_pose,
            'converged': converged,
            'steps': step + 1,
        }

        if multi_phase_controller is not None:
            result['phase_step_counts'] = phase_step_counts
            result['final_phase'] = multi_phase_controller.phase

        return result
