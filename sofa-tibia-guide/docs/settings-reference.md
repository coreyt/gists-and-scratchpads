# Settings Reference

This document provides detailed documentation for all tunable parameters in the attractive seating simulation. Parameters are configured in `settings.yaml` at the project root.

## Quick Start

For most use cases, the default settings work well. If you need to adjust behavior:

| Problem | Solution |
|---------|----------|
| Guide settles above the bone | Reduce `mating_points.distance_threshold_mm` to 0.5 |
| Too few mating points found | Increase `mating_points.distance_threshold_mm` to 2.0 |
| Guide oscillates without settling | Increase `springs.damping` to 15-20 |
| Guide spins wildly | Set `rotation.max_angular_velocity` to 5.0 |
| Simulation too slow | Reduce `collision.multi_phase.fine_faces` to 1500 |
| Guide penetrates bone | Increase `collision.distances.contact_distance` |

---

## Mating Point Identification

The algorithm identifies which points on the guide surface should be attracted to the bone. This is critical for accurate seating.

### `mating_points.distance_threshold_mm`

**Type:** float
**Default:** 1.0
**Range:** 0.1 - 5.0

The maximum distance (in mm) from the bone surface for a guide point to be considered a "mating point". Only guide surface points within this distance of the bone (at the designed pose) are selected.

**How it works:**
1. Sample points uniformly across the guide surface
2. Find the closest point on the bone mesh for each sample
3. Keep only points where distance < threshold AND normal points toward bone

**Tuning guidance:**

| Value | Effect |
|-------|--------|
| 0.5 mm | Very strict - only actual contact points. May result in <30 points. |
| 1.0 mm | Default - near-contact points. Typically 20-50 points. |
| 2.0 mm | Loose - includes points near but not touching bone. 50-100 points. |
| 5.0 mm | Very loose - includes surrounding area. May cause bias. |

**Warning:** Using values >2mm can cause the guide to settle above the bone because points far from the contact surface create a spring bias that pulls the guide away from optimal seating.

### `mating_points.n_points`

**Type:** integer
**Default:** 150
**Range:** 20 - 500

Maximum number of mating points to use. If fewer qualifying points are found, all are used.

More points provide more stable forces but increase computation time. For most guides, 50-150 points is sufficient.

### `mating_points.seed`

**Type:** integer
**Default:** 42

Random seed for surface sampling. Use the same seed for reproducible results across runs.

---

## Spring Physics

Springs connect each mating point on the guide to its corresponding target point on the bone. These parameters control the attractive force behavior.

### `springs.stiffness`

**Type:** float
**Default:** 50.0
**Range:** 10 - 200

Spring stiffness in SOFA units (approximately equivalent to N/mm when combined with the mass model). Higher values create stronger attraction.

**Tuning guidance:**

| Value | Behavior |
|-------|----------|
| 10-20 | Gentle attraction, slow convergence, very stable |
| 50 | Default - balanced speed and stability |
| 100-200 | Strong attraction, fast convergence, may oscillate |

If the guide oscillates around the equilibrium without settling, reduce stiffness or increase damping.

### `springs.damping`

**Type:** float
**Default:** 10.0
**Range:** 1 - 50

Damping coefficient that dissipates kinetic energy. Higher values reduce oscillation.

**Critical damping formula:**
```
damping_critical = 2 * sqrt(stiffness * mass)
```

With default stiffness=50 and mass=0.05kg:
```
damping_critical ≈ 2 * sqrt(50 * 0.05) ≈ 3.2
```

Using damping > critical (e.g., 10.0) provides overdamped behavior that settles quickly without oscillation.

---

## Rotation Constraints

These parameters control how the guide rotates during seating.

### `rotation.stiffness`

**Type:** float
**Default:** 0.0
**Range:** 0 - 5000

Torsional spring stiffness that pulls the guide back toward its initial orientation (identity quaternion). Set to 0 for free rotation.

| Value | Effect |
|-------|--------|
| 0 | Free rotation - guide rotates to minimize spring potential |
| 100-500 | Moderate constraint - allows some rotation adjustment |
| 1000-5000 | Strong constraint - nearly fixed orientation |

Use non-zero values when you want the guide to translate into position without significant rotation.

### `rotation.max_angular_velocity`

**Type:** float
**Default:** 5.0
**Range:** 0 - 50 (rad/s)

Maximum angular velocity magnitude. Limits how fast the guide can spin during transients.

| Value | Effect |
|-------|--------|
| 0 | Unlimited - guide may spin rapidly during oscillation |
| 5.0 | Default - prevents wild spinning while allowing gradual rotation |
| 20.0 | Faster rotation allowed |

This is a velocity clamp, not a constraint. It prevents numerical instability from rapid rotation without affecting the final settled orientation.

---

## Collision Detection

Multi-phase collision provides efficient and accurate seating by progressively refining the collision mesh.

### `collision.enabled`

**Type:** boolean
**Default:** true

Set to `false` to disable collision entirely (springs only). Useful for debugging spring behavior.

### `collision.multi_phase.enabled`

**Type:** boolean
**Default:** true

Enable multi-phase collision with coarse-to-fine mesh progression.

**Phase progression:**
1. **Phase 1 (Springs Only):** No collision detection. Guide moves quickly toward bone under spring forces alone. Fast but may interpenetrate.
2. **Phase 2 (Coarse Collision):** Activate decimated collision mesh (500 faces). Prevents gross interpenetration with low computational cost.
3. **Phase 3 (Fine Collision):** Activate finer collision mesh (2000 faces). Accurate final positioning.

### `collision.multi_phase.coarse_faces`

**Type:** integer
**Default:** 500
**Range:** 200 - 1000

Number of faces in the coarse collision mesh. Created by quadric decimation of the original mesh.

### `collision.multi_phase.fine_faces`

**Type:** integer
**Default:** 2000
**Range:** 1000 - 5000

Number of faces in the fine collision mesh. Higher values give more accurate seating but slower simulation.

| Value | Accuracy | Speed |
|-------|----------|-------|
| 1000 | Moderate | Fast (~30ms/step in Phase 3) |
| 2000 | Good | Default (~100ms/step in Phase 3) |
| 5000 | High | Slow (~300ms/step in Phase 3) |

### `collision.distances.alarm_distance`

**Type:** float
**Default:** 5.0 mm

Distance at which collision detection begins checking for potential contacts. Larger values detect collisions earlier but increase computation.

### `collision.distances.contact_distance`

**Type:** float
**Default:** 2.0 mm

Minimum separation maintained between colliding surfaces. This effectively creates a "cushion" between the guide and bone.

**Warning:** Very small values (<0.5mm) may cause instability. Very large values (>5mm) prevent proper seating.

---

## Convergence Criteria

These parameters define when the simulation considers the guide to be "seated".

### `convergence.thresholds.phase1/2/3`

**Type:** float
**Default:** 0.2 / 0.05 / 0.01 mm

Position change threshold for each phase. The simulation exits a phase when position changes less than this amount over 100 simulation steps.

| Phase | Threshold | Meaning |
|-------|-----------|---------|
| Phase 1 | 0.2 mm | Coarse positioning complete |
| Phase 2 | 0.05 mm | Stable under coarse collision |
| Phase 3 | 0.01 mm | Final position reached |

### `convergence.settling_steps`

**Type:** integer
**Default:** 50

Number of steps to wait after a phase transition before checking convergence. Allows transients from enabling collision to dissipate.

---

## Simulation Parameters

### `simulation.dt`

**Type:** float
**Default:** 0.001 (1ms)

Simulation timestep in seconds. Smaller values increase accuracy but slow down the simulation.

### `simulation.max_time`

**Type:** float
**Default:** 10.0 seconds

Maximum simulation time. The simulation stops if convergence is not reached by this time.

### `simulation.gravity`

**Type:** [float, float, float]
**Default:** [0.0, 0.0, -9810.0]

Gravity vector in mm/s². Default is Earth gravity in Z-down orientation.

Set to `[0, 0, 0]` to disable gravity and test pure spring-based seating.

### `simulation.guide_mass_kg`

**Type:** float
**Default:** 0.05 (50 grams)

Mass of the guide in kilograms. Affects dynamic response and damping behavior.

---

## Initial Offset

### `initial_offset.offset_mm`

**Type:** [float, float, float]
**Default:** [0.0, 5.0, 0.0]

Offset vector to apply to the guide's initial position before simulation. Use this to:

1. Prevent initial interpenetration when starting at the designed pose
2. Test seating from different distances
3. Validate that the guide reaches the same final position from different starts

Example values:
- `[0, 0, 0]`: Start at designed position
- `[0, 5, 0]`: Start 5mm away in Y direction
- `[0, 0, 10]`: Start 10mm away in Z direction

---

## Example Configurations

### Tight Fit (Accurate but Slow)

```yaml
mating_points:
  distance_threshold_mm: 0.5
  n_points: 200

collision:
  multi_phase:
    fine_faces: 3000

convergence:
  thresholds:
    phase3: 0.005
```

### Fast Iteration (Quick but Less Accurate)

```yaml
mating_points:
  distance_threshold_mm: 1.5
  n_points: 50

collision:
  multi_phase:
    coarse_faces: 300
    fine_faces: 1000

simulation:
  max_time: 5.0
```

### Debug Mode (Springs Only)

```yaml
collision:
  enabled: false

simulation:
  gravity: [0, 0, 0]
```
