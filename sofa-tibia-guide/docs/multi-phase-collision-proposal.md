# Proposal: Multi-Phase Progressive Collision for Guide Seating

**Date**: 2026-01-28
**Status**: Proposal (Validated via Hypothesis Testing)
**Depends on**: Phase 3b global optimum (confirmed), mating point normal filter (done)

## Problem

Collision is disabled (`--no-collision`) because SOFA's `BVHNarrowPhase` +
`GenericConstraintSolver` pipeline is too expensive for full-resolution meshes
(56K + 54K faces). Without collision, the spring equilibrium sits ~2.78mm inside
the bone. Collision provides the restoring force to stop interpenetration and is
required for slop measurement.

Initial attempt at decimation (this session) showed:

| Collision faces | Avg step time | Behavior |
|-----------------|---------------|----------|
| 5000 | ~6s | No hang, but too slow (3000 steps = 5 hours) |
| 1000 | ~1.7s | Too slow (3000 steps = 85 min) |
| 500 | ~370ms | Feasible speed but guide oscillates -- coarse collision surface causes bouncing |

The cost comes from three compounding factors:
1. `FreeMotionAnimationLoop` does free-motion solve, collision, then constraint solve -- three passes per step
2. `GenericConstraintSolver` with 1000 max iterations runs Gauss-Seidel on all contacts
3. Triangle + Line + Point collision models on both meshes create large contact pair counts

## Hypothesis

The collision pipeline cost is dominated by the constraint solver and the number
of contact pairs, both of which scale with mesh complexity. However, collision is
only needed when the guide is close to the bone surface. By running the simulation
in phases with progressively increasing collision resolution, we can:

1. Converge to the approximate seating position quickly (no collision overhead)
2. Enable coarse collision to prevent gross interpenetration (low overhead)
3. Refine with finer collision for accurate final seating (higher overhead, but fewer steps needed)

Additionally, SOFA's `GenericConstraintSolver` parameters (`maxIterations`,
`tolerance`) and intersection parameters (`alarmDistance`, `contactDistance`) can
be tuned per-phase to minimize cost when precision is not yet needed.

### Hypothesis Validation (2026-01-28)

Three hypotheses were tested via web research against SOFA source code,
documentation, forums, and academic papers. See `hypothesis-testing-results.md`
for full details.

| Hypothesis | Result |
|------------|--------|
| H1: `setActive()` reliably toggles collision mid-simulation | **SUPPORTED** - verified in SOFA source code |
| H2: Solver cost scales with contact pairs; decimation helps | **PARTIALLY SUPPORTED** - cost scales with *active contacts*, not mesh faces |
| H3: Collision activation on interpenetrating bodies is stable | **CONDITIONALLY SUPPORTED** - manageable with settling periods and damping |

**Key findings incorporated into this proposal:**
- Triangle+Line+Point creates ~3x collision elements vs Triangle-only
- Constraint solver is 54% of step time in SOFA benchmarks
- `currentNumConstraints` should be monitored for adaptive phase transitions
- Settling periods after phase transitions prevent explosive impulses
- Parallelism options (`multithreading`, `UnbuiltGaussSeidel`) reduce solver cost

## Approach: Pre-Loaded Dual Nodes with `setActive()` Toggling

SOFA supports runtime activation/deactivation of scene graph nodes:

- `node.setActive(False)` -- deactivates entire subtree; all visitors skip it
- `node.setActive(True)` -- reactivates
- All `Data` fields are writable at runtime (alarm/contact distances, solver iterations, contact stiffness)
- The collision pipeline runs but finds zero contacts when all collision nodes are deactivated (near-zero overhead)

**Source code verification** (from `Node.cpp`):
```cpp
void Node::executeVisitor(Visitor* action, bool precomputedOrder) {
    if (!this->isActive()) return;  // Early return - no traversal
}
```

**Recommended activation pattern** (using `DeactivationVisitor` for robustness):
```python
from sofa.simulation import DeactivationVisitor

# To activate
visitor = DeactivationVisitor(params, True)
node.executeVisitor(visitor)
node.setActive(True)

# To deactivate
visitor = DeactivationVisitor(params, False)
node.executeVisitor(visitor)
node.setActive(False)
```

### Scene Graph Structure

```
root
  FreeMotionAnimationLoop (parallelCollisionDetectionAndFreeMotion=True)
  GenericConstraintSolver (multithreading=True, resolutionMethod='UnbuiltGaussSeidel')
  CollisionPipeline + BruteForceBroadPhase + BVHNarrowPhase
  LocalMinDistance (alarmDistance/contactDistance tuned per phase)
  CollisionResponse (FrictionContactConstraint)

  Tibia (full mesh for topology/visual)
    MeshSTLLoader -> MeshTopology -> MechanicalObject
    CollisionCoarse (500 faces, Triangle-only)   [starts DEACTIVATED]
    CollisionFine (2000 faces, Triangle+Line+Point) [starts DEACTIVATED]

  Targets (fixed mating target points)

  Guide (rigid body)
    EulerImplicitSolver (rayleighMass=0.1) + CGLinearSolver
    MechanicalObject (Rigid3d) + UniformMass
    UncoupledConstraintCorrection
    CollisionCoarse (500 faces, Triangle-only)   [starts DEACTIVATED]
    CollisionFine (2000 faces, Triangle+Line+Point) [starts DEACTIVATED]
    MatingPoints (spring attachment points)

  StiffSpringForceField or AttractiveForceController
```

All collision nodes are created and initialized at startup (so SOFA builds BVH
trees, allocates memory, etc.), then immediately deactivated. This avoids runtime
`init()` calls which are less reliable.

**Key changes from initial proposal:**
- `parallelCollisionDetectionAndFreeMotion=True` for concurrent execution
- `multithreading=True` and `resolutionMethod='UnbuiltGaussSeidel'` for faster solving
- `rayleighMass=0.1` on EulerImplicitSolver for damping during phase transitions
- **Triangle-only** collision models for coarse phase (3x fewer collision elements)

### Phase Definitions

#### Phase 1: Springs Only (no collision)

- **All collision nodes deactivated** -- pipeline finds nothing, solver idles
- `GenericConstraintSolver.maxIterations = 10` (fast no-op)
- `LocalMinDistance.alarmDistance = 10.0` (irrelevant but wide)
- Springs pull guide toward equilibrium position
- **Exit condition**: Position change < 0.2mm over 100 steps (conservative exit before deep penetration)
- **Expected performance**: ~1-5ms/step (same as `DefaultAnimationLoop`)
- **Expected steps**: ~1000-1500 (based on Phase 3b results)

#### Phase 2: Coarse Collision (500 faces, Triangle-only)

- **Activate coarse collision nodes** for both tibia and guide
- **Triangle-only collision models** (no Line/Point -- 3x fewer collision elements)
- `GenericConstraintSolver.maxIterations = 100` (reduced; monitor `currentIterations`)
- `LocalMinDistance.alarmDistance = 6.0, contactDistance = 3.0` (larger buffer zone)
- `rayleighMass = 0.1` (damping active)
- **Settling period**: 50 steps after activation before evaluating convergence
- Springs continue pulling; collision prevents gross interpenetration
- **Exit condition**: Position change < 0.05mm over 100 steps AND `currentNumConstraints` < 200
- **Expected performance**: ~50-200ms/step (fewer collision elements than original test)
- **Expected steps**: ~200-400 (guide is already near equilibrium)

#### Phase 3: Fine Collision (2000 faces, Triangle+Line+Point)

- **Deactivate coarse, activate fine collision nodes**
- **Triangle + Line + Point collision models** (full accuracy for final positioning)
- `GenericConstraintSolver.maxIterations = 500` (reduced from 1000; monitor convergence)
- `LocalMinDistance.alarmDistance = 3.0, contactDistance = 1.0`
- **Settling period**: 50 steps after activation before evaluating convergence
- Final refinement of seating position
- **Exit condition**: Position change < 0.01mm over 100 steps (converged)
- **Expected performance**: ~300ms-1.5s/step
- **Expected steps**: ~100-300 (fine adjustment only)

### Estimated Total Wall Time

| Phase | Steps | Per-step | Wall time |
|-------|-------|----------|-----------|
| 1 | ~1200 | ~3ms | ~4s |
| 2 (settling) | 50 | ~150ms | ~8s |
| 2 (convergence) | ~250 | ~150ms | ~38s |
| 3 (settling) | 50 | ~800ms | ~40s |
| 3 (convergence) | ~150 | ~800ms | ~120s |
| **Total** | ~1700 | | **~3.5 min** |

This is a rough estimate. Key improvements over initial proposal:
- Triangle-only Phase 2 reduces collision element count by ~3x
- Parallelism options reduce solver overhead
- Lower `maxIterations` based on expected quick convergence near equilibrium

## Implementation Plan

### 1. `scenes/attractive_seating.py` -- `create_attractive_scene()`

Add parameters for coarse and fine collision mesh paths:

```python
def create_attractive_scene(
    ...,
    tibia_collision_coarse_path: str = None,
    tibia_collision_fine_path: str = None,
    guide_collision_coarse_path: str = None,
    guide_collision_fine_path: str = None,
):
```

Create dual collision child nodes for tibia and guide:
- **Coarse nodes**: Triangle-only collision models
- **Fine nodes**: Triangle + Line + Point collision models
- All start deactivated

Return references to the collision nodes and key components (solver, intersection)
for the phase controller to manipulate.

### 2. `scenes/attractive_seating.py` -- Phase Controller

New `MultiPhaseCollisionController` (extends existing `AttractiveForceController`
or runs alongside it):

```python
class MultiPhaseCollisionController(Sofa.Core.Controller):
    def __init__(self, *args, **kwargs):
        # Store references to collision nodes, solver, intersection
        self.phase = 1
        self.step_count = 0
        self.settling_steps = 0
        self.position_history = []

    def onAnimateEndEvent(self, event):
        self.step_count += 1
        current_pos = self.get_guide_position()
        self.position_history.append(current_pos)

        if self.settling_steps > 0:
            self.settling_steps -= 1
            return  # Still in settling period

        # Check phase transition conditions
        if self.phase == 1:
            if self.position_change_over_n_steps(100) < 0.2:
                self.transition_to_phase_2()
        elif self.phase == 2:
            constraints = self.solver.currentNumConstraints.value
            if self.position_change_over_n_steps(100) < 0.05 and constraints < 200:
                self.transition_to_phase_3()
        elif self.phase == 3:
            if self.position_change_over_n_steps(100) < 0.01:
                self.mark_converged()

    def transition_to_phase_2(self):
        self.activate_coarse_collision()
        self.solver.maxIterations.value = 100
        self.intersection.contactDistance.value = 3.0
        self.settling_steps = 50
        self.phase = 2

    def transition_to_phase_3(self):
        self.deactivate_coarse_collision()
        self.activate_fine_collision()
        self.solver.maxIterations.value = 500
        self.intersection.contactDistance.value = 1.0
        self.settling_steps = 50
        self.phase = 3
```

Key behaviors:
- Monitors position change per 100 steps
- **Monitors `currentNumConstraints`** for Phase 2→3 transition
- **50-step settling period** after each phase transition
- Activates/deactivates collision nodes via `setActive()` with `DeactivationVisitor`
- Adjusts solver and intersection parameters via `.value =`
- Logs phase transitions, per-phase step counts, and constraint counts

### 3. `scenes/attractive_seating.py` -- `run_attractive_seating()`

Add `collision_faces_coarse` and `collision_faces_fine` parameters. Generate three
decimation levels:

- Coarse: `simplify_quadric_decimation(collision_faces_coarse)` (default: 500)
- Fine: `simplify_quadric_decimation(collision_faces_fine)` (default: 2000)
- Full: original mesh (for mating point identification and visual)

Export all to temp files and pass to `create_attractive_scene()`.

### 4. `experiments/test_attractive_seating.py`

Add CLI arguments:
- `--collision-faces-coarse` (default: 500)
- `--collision-faces-fine` (default: 2000)
- Replace `--collision-faces` with these two

### 5. Validation

Run V1-V6 with multi-phase collision enabled. Success criteria:
- No hang or timeout
- Total wall time < 10 minutes per test
- Final position within 1.0mm of known-good pose (relaxed from 0.5mm initially,
  since collision will change the equilibrium from the spring-only result)

## Risks and Mitigations

### Risk 1: Phase transition causes instability

When collision nodes activate, the guide may already be interpenetrating the bone
(springs pulled it in during Phase 1). The collision response may apply a large
impulse, causing oscillation or divergence.

**Mitigation** (validated by research):
- Phase 1 exit threshold is conservative (0.2mm change, exiting before deep penetration)
- **50-step settling period** after each phase transition allows gradual constraint resolution
- **Rayleigh damping** (`rayleighMass=0.1`) dampens oscillation during settling
- Large `contactDistance=3.0` in Phase 2 creates buffer zone before true surface contact
- Constraint-based contact (FrictionContactConstraint) is more stable than penalty for pre-penetration

### Risk 2: Coarse collision oscillation (observed in initial tests)

The 500-face mesh caused oscillation in the single-phase test.

**Mitigation** (validated by research):
- In the multi-phase approach, the guide enters Phase 2 already near its equilibrium
- **Triangle-only collision** for Phase 2 reduces contact pair count by ~3x
- Settling period absorbs initial impulse before convergence evaluation
- The oscillation in the single-phase test occurred because the guide was far from
  equilibrium and accelerating when it hit the coarse collision surface

### Risk 3: Fine collision still too slow

If 2000+ faces produce step times > 1.5s, Phase 3 could still take 5+ minutes.

**Mitigation** (validated by research):
- **Monitor `currentIterations`** -- if solver consistently converges in < 100 iterations, reduce `maxIterations`
- **`UnbuiltGaussSeidel` resolution method** avoids matrix assembly overhead
- **`multithreading=True`** parallelizes compliance matrix building
- **`parallelCollisionDetectionAndFreeMotion=True`** runs collision and free motion concurrently
- If still too slow, consider 1500-face fine mesh instead of 2000

### Risk 4: `setActive()` toggle doesn't properly re-engage collision

~~SOFA's runtime node activation is documented but not heavily tested in this specific pattern.~~

**Resolved** (validated by source code analysis):
- `Node.executeVisitor()` explicitly checks `isActive()` and returns early if false
- `CollisionPipeline.cpp` explicitly skips inactive collision models
- BVH trees are rebuilt automatically each step via `computeBoundingTree()` -- no manual reinit needed
- No bug reports found for `setActive()` + collision
- Use `DeactivationVisitor` pattern for robustness when toggling entire subtrees

## Alternative Considered: Python-Side SDF Penalty

Bypass SOFA's collision pipeline entirely by computing signed-distance-field-based
penalty forces in the Python controller. This was the recommendation from initial
research but rejected because:

1. The user has already spent weeks on Python-only SDF approaches without success
2. SOFA's contact mechanics are needed for slop measurement (how much the guide
   can move once seated)
3. `FreeMotionAnimationLoop` + `GenericConstraintSolver` provides physically
   accurate Lagrangian constraint-based contact, which is important for measuring
   fit quality -- not just preventing interpenetration

## Alternative Considered: Penalty Contact (`PenalityContactForceField`)

Replace the constraint pipeline with penalty-based contact (no
`FreeMotionAnimationLoop`, no `GenericConstraintSolver`). This would be faster
but:

1. Penalty contact is less stable (requires careful stiffness tuning)
2. Previous attempt (documented in work-in-progress.md "What Didn't Work" #4)
   had the guide fall through
3. Does not provide the contact force information needed for slop measurement
4. Research confirms: "subject to instability if not properly tuned" for pre-penetration

The multi-phase approach preserves the constraint-based pipeline's accuracy while
managing its cost through progressive activation.

## Success Criteria

1. Single test completes without hang or timeout (< 5 min wall time)
2. V1-V6 validation: final position within 1.0mm of known-good pose
3. Collision prevents interpenetration (guide does not pass through bone)
4. Foundation for slop measurement (constraint forces available at equilibrium)
5. Phase transitions are smooth (no explosive impulses or divergence)
