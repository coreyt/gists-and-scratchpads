# Hypothesis Testing: Multi-Phase Collision Proposal

**Date**: 2026-01-28

## Hypotheses Tested

| # | Hypothesis | Status |
|---|-----------|--------|
| H1 | `setActive()` can reliably toggle collision subtrees mid-simulation | **SUPPORTED** |
| H2 | Solver cost scales with contact pairs; 500-2000 faces yields sub-500ms steps | **PARTIALLY SUPPORTED** |
| H3 | Enabling collision on interpenetrating bodies won't cause explosive instability | **CONDITIONALLY SUPPORTED** |

---

## H1: Runtime Toggling Reliability

### Evidence Found

**STRONGLY SUPPORTED** - Source code verification confirms:

1. **Visitor traversal checks `isActive()` at each node** (Node.cpp):
   ```cpp
   void Node::executeVisitor(Visitor* action, bool precomputedOrder) {
       if (!this->isActive()) return;  // Early return - no traversal
   }
   ```

2. **Collision pipeline explicitly skips inactive models** (CollisionPipeline.cpp):
   ```cpp
   if (!(*it)->isActive()) continue;  // Skip inactive collision models
   ```

3. **BVH trees rebuild automatically each step** - no manual `reinit()` needed:
   ```cpp
   (*it)->computeBoundingTree(used_depth);  // Called every step for active models
   ```

4. **No bug reports found** for `setActive()` causing crashes/stale data with collision.

5. **Recommended pattern exists** using `DeactivationVisitor`:
   ```python
   visitor = DeactivationVisitor(params, True)
   node.executeVisitor(visitor)
   node.setActive(True)
   ```

### Alternatives Discovered

- **Per-model toggle**: `collision_model.active.value = False` (simpler for individual models)
- **Collision groups**: Assign same group to prevent collision between specific pairs
- **ContactManager rules**: Set response to `null` for specific collision pairs

### Impact on Proposal

**No changes needed.** The `setActive()` approach is well-supported. Consider using
`DeactivationVisitor` for robustness when toggling entire subtrees.

---

## H2: Solver Cost Scaling

### Evidence Found

**PARTIALLY SUPPORTED** - The picture is more nuanced:

1. **Constraint solver is the dominant bottleneck** - up to 54% of step time in SOFA examples.

2. **Complexity is superlinear** in contact count:
   - Per-iteration: O(k) for sparse, O(n²) worst case for dense/coupled
   - Total: O(iterations × per-iteration cost)
   - Compliance matrix W = H·A⁻¹·Hᵀ requires solve() call per constraint line

3. **Contact pair explosion with Triangle+Line+Point**:
   - N-face mesh generates ~3N collision elements (triangles + edges + vertices)
   - Two 1000-face meshes: up to 4.5M pairs tested in broad phase
   - Narrow phase filters to hundreds-thousands of actual contacts

4. **Empirical guidance** from forums:
   - ~100 contacts: 20-50ms solve time
   - ~500 contacts: 100-300ms solve time
   - ~1000+ contacts: may exceed real-time

5. **Key insight discovered**: Cost depends on **active contact count**, not mesh faces.
   Two 1000-face meshes barely touching = few constraints. Same meshes deeply
   interpenetrating = thousands of constraints.

### Optimizations Available

| Optimization | Expected Impact |
|-------------|-----------------|
| `UncoupledConstraintCorrection` | 2-5x faster (vs LinearSolver) |
| `resolutionMethod="UnbuiltGaussSeidel"` | Avoids matrix assembly overhead |
| `multithreading=True` | Parallel compliance building |
| `parallelCollisionDetectionAndFreeMotion=True` | Concurrent collision + free motion |
| Triangle-only collision (no Line/Point) | ~3x fewer collision elements |

### Impact on Proposal

**Modifications recommended:**

1. **Monitor `currentNumConstraints` to tune phases** - not just position change.
   Phase 2→3 transition should consider contact count.

2. **Use Triangle-only collision models** for coarse phase. Line+Point add
   ~2x collision elements with minimal accuracy benefit at 500 faces.

3. **Enable all parallelism options** in GenericConstraintSolver and FreeMotionAnimationLoop.

4. **Use `UncoupledConstraintCorrection`** for the rigid guide (already in proposal).

5. **Start with `maxIterations=200`** for Phase 2 (not 1000) - if converging in fewer,
   reduce further based on `currentIterations` monitoring.

---

## H3: Interpenetration Stability

### Evidence Found

**CONDITIONALLY SUPPORTED** - Manageable with proper setup:

1. **Divergence CAN occur** - users report "simulation does blow up eventually"
   when using constraint-based collision with improper tuning.

2. **Constraint solver uses iterative approach** - penetration is resolved over
   multiple iterations/steps, not instantaneously. This is inherently stabilizing.

3. **Critical parameters for stability**:
   - `contactDistance` > 0 creates "buffer zone" before true interpenetration
   - Higher `alarmDistance` detects collisions earlier
   - Lower contact stiffness = more gradual resolution
   - Smaller time step = more stable

4. **Baumgarte stabilization** (common in physics engines): Don't correct full
   penetration in one step - use factor of 0.1-0.2 to avoid overshoot.

5. **PrecomputedConstraintCorrection** reported as "much more stable" for heavy
   deformation scenarios, but rigid bodies can use `UncoupledConstraintCorrection`.

### Key Risk Factor

The **penalty method** (`PenaltyContactForceField`) is "subject to instability
if not properly tuned" - but the proposal uses **constraint-based contact**
(`FrictionContactConstraint`), which is more stable for pre-penetration.

### Mitigations from Research

| Risk | Mitigation |
|------|-----------|
| Explosive impulse | Use constraint-based (not penalty) response |
| Overshoot | Increase solver iterations, reduce time step |
| Phase 1→2 interpenetration | Exit Phase 1 before deep penetration (0.1mm threshold conservative) |
| Oscillation at coarse mesh | Increase `contactDistance` to create buffer |

### Impact on Proposal

**Modifications recommended:**

1. **Add explicit settling period after phase transitions**:
   - After activating collision, run 50-100 steps with increased damping
   - Monitor position change; proceed only when < 0.1mm/step

2. **Use larger `contactDistance` in Phase 2** (proposal says 2.0mm - keep this):
   - Creates buffer zone before true surface contact
   - Allows gradual constraint resolution

3. **Add Rayleigh damping to guide**:
   - Set `rayleighMass` in EulerImplicitSolver
   - Dampens oscillation during phase transitions

4. **Consider warm-start strategy**:
   - End Phase 1 with guide ~1-2mm above equilibrium (not fully converged)
   - Coarse collision catches it before deep penetration

---

## Findings Summary

### Confirmed Strengths of Proposal

1. **`setActive()` toggling is reliable** - verified in SOFA source code
2. **Constraint-based collision is more stable** than penalty for pre-penetration
3. **Iterative solver naturally prevents single-step explosion**
4. **Phase approach fundamentally sound** - bulk convergence without collision cost

### New Risks Discovered

1. **Contact pair count, not mesh faces, drives cost** - Phase 2 may have more
   contacts than expected if guide is near-equilibrium (close to surface)

2. **Triangle+Line+Point triples collision element count** - significant for
   coarse phase performance

3. **No native warm-start for constraints** - each phase starts cold

### Opportunities Discovered

1. **`UnbuiltGaussSeidel` resolution method** - matrix-free, potentially faster
2. **Parallel collision + free motion** - concurrent execution
3. **`currentNumConstraints` monitoring** - adaptive phase transition
4. **Triangle-only collision** - simpler, faster for coarse phase

---

## Proposed Changes to Multi-Phase Approach

### Change 1: Use Triangle-Only Collision for Coarse Phase

**Before**: Triangle + Line + Point on 500-face meshes
**After**: Triangle-only on 500-face meshes for Phase 2

**Rationale**: Line+Point add ~2x collision elements with minimal accuracy benefit
at coarse resolution. Save computation for fine phase.

### Change 2: Add Constraint Count Monitoring

**Before**: Phase transitions based only on position change threshold
**After**: Also monitor `currentNumConstraints` - if > 500, consider earlier
transition to fine phase (coarse mesh may be generating too many contacts)

**Rationale**: Cost scales with contact count, not mesh complexity.

### Change 3: Add Settling Period After Phase Transitions

**Before**: Immediate continuation after phase transition
**After**: 50-step settling period with increased Rayleigh damping after each
phase transition

**Rationale**: Research shows divergence risk when collision activates on
pre-existing penetration. Settling period allows gradual constraint resolution.

### Change 4: Enable All Parallelism

**Before**: Default settings
**After**:
```python
root.addObject('FreeMotionAnimationLoop',
    parallelCollisionDetectionAndFreeMotion=True)
root.addObject('GenericConstraintSolver',
    multithreading=True,
    resolutionMethod='UnbuiltGaussSeidel')
```

**Rationale**: Research shows 54% of step time in constraint solving. Parallelism
helps reduce this bottleneck.

### Change 5: Conservative Phase 1 Exit

**Before**: Exit Phase 1 when position change < 0.1mm over 100 steps
**After**: Exit Phase 1 when position change < 0.2mm over 100 steps (earlier exit)
AND distance to estimated surface > 2mm

**Rationale**: Better to exit Phase 1 with guide still 2-3mm from surface than
to let it penetrate. Coarse collision catches it safely.

### Change 6: Lower maxIterations for Phase 2

**Before**: `maxIterations = 200` for Phase 2
**After**: `maxIterations = 100` for Phase 2, monitor `currentIterations`

**Rationale**: Research shows guide near-equilibrium with few contacts should
converge quickly. Start low, increase if `currentIterations` consistently hits limit.

---

## Updated Phase Definitions

### Phase 1: Springs Only (unchanged from proposal)
- All collision nodes deactivated
- `GenericConstraintSolver.maxIterations = 10`
- Exit: position change < 0.2mm over 100 steps (more conservative)

### Phase 2: Coarse Collision (modified)
- **Triangle-only** collision models on 500-face meshes
- `GenericConstraintSolver.maxIterations = 100` (reduced from 200)
- `multithreading = True`, `resolutionMethod = 'UnbuiltGaussSeidel'`
- `LocalMinDistance.contactDistance = 3.0` (increased from 2.0 for buffer)
- Add Rayleigh damping: `rayleighMass = 0.1`
- **Settling period**: 50 steps after activation before evaluating convergence
- Exit: position change < 0.05mm over 100 steps AND `currentNumConstraints` < 200

### Phase 3: Fine Collision (modified)
- Triangle + Line + Point on 2000-face meshes (Line+Point now worth cost)
- `GenericConstraintSolver.maxIterations = 500` (reduced from 1000)
- `LocalMinDistance.contactDistance = 1.0`
- **Settling period**: 50 steps after activation
- Exit: position change < 0.01mm over 100 steps

---

## Next Steps

1. Update `docs/multi-phase-collision-proposal.md` with these changes
2. Implement Phase 1→2 transition in isolation as proof-of-concept
3. Verify `setActive()` + `DeactivationVisitor` pattern works in Python
4. Add `currentNumConstraints` monitoring to controller
5. Run V1 with modified approach, measure step times per phase
