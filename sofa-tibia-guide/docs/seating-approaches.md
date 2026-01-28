# Guide Seating Approaches

## Problem Statement

The goal is to find where a surgical cutting guide **best fits** onto a tibia bone model. This involves:

1. **Global Optimum**: Find the pose where the guide's mating surface aligns best with the bone
2. **Local Optima**: Identify other stable positions where the guide might "feel" seated but isn't optimal
3. **Slop Measurement**: After seating, measure how much the guide can move while still feeling snug

The current gravity-based approach fails because:
- Simplified collision meshes (convex hull/OBB) don't capture mating surfaces
- Gravity only pushes in one direction, doesn't find best fit
- The guide slides off rather than settling into the bone

---

## Option 1: ICP Surface Registration

**Approach**: Use Iterative Closest Point algorithm to geometrically align mating surfaces.

**Pros**:
- Well-established algorithm with known convergence properties
- Fast, no physics simulation needed
- Directly optimizes surface alignment

**Cons**:
- Requires identifying/extracting mating surfaces
- Doesn't naturally handle collision (guide could penetrate bone)
- Finding multiple optima requires explicit multi-start
- Doesn't give physical insight into stability/slop

**Implementation**:
- Extract mating surfaces from both meshes
- Run ICP from multiple initial poses
- Post-process to ensure no interpenetration
- Use registration error as quality metric

---

## Option 2: Attractive Contact Physics (SELECTED FOR POC)

**Approach**: Physics simulation with attractive forces between mating surfaces plus collision/repulsion.

**Pros**:
- Naturally handles collision constraints
- Energy landscape gives insight into stability
- Perturbation analysis directly measures slop
- More physically intuitive results

**Cons**:
- More computationally expensive
- Requires tuning force parameters
- May need many simulation runs for global search

**Implementation**: See detailed design below.

---

## Option 3: Constrained Optimization

**Approach**: Formulate as optimization problem minimizing surface distance subject to non-penetration.

**Pros**:
- Can use powerful optimization algorithms (BFGS, etc.)
- Explicit constraint handling
- Can compute Hessian for stability analysis

**Cons**:
- Constraint handling is complex for mesh collision
- May struggle with non-convex energy landscape
- Less intuitive than physics-based approach

---

## Option 2 Detailed Design: Attractive Contact Physics

### Core Concept

Create a force field that:
1. **Attracts** the guide's mating surface toward the bone at medium range
2. **Repels** when surfaces get too close (prevents interpenetration)
3. **Stabilizes** at the equilibrium distance (contact)

This mimics how a physical guide would be pressed onto a bone - there's a "snap" when surfaces align.

### Mating Surface Identification

The guide has an inner (concave) surface designed to mate with the bone. We identify this by:

1. **Normal-based**: Mating surface normals point "inward" (toward bone centroid when seated)
2. **Proximity-based**: Points on guide that are closest to bone in designed configuration
3. **Curvature-based**: Concave regions of the guide mesh

For POC, use proximity-based: sample guide surface, identify points that face the bone.

### Force Model

Use a **Lennard-Jones-like potential** between mating surfaces:

```
U(r) = ε * [(σ/r)^12 - 2*(σ/r)^6]

Where:
- r = distance between surface points
- σ = equilibrium distance (contact distance)
- ε = well depth (attraction strength)
```

This gives:
- Strong repulsion at r < σ (prevents penetration)
- Attraction at r > σ (pulls surfaces together)
- Equilibrium at r = σ (stable contact)

### Algorithm for Finding Optima

```
1. MULTI-START SEARCH:
   - Generate N initial poses covering configuration space
   - For each initial pose:
     a. Run physics simulation with attractive forces
     b. Detect convergence (velocity → 0, energy stable)
     c. Record final pose and energy

2. CLUSTER RESULTS:
   - Group final poses by similarity (position/orientation threshold)
   - Each cluster represents a local optimum
   - Lowest energy cluster is global optimum

3. RANK OPTIMA:
   - Sort by energy (lower = better fit)
   - Report top K distinct optima
```

### Slop Measurement Protocol

Once guide is seated at an optimum:

```
1. PERTURBATION TEST:
   For each DOF (tx, ty, tz, rx, ry, rz):
     a. Apply small force/torque impulse
     b. Measure maximum displacement
     c. Measure return time to equilibrium
     d. Record stiffness = force / displacement

2. SLOP METRIC:
   - Max displacement under standard perturbation
   - Eigenvalues of stiffness matrix (lower = more slop)
   - Directions of maximum slop (eigenvectors)

3. INTERPRETATION:
   - High stiffness in all directions = snug fit
   - Low stiffness in some direction = slop in that direction
   - Very low stiffness = unstable seating
```

### Implementation Phases

**Phase 1: Basic Attractive Force POC**
- Implement simple point-to-surface attractive force
- Test with existing meshes
- Verify guide moves toward bone and stabilizes

**Phase 2: Multi-Start Search**
- Generate diverse initial poses
- Run multiple simulations
- Cluster and rank results

**Phase 3: Slop Measurement**
- Implement perturbation protocol
- Compute stiffness metrics
- Visualize slop directions

---

## Key Parameters to Tune

| Parameter | Description | Initial Value |
|-----------|-------------|---------------|
| σ (sigma) | Equilibrium distance | 0.5 mm |
| ε (epsilon) | Attraction strength | TBD |
| Sample density | Points per surface area | 1 per 10 mm² |
| Convergence threshold | Velocity magnitude | 0.01 mm/s |
| Perturbation force | Test impulse magnitude | 1 N |
| N_starts | Number of initial poses | 50-100 |

---

## Success Criteria

1. **Global optimum found**: Guide visually seats correctly on bone
2. **Local optima detected**: Multiple distinct stable poses identified
3. **Slop quantified**: Numerical measure of fit quality
4. **Reproducible**: Same results from different random seeds
