# SOFA Collision Test Results Tracking

## Test Configuration

**SOFA Version**: v24.06.00
**Collision Pipeline**:
- `FreeMotionAnimationLoop`
- `BruteForceBroadPhase` + `BVHNarrowPhase`
- `LocalMinDistance` (default: alarm=5, contact=2)
- `LCPConstraintSolver` (maxIt=1000, tolerance=0.001)
- `FrictionContactConstraint` response

**Mesh Files**:
- Tibia: `/tmp/repaired/CASE4_CroppedTibiaBone-repaired.stl` (56,458 faces original)
- Guide: `/tmp/repaired/CASE4_ResectThruTibiaGuide-repaired.stl` (53,602 faces original)

---

## Summary Table

| # | Static Mesh | Dynamic Mesh | Positioning | Result | Final Z | Notes |
|---|-------------|--------------|-------------|--------|---------|-------|
| 1 | Simple floor (12 faces) | Sphere (1280 faces) | Centered | PASS | 2.0 | Baseline working |
| 2 | Simple floor (12 faces) | Box (12 faces) | Centered | PASS | 2.0 | Baseline working |
| 3 | Tibia hull (9060 faces) | Sphere (1280 faces) | Centered | PASS | 1.9 | Tibia hull works |
| 4 | Tibia hull | Small box (10x10x10) | Centered | PASS | 1.8 | Works |
| 5 | Tibia hull | Large box (30x30x30) | Centered | PASS | 1.5 | Works |
| 6 | Tibia hull | Elongated box (50x43x63) | Centered | PASS | 1.8 | Similar to guide OBB |
| 7 | Guide hull (1764 faces) | Sphere | Centered | **FAIL** | -807 | Guide hull issue |
| 8 | Guide OBB (12 faces) | Sphere | Centered | PASS | -0.16 | OBB workaround works |
| 9 | Tibia hull | Guide OBB | Combined centroid | **FAIL** | -685 | Position issue |
| 10 | Tibia hull | Guide OBB | At (0, 3) | PASS | 1.6 | Position matters! |
| 11 | Tibia hull | Guide OBB | At (15, 0) | PASS | 1.0 | Outer edge works |
| 12 | Tibia hull | Guide OBB | At (10, 0) | **FAIL** | -825 | Central area fails |

---

## Guide Hull Analysis

| Property | Tibia Hull | Guide Hull | Working OBB |
|----------|------------|------------|-------------|
| Vertices | 4532 | 884 | 8 |
| Faces | 9060 | 1764 | 12 |
| Is watertight | True | True | True |
| Is convex | True | True | True |
| Min edge (mm) | 0.001754 | 0.005099 | 39.8 |
| Short edges (<0.1mm) | 171 (1.3%) | 602 (22.8%) | 0 |
| High aspect faces (>10) | 843 (9.3%) | 839 (47.6%) | 0 |

---

## Position Grid Test: Guide OBB on Tibia Hull

Tested guide OBB falling at different XY positions on normalized tibia hull.

### Pass/Fail Grid

|     | Y=-6 | Y=-3 | Y=0 | Y=3 | Y=6 |
|-----|------|------|-----|-----|-----|
| X=-15 | PASS | PASS | FAIL | FAIL | PASS |
| X=-10 | PASS | PASS | FAIL | FAIL | PASS |
| X=-5 | FAIL | PASS | FAIL | PASS | PASS |
| X=0 | FAIL | PASS | FAIL | PASS | FAIL |
| X=5 | PASS | FAIL | FAIL | FAIL | PASS |
| X=10 | FAIL | FAIL | FAIL | FAIL | FAIL |
| X=15 | PASS | PASS | PASS | PASS | PASS |

### Final Z Values (mm)

|     | Y=-6 | Y=-3 | Y=0 | Y=3 | Y=6 |
|-----|------|------|-----|-----|-----|
| X=-15 | -2.7 | 0.4 | -230.6 | -198.0 | -5.0 |
| X=-10 | 0.3 | 0.2 | -13.9 | -845.4 | -9.1 |
| X=-5 | -74.6 | -5.4 | -772.5 | 1.2 | -0.6 |
| X=0 | -10.9 | -8.4 | -660.3 | 1.6 | -101.4 |
| X=5 | 1.6 | -18.6 | -79.6 | -820.8 | -5.8 |
| X=10 | -866.6 | -856.1 | -825.2 | -816.4 | -879.3 |
| X=15 | 1.7 | 1.5 | 1.0 | 1.3 | 1.3 |

**Tibia hull XY bounds**: X=[-30.2, 28.9], Y=[-13.0, 8.1]

### Key Observations

1. **X=15 always works** - At the positive X edge of the tibia hull
2. **X=10 always fails** - In the central-positive X region
3. **Y=0 mostly fails** - Center line is problematic
4. **Edge positions work better** - Corners and edges of tibia hull

---

## Collision Parameter Tests (Guide Hull)

| Alarm | Contact | Result | Final Z |
|-------|---------|--------|---------|
| 5 | 2 | FAIL | -807 |
| 10 | 5 | FAIL | -716 |
| 20 | 10 | FAIL | -375 |
| **50** | **25** | **PASS** | 4703 (bounced) |
| 100 | 50 | PASS | 15243 (bounced way up) |

With very large alarm/contact distances, the guide hull collision works but response is exaggerated.

---

## Working Configurations

### 1. Use OBB for Guide Collision
```python
guide_obb = guide_mesh.bounding_box_oriented.to_mesh()
```

### 2. Position Guide at Tibia Hull Edges
Working positions on tibia hull:
- X=15 (any Y)
- X=-15, Y=-6 or Y=-3 or Y=6
- Avoid X=10 and Y=0 center region

### 3. Increase Collision Parameters (if convex hull needed)
```python
root.addObject("LocalMinDistance", alarmDistance=50, contactDistance=25, angleCone=0.0)
```

---

## Test Scripts Created

| Script | Purpose |
|--------|---------|
| `analyze_hull_geometry.py` | Compare tibia/guide hull geometry |
| `check_normals.py` | Verify face normal orientation |
| `compare_hull_shapes.py` | Shape analysis (sphericity, inertia) |
| `test_tibia_vs_guide_static.py` | Side-by-side hull comparison |
| `test_collision_groups.py` | Test explicit collision groups |
| `test_inline_guide_hull.py` | Test without STL loading |
| `test_scipy_convex_hull.py` | Alternative hull computation |
| `test_guide_obb.py` | OBB collision test |
| `test_clean_hull.py` | Various hull cleaning methods |
| `test_centered_sphere.py` | Position alignment check |
| `test_collision_params.py` | Parameter sensitivity |
| `test_tibia_hull_guide_obb.py` | Tibia + Guide OBB |
| `test_tibia_hull_simple_box.py` | Tibia + simple objects |
| `test_tibia_guide_simple_positioning.py` | Position testing |
| `test_position_grid.py` | Grid position mapping |
