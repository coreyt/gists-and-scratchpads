# Session Notes - 2026-01-27

## Overview

This session implemented the SOFA tibial cutting guide simulation project from the ground up, including project structure, documentation, tests, and core implementation. The main blocker encountered was getting SOFA's collision response to work in Python-created scenes.

## Work Completed

### 1. Project Structure Created

Complete project structure following the plan:

```
sofa-tibia-guide/
├── docs/
│   ├── user-needs.md
│   ├── requirements.md
│   ├── architecture.md
│   └── asset-pipeline.md
├── scenes/
│   ├── __init__.py
│   └── mvm_seating.py
├── experiments/
│   ├── __init__.py
│   └── run_disturbance.py
├── src/tibia_guide/
│   ├── __init__.py
│   ├── assets/
│   │   ├── __init__.py
│   │   ├── loader.py
│   │   └── transform.py
│   ├── metrics/
│   │   ├── __init__.py
│   │   ├── pose.py
│   │   └── contact.py
│   └── config.py
├── tests/
│   ├── __init__.py
│   ├── conftest.py
│   ├── unit/
│   │   ├── test_loader.py
│   │   ├── test_transform.py
│   │   └── test_pose.py
│   └── integration/
│       └── test_scene_creation.py
├── assets/README.md
├── config_case4.yaml
├── environment.yml
├── pyproject.toml
├── setup_sofa_env.sh
├── .gitignore
└── README.md
```

### 2. Core Implementation

#### Asset Loading (`src/tibia_guide/assets/loader.py`)
- `load_mesh()`: Load STL/OBJ/PLY files using trimesh
- `validate_mesh()`: Check watertight, degenerate faces
- Custom exceptions: `MeshLoadError`, `MeshValidationError`

#### Transforms (`src/tibia_guide/assets/transform.py`)
- `apply_transform()`: Apply 4x4 homogeneous transform to mesh
- `matrix_to_pose()`: Extract position + quaternion from 4x4 matrix
- `pose_to_matrix()`: Create 4x4 matrix from position + quaternion
- `quaternion_to_rotation_matrix()` / `rotation_matrix_to_quaternion()`
- `compose_transforms()`: Chain multiple transforms
- `invert_transform()`: Compute inverse transform

#### Pose Metrics (`src/tibia_guide/metrics/pose.py`)
- `Pose` dataclass with position [x,y,z] and quaternion [w,x,y,z]
- `compute_translation_error()`: Euclidean distance in mm
- `compute_rotation_error()`: Geodesic distance in degrees
- `compare_poses()`: Returns dict with both errors
- `poses_within_tolerance()`: Check if within specified tolerances

#### Configuration (`src/tibia_guide/config.py`)
- Dataclasses: `MeshConfig`, `PoseConfig`, `DisturbanceConfig`, `SolverConfig`, `OutputConfig`, `SimulationConfig`
- YAML load/save support

#### Scene Creation (`scenes/mvm_seating.py`)
- `create_scene()`: Create SOFA scene with tibia (fixed) and guide (dynamic)
- `extract_guide_pose()`: Get current guide pose from simulation
- `add_constant_force()`: Apply disturbance forces

#### Experiment Runner (`experiments/run_disturbance.py`)
- CLI interface with argparse
- Runs simulation and exports pose time series to CSV/JSON
- Optional comparison against reference pose

### 3. Test Suite

**60 unit tests** - all passing:
- `test_loader.py`: 14 tests for mesh loading and validation
- `test_transform.py`: 24 tests for coordinate transforms
- `test_pose.py`: 22 tests for pose comparison metrics

**12 integration tests** - require SOFA:
- Scene creation tests
- Simulation step tests
- Pose extraction tests

### 4. SOFA Installation

Downloaded and configured SOFA v24.06.00:
- Location: `~/sofa/SOFA_v24.06.00_Linux/`
- Environment script: `setup_sofa_env.sh`
- Sets `SOFA_ROOT`, `PYTHONPATH`, `LD_LIBRARY_PATH`
- Verified SOFA imports work: `import Sofa` succeeds

### 5. Mesh Preparation

**Original meshes** (from 6dof-mc project):
- `/home/coreyt/projects/6dof-mc/prod/data/CASE4/CASE4_CroppedTibiaBone.stl` - 61,263 faces
- `/home/coreyt/projects/6dof-mc/prod/data/CASE4/CASE4_ResectThruTibiaGuide.stl` - 53,602 faces

**Repaired meshes** (user provided):
- `/tmp/repaired/CASE4_CroppedTibiaBone-repaired.stl` - 61,271 faces, not watertight
- `/tmp/repaired/CASE4_ResectThruTibiaGuide-repaired.stl` - 53,602 faces, watertight

**Decimated collision meshes** (created this session):
- `/tmp/repaired/CASE4_CroppedTibiaBone-collision.stl` - 4,999 faces
- `/tmp/repaired/CASE4_ResectThruTibiaGuide-collision.stl` - 5,000 faces

**Key mesh info:**
- CASE4 meshes are pre-positioned at optimal seating (unique to this case)
- Tibia Z range: -155.0 to -94.5 mm
- Guide Z range: -153.1 to -96.2 mm (when at origin)
- When guide at Z=70, guide bottom at -83.1, tibia top at -94.5 → 11.4mm gap

---

## Collision Detection Attempts

### Attempt 1: Initial Scene Setup

**Configuration:**
```python
root.addObject("FreeMotionAnimationLoop")
root.addObject("GenericConstraintSolver", maxIterations=100, tolerance=1e-6)
root.addObject("CollisionPipeline")
root.addObject("BruteForceBroadPhase")
root.addObject("BVHNarrowPhase")
root.addObject("LocalMinDistance", alarmDistance=1.0, contactDistance=0.5)
root.addObject("CollisionResponse", response="FrictionContactConstraint")
```

**Collision models:**
- `TriangleCollisionModel` only
- No `moving`/`simulated` flags set

**Result:** Guide fell through tibia. Final Z after 0.5s: -1208mm

### Attempt 2: Larger Collision Distances

**Changes:**
- `alarmDistance=5.0, contactDistance=2.0`
- Changed `CollisionResponse` to `DefaultContactManager`

**Result:** Still fell through. Same behavior.

### Attempt 3: Added Line and Point Collision Models

**Changes:**
```python
tibia_collision.addObject("TriangleCollisionModel", moving=False, simulated=False)
tibia_collision.addObject("LineCollisionModel", moving=False, simulated=False)
tibia_collision.addObject("PointCollisionModel", moving=False, simulated=False)

guide_collision.addObject("TriangleCollisionModel", moving=True, simulated=True)
guide_collision.addObject("LineCollisionModel", moving=True, simulated=True)
guide_collision.addObject("PointCollisionModel", moving=True, simulated=True)
```

**Result:** Still fell through.

### Attempt 4: Full-Resolution Meshes (61k + 54k faces)

**Result:** Simulation timed out during collision detection (BVHNarrowPhase). Too many triangles for real-time collision.

### Attempt 5: Decimated Meshes (5k faces each)

Created decimated collision meshes using `trimesh.simplify_quadric_decimation(5000)`.

**Result:** Simulation ran faster but guide still fell through.

### Attempt 6: Correct Starting Position

Calculated that guide must start at Z > 58.6 to be above tibia. Changed to Z=70.

**Debug output confirmed:**
- Tibia Z range: -155.0 to -94.5
- Guide Z range: -83.1 to -26.2 (after +70 translation)
- Gap of 11.4mm between guide bottom and tibia top

**Result:** Guide still fell through. Some X/Y drift and rotation observed (suggesting collision detection fires) but no Z-stopping force.

### Attempt 7: NewProximityIntersection

**Changes:**
- Replaced `LocalMinDistance` with `NewProximityIntersection`

**Result:** Segmentation fault in `GenericConstraintSolver::buildSystem`

### Attempt 8: Penalty Contact (No Constraints)

**Changes:**
```python
root.addObject("DefaultAnimationLoop")  # Instead of FreeMotionAnimationLoop
root.addObject("MinProximityIntersection", alarmDistance=5.0, contactDistance=2.0)
root.addObject("DefaultContactManager", response="PenalityContactForceField")
```

Removed `UncoupledConstraintCorrection` from guide.

**Result:** Guide still fell through with X/Y drift and rotation.

### Attempt 9: Match Dental Surgery Example

Studied SOFA's working example: `dentalSurgery_05.scn`

**Key differences noted:**
- Uses `LCPConstraintSolver` instead of `GenericConstraintSolver`
- Uses `CollisionResponse` instead of `DefaultContactManager`
- Uses `contactStiffness` parameter on collision models
- Has `depth=6` on CollisionPipeline

**Changes:**
```python
root.addObject("CollisionPipeline", depth=6)
root.addObject("CollisionResponse", response="FrictionContactConstraint")
root.addObject("LocalMinDistance", alarmDistance=5.0, contactDistance=2.0, angleCone=0.0)
root.addObject("FreeMotionAnimationLoop")
root.addObject("LCPConstraintSolver", tolerance=0.001, maxIt=1000)

# Added contactStiffness=5 to all collision models
```

**Result:** Still fell through.

### Attempt 10: Simple Box Test (No Complex Meshes)

Created minimal test with two cubes defined by inline vertex positions.

```python
floor.addObject("MechanicalObject", template="Vec3d",
    position=[[-50,-50,-5], [50,-50,-5], ...])
floor.addObject("MeshTopology", hexas=[[0,1,3,2,4,5,7,6]])
floor.addObject("TriangleCollisionModel", ...)
```

**Result:** Box fell through floor. Final Z: -1188mm

### Attempt 11: Constraint-Based with Compliance

Added `defaultCompliance=1e-5` to `UncoupledConstraintCorrection`.

**Result:** Still fell through.

### Attempt 12: No Gravity Test

Disabled gravity, gave box initial velocity of -100 mm/s.

**Result:** Box moved at constant velocity, passed through floor. No collision response at all. Velocity remained ~-99 mm/s throughout.

### Attempt 13: STL-Loaded Simple Boxes

Created simple box STL files with trimesh and loaded via `MeshSTLLoader`.

**Result:** Simulation hung/timed out during initialization.

---

## Verification That SOFA Works

### Test 1: SOFA Import
```python
import Sofa
print('SOFA imported successfully')
```
**Result:** Success

### Test 2: Scene Creation
```python
root = Sofa.Core.Node("root")
print(f'Created node: {root.name.value}')
```
**Result:** Success

### Test 3: Load SOFA XML Example
```python
root = Sofa.Simulation.load('.../dentalSurgery_05.scn')
Sofa.Simulation.init(root)
for i in range(10):
    Sofa.Simulation.animate(root, 0.005)
```
**Result:** Success - example loads and runs

---

## Key Observations

1. **Collision Detection Fires**: The guide shows X/Y position drift and rotation changes during simulation, suggesting the collision pipeline detects something.

2. **No Normal Force**: Despite detection, no force is applied to stop penetration in the Z direction.

3. **XML vs Python**: SOFA's bundled XML examples work correctly. The issue is specific to Python-created scenes.

4. **All Response Types Fail**: Tried both penalty-based (`PenalityContactForceField`) and constraint-based (`FrictionContactConstraint`) - neither produces stopping forces.

5. **Mesh Complexity Not Root Cause**: Even 12-triangle cubes fail.

6. **RigidMapping Works**: The collision mesh positions are correctly transformed based on rigid body pose (verified by printing Z ranges after init).

---

## Possible Root Causes (Hypotheses)

1. **Scene Graph Structure**: The way we construct the scene in Python may differ subtly from how XML scenes are parsed.

2. **Component Order**: SOFA may be sensitive to the order in which components are added.

3. **Missing Component**: There may be a required component that XML scenes implicitly add.

4. **Python Binding Issue**: The Python bindings might not properly trigger collision response calculations.

5. **Mapping Issue**: The RigidMapping might not be updating collision positions at the right time in the simulation loop.

---

## Files Created This Session

### Core Project Files
- `README.md`
- `.gitignore`
- `environment.yml`
- `pyproject.toml`
- `setup_sofa_env.sh`
- `config_case4.yaml`

### Documentation
- `docs/user-needs.md`
- `docs/requirements.md`
- `docs/architecture.md`
- `docs/asset-pipeline.md`
- `assets/README.md`

### Source Code
- `src/tibia_guide/__init__.py`
- `src/tibia_guide/config.py`
- `src/tibia_guide/assets/__init__.py`
- `src/tibia_guide/assets/loader.py`
- `src/tibia_guide/assets/transform.py`
- `src/tibia_guide/metrics/__init__.py`
- `src/tibia_guide/metrics/pose.py`
- `src/tibia_guide/metrics/contact.py`
- `scenes/__init__.py`
- `scenes/mvm_seating.py`
- `experiments/__init__.py`
- `experiments/run_disturbance.py`

### Tests
- `tests/__init__.py`
- `tests/conftest.py`
- `tests/unit/__init__.py`
- `tests/unit/test_loader.py`
- `tests/unit/test_transform.py`
- `tests/unit/test_pose.py`
- `tests/integration/__init__.py`
- `tests/integration/test_scene_creation.py`

### Debug/Test Scripts
- `test_collision.py`
- `test_simple_collision.py`
- `test_box_collision.py`
- `test_constraint_collision.py`
- `test_no_gravity.py`
- `test_stl_collision.py`

---

## Continued Session - Collision Investigation

### Attempt 14: XML Scene File

Created `scenes/simple_collision.scn` XML file matching the Python scene structure.

**Result:** FAILED - Objects still fell through. This disproved the hypothesis that XML vs Python was the issue.

### Attempt 15: SOFA Bundled Example Analysis

Tested SOFA's bundled `AdvancedResponseConstraintBased.scn` example:
```python
root = Sofa.Simulation.load("AdvancedResponseConstraintBased.scn")
# Cube falls and stops at y=-0.24
```
**Result:** SUCCESS - The SOFA example works correctly.

**Key insight from example:** The example uses:
1. `template="Rigid3"` for MechanicalObject (7-DOF: position + quaternion)
2. Collision mesh in a **child node** with its own Vec3d MechanicalObject
3. `RigidMapping` connecting rigid body state to collision mesh vertices

### Attempt 16: Correct Rigid3 Scene Structure

Created `scenes/rigid_collision_test.scn` and `test_rigid_collision_python.py` following the correct structure.

**Working structure:**
```python
# Dynamic rigid body
box = root.addChild("Box")
box.addObject("EulerImplicitSolver", rayleighStiffness=0.1, rayleighMass=0.1)
box.addObject("CGLinearSolver", iterations=25, tolerance=1e-9, threshold=1e-9)
box.addObject("MechanicalObject", name="rigidDOF", template="Rigid3",
              position=[[0, 0, 20, 0, 0, 0, 1]])  # [x, y, z, qx, qy, qz, qw]
box.addObject("UniformMass", totalMass=0.1)
box.addObject("UncoupledConstraintCorrection")

# Collision mesh in child node
box_collision = box.addChild("Collision")
box_collision.addObject("MeshTopology", name="topology", position=[...], hexas=[...])
box_collision.addObject("MechanicalObject", name="collisionState")
box_collision.addObject("TriangleCollisionModel")
box_collision.addObject("LineCollisionModel")
box_collision.addObject("PointCollisionModel")
box_collision.addObject("RigidMapping", input="@../rigidDOF", output="@collisionState")
```

**Result:** SUCCESS for simple boxes - Box stopped at z=6.0 (contact distance + half-height)

### Attempt 17: Medical Meshes with Correct Structure

Applied the correct Rigid3 structure to tibia/guide meshes.

**Result:** FAILED - Guide still fell through tibia despite correct structure.

### Attempt 18: STL Loading Investigation

Created tests comparing inline mesh definition vs STL-loaded meshes:
- `test_inline_vs_stl.py` - Floor(inline) + Box(STL) → SUCCESS
- `test_both_stl.py` - Floor(STL) + Box(STL) → SUCCESS
- `test_higher_poly.py` - 768-face meshes → SUCCESS
- `test_5k_faces.py` - 5120-face spheres → SUCCESS

**Conclusion:** STL loading and mesh complexity are NOT the issue.

### Attempt 19: Medical Mesh Analysis

Analyzed medical meshes with trimesh:

**Tibia (collision mesh):**
- 4999 faces, 2502 vertices
- **NOT watertight** (has holes)
- Euler number: 1 (should be 2 for closed surface)
- Min edge: 0.107mm, Max edge: 34.5mm
- Z range: -155.0 to -94.5

**Guide (collision mesh):**
- 5000 faces, 2496 vertices
- **Watertight**
- Euler number: -4 (genus 3 - has 3 holes/handles)
- Min edge: 0.048mm, Max edge: 29.6mm
- Z range: -153.1 to -96.2

**Working sphere:**
- 5120 faces
- Watertight, convex
- Euler number: 2
- Min edge: 3.46mm, Max edge: 4.13mm (uniform)

### Attempt 20: Mesh Normalization

Tested normalizing guide mesh (center at origin, scale to ~20mm extent):

```python
guide.vertices -= guide.centroid
current_extent = max(guide.vertices.max(axis=0) - guide.vertices.min(axis=0))
scale = 20 / current_extent
guide.apply_scale(scale)
guide.apply_translation([0, 0, -guide.bounds[0][2] + 15])
```

**Result with simple floor:** SUCCESS - Guide stopped at z=2.0

### Attempt 21: Both Medical Meshes Normalized

Tested both tibia and guide together, both normalized and centered.

**Result:** FAILED - Guide still fell through tibia.

### Attempt 22: Convex Hulls

Tested using convex hulls of both meshes:
- Tibia hull: 934 faces, min_edge=0.27mm
- Guide hull: 384 faces, min_edge=0.27mm

**Result:** FAILED - Guide still fell through tibia.

### Attempt 23: Isolate Static Mesh Behavior

**Tibia hull as static + Sphere as dynamic:**
- **Result:** SUCCESS - Sphere stopped at z=1.9

**Guide hull as static + Sphere as dynamic:**
- **Result:** FAILED - Sphere fell through

**Key finding:** The tibia convex hull works as a static obstacle, but the guide convex hull does NOT work as a static obstacle. This suggests there's something fundamentally different about the guide hull geometry that SOFA cannot handle.

### Pending Investigation: Guide Hull vs Tibia Hull Geometry

Need to analyze the geometric differences between the tibia hull and guide hull:

```python
# Analysis to perform:
tibia_hull = tibia_full.convex_hull
guide_hull = guide_full.convex_hull

# Compare:
# - Is watertight?
# - Is convex? (should be True for convex hull)
# - Euler number
# - Area, Volume
# - Face normal distribution
# - Face area range (check for degenerate faces)
# - Face aspect ratios (check for slivers)
```

This investigation was interrupted and should be continued.

---

## Working Configuration Summary

### Scene Structure That Works

```python
root.addObject("FreeMotionAnimationLoop")
root.addObject("CollisionPipeline")
root.addObject("BruteForceBroadPhase")
root.addObject("BVHNarrowPhase")
root.addObject("LocalMinDistance", alarmDistance=5, contactDistance=2, angleCone=0.0)
root.addObject("CollisionResponse", response="FrictionContactConstraint")
root.addObject("LCPConstraintSolver", maxIt=1000, tolerance=0.001)

# Static object: direct Vec3d positions
static = root.addChild("Static")
static.addObject("MeshSTLLoader", name="loader", filename="static.stl")
static.addObject("MeshTopology", src="@loader")
static.addObject("MechanicalObject", name="mstate", src="@loader")
static.addObject("TriangleCollisionModel", moving=False, simulated=False)
static.addObject("LineCollisionModel", moving=False, simulated=False)
static.addObject("PointCollisionModel", moving=False, simulated=False)

# Dynamic object: Rigid3 with child collision mesh
dynamic = root.addChild("Dynamic")
dynamic.addObject("EulerImplicitSolver", rayleighStiffness=0.1, rayleighMass=0.1)
dynamic.addObject("CGLinearSolver", iterations=25, tolerance=1e-9, threshold=1e-9)
dynamic.addObject("MechanicalObject", name="rigidDOF", template="Rigid3",
                  position=[[x, y, z, qx, qy, qz, qw]])
dynamic.addObject("UniformMass", totalMass=mass)
dynamic.addObject("UncoupledConstraintCorrection")

collision = dynamic.addChild("Collision")
collision.addObject("MeshSTLLoader", name="loader", filename="dynamic.stl")
collision.addObject("MeshTopology", src="@loader")
collision.addObject("MechanicalObject", name="collisionState", src="@loader")
collision.addObject("TriangleCollisionModel")
collision.addObject("LineCollisionModel")
collision.addObject("PointCollisionModel")
collision.addObject("RigidMapping", input="@../rigidDOF", output="@collisionState")
```

### Test Results Summary

| Test | Static Mesh | Dynamic Mesh | Result |
|------|-------------|--------------|--------|
| Simple box floor + Simple box | Inline | Inline | SUCCESS |
| STL box floor + STL box | STL 12-face | STL 12-face | SUCCESS |
| STL floor + 768-face box | STL 12-face | STL 768-face | SUCCESS |
| STL floor + 5120-face sphere | STL 12-face | STL 5120-face | SUCCESS |
| Simple floor + Normalized guide | Inline 12-face | Guide 5000-face | SUCCESS |
| Tibia hull + Sphere | Tibia hull 934-face | Sphere 1280-face | SUCCESS |
| Guide hull + Sphere | Guide hull 384-face | Sphere 1280-face | **FAILED** |
| Tibia hull + Guide hull | Tibia hull 934-face | Guide hull 384-face | **FAILED** |
| Tibia 5k + Guide 5k (original pos) | Tibia 4999-face | Guide 5000-face | **FAILED** |

### Key Findings

1. **Python vs XML is NOT the issue** - Python-created scenes work correctly with proper structure
2. **STL loading is NOT the issue** - STL-loaded meshes work fine
3. **Mesh complexity is NOT the issue** - 5000-face meshes work with synthetic geometry
4. **Rigid3 DOF structure is REQUIRED** for dynamic rigid bodies
5. **Guide hull has problematic geometry** - Even as a static surface, it doesn't work
6. **Tibia hull works** as a static surface
7. **Normalization alone is not sufficient** - The guide mesh has inherent geometry issues

---

## Test Scripts Created

### Collision Investigation Scripts
- `test_xml_collision.py` - Test XML scene loading
- `test_sofa_example.py` - Test SOFA bundled example
- `test_constraint_example.py` - Test AdvancedResponseConstraintBased.scn
- `test_rigid_collision.py` - Test Rigid3 structure (XML)
- `test_rigid_collision_python.py` - Test Rigid3 structure (Python)
- `test_tibia_guide_collision.py` - Test actual medical meshes
- `test_stl_rigid_collision.py` - Test STL loading with Rigid3
- `test_inline_vs_stl.py` - Compare inline vs STL
- `test_both_stl.py` - Both meshes from STL
- `test_higher_poly.py` - 768-face meshes
- `test_5k_faces.py` - 5120-face spheres
- `test_decimated_meshes.py` - 100-face simplified meshes
- `test_translated_meshes.py` - Meshes translated near origin
- `test_proper_positioning.py` - Non-interpenetrated starting positions
- `test_guide_on_sphere.py` - Guide mesh on simple floor
- `test_cleaned_guide.py` - Cleaned guide mesh
- `test_convex_hull.py` - Guide convex hull
- `test_compare_sphere_guide.py` - Side-by-side sphere vs guide
- `test_normalized_full_guide.py` - Full guide normalized
- `test_scaled_guide.py` - 100x scaled guide
- `test_normalized_tibia_guide.py` - Both meshes normalized
- `test_scaled_tibia_guide.py` - Both meshes scaled
- `test_convex_hulls.py` - Both convex hulls
- `test_tibia_hull_with_sphere.py` - Tibia hull + sphere
- `test_guide_static_sphere_dynamic.py` - Guide hull + sphere
- `test_debug_mapping.py` - Debug RigidMapping behavior
- `analyze_meshes.py` - Mesh geometry analysis

### Scene Files
- `scenes/simple_collision.scn` - XML collision test
- `scenes/rigid_collision_test.scn` - XML Rigid3 test

---

## Continued Session - Guide Hull vs Tibia Hull Geometry Analysis

### Geometry Comparison (using repaired STL files)

| Property | Tibia Hull | Guide Hull | Working OBB |
|----------|------------|------------|-------------|
| Vertices | 4532 | 884 | 8 |
| Faces | 9060 | 1764 | 12 |
| Is watertight | True | True | True |
| Is convex | True | True | True |
| Euler number | 2 | 2 | 2 |
| Min edge (mm) | 0.001754 | 0.005099 | 39.8 |
| Short edges (<0.1mm) | 171 (1.3%) | 602 (22.8%) | 0 |
| High aspect faces (>10) | 843 (9.3%) | 839 (47.6%) | 0 |
| Sphericity | 0.723 | 0.860 | 0.806 |
| Upward-facing area | 235 mm² (3.5%) | 2104 mm² (24.8%) | 1739 mm² (13.1%) |

### Tests Performed

1. **Normals check**: Both hulls have 100% outward-facing normals - NOT the issue
2. **Collision groups**: Explicit groups don't help - NOT the issue
3. **STL vs inline data**: Both fail equally - NOT the issue
4. **scipy vs trimesh ConvexHull**: Same result - NOT the issue
5. **Surface sampling + hull**: 100-1000 sample points all fail - NOT the issue
6. **Voxelization + hull**: All pitch values fail - NOT the issue
7. **Sphere position**: Sphere is correctly within XY bounds - NOT the issue

### What Works

| Test | Result |
|------|--------|
| Tibia hull + Sphere | PASS (z=1.90) |
| Guide OBB + Sphere | PASS (z=-0.16) |
| Guide OBB subdivided 1x (48 faces) | PASS |
| Guide OBB subdivided 2x (192 faces) | PASS |
| Simple floor + Sphere | PASS (z=2.00) |
| Icosphere (5k faces) + Sphere | PASS |

### What Fails

| Test | Result |
|------|--------|
| Guide hull + Sphere | FAIL (z=-807) |
| Guide hull (scipy) + Sphere | FAIL |
| Guide sampled hull (100 pts) + Sphere | FAIL |
| Guide sampled hull (1000 pts) + Sphere | FAIL |
| Guide voxelized hull + Sphere | FAIL |
| Remeshed guide hull (no short edges) + Sphere | FAIL |

### Collision Parameter Tests

| Alarm/Contact | Guide Hull Result |
|---------------|-------------------|
| 5/2 (default) | FAIL (z=-807) |
| 10/5 | FAIL (z=-716) |
| 20/10 | FAIL (z=-374) |
| **50/25** | **PASS** (z=4702 - bounced) |
| 100/50 | PASS (z=15242 - bounced way up) |

With very large alarm/contact distances (50/25+), the guide hull DOES work, but the response is exaggerated. This suggests the collision IS detectable but with normal parameters the detection happens too late (tunneling).

### Key Insight: Shape Not Mesh Quality

The guide hull fails even when:
- No short edges (remeshed version)
- Clean triangulation (sampled surface)
- Different triangulation method (scipy)

But the guide OBB (same shape footprint, simple box) always works. This suggests the issue is related to the guide's **convex hull shape** itself, not mesh quality.

### Hypothesis

The guide mesh has vertex distribution that creates a convex hull with a shape that SOFA's collision detection struggles with. Possible factors:
1. The guide hull is more spherical (0.860) than the tibia (0.723), making contact normals less distinct
2. The guide has many more short edges on its convex hull (22.8% vs 1.3%)
3. The triangulation creates thin slivers that SOFA's proximity algorithms fail on

### Workaround

Use the **oriented bounding box (OBB)** as collision geometry for the guide:
```python
guide_obb = guide_mesh.bounding_box_oriented.to_mesh()
```
This provides a working collision shape that approximates the guide's volume.

---

## Next Steps (Recommendations)

1. **Use OBB for guide collision** - As a workaround, use the oriented bounding box
2. **Increase collision parameters** - Use alarm=50, contact=25 if convex hull is required
3. **Create custom collision shape** - Hand-model a simplified collision mesh for the guide
4. **Consult SOFA forums** - Post findings about convex hull shape affecting collision detection
5. **Try SOFA v25.x** - Newer collision system might handle this better
