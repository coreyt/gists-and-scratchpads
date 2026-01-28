# SOFA Tibial Cutting Guide Simulation

Physics simulation for surgical cutting guide placement on the distal anterior tibia using the SOFA framework. Models guide seating with frictional contact to verify design robustness.

## Quick Start

```python
from tibia_guide import find_seating_pose

result = find_seating_pose(
    tibia_mesh="path/to/tibia.stl",
    guide_mesh="path/to/guide.stl",
)

print(f"Guide position: {result.pose.position}")      # [x, y, z] in mm
print(f"Guide orientation: {result.pose.quaternion}") # [w, x, y, z]
print(f"Converged: {result.converged}")               # True/False
print(f"Gap to tibia: {result.final_gap_mm:.2f}mm")   # Distance above surface
```

## Features

- **Simple API**: Single function call returns seated guide pose
- **Automatic coordinate handling**: Normalizes coordinates internally to prevent numerical issues
- **Convergence detection**: Stops when guide reaches equilibrium
- **Original frame output**: Returns pose in original mesh coordinate frame

## Test Results

Validated with CASE4 surgical meshes:

| Metric | Value |
|--------|-------|
| Final position | [-28.33, 2.87, -62.32] mm |
| Gap to tibia surface | 0.53 mm |
| Converged | Yes |
| Simulation steps | 1901 (~1.9s) |
| Orientation drift | < 3° from upright |

## Installation

### Prerequisites

- Python 3.10+
- SOFA v24.06+ with SofaPython3

### Environment Setup

```bash
# Create conda environment
conda env create -f environment.yml
conda activate sofa-tibia-guide

# Install package
pip install -e .
```

### SOFA Installation

SOFA must be installed from pre-built binaries:

1. **Download SOFA** from [releases](https://github.com/sofa-framework/sofa/releases):
   - Python 3.10: Use SOFA v24.06.00
   - Python 3.12: Use SOFA v24.12.00 or v25.06.00

2. **Extract and configure**:

```bash
# Extract
unzip SOFA_v24.06.00_Linux.zip -d ~/sofa
export SOFA_ROOT=~/sofa/SOFA_v24.06.00_Linux

# Set paths (add to .bashrc)
export LD_LIBRARY_PATH=$SOFA_ROOT/lib:$LD_LIBRARY_PATH
```

3. **Verify**:

```bash
python -c "import Sofa; print('SOFA ready')"
```

## Usage

### High-Level API (Recommended)

```python
from tibia_guide import find_seating_pose

# Basic usage
result = find_seating_pose("tibia.stl", "guide.stl")

# With options
result = find_seating_pose(
    tibia_mesh="tibia.stl",
    guide_mesh="guide.stl",
    max_simulation_time=2.0,       # Max sim time (seconds)
    convergence_threshold=0.01,    # Stop when position change < threshold (mm)
    guide_mass_kg=0.1,             # Guide mass
)

# Access results
print(result.pose.position)    # [x, y, z] in original coordinate frame
print(result.pose.quaternion)  # [w, x, y, z] orientation
print(result.converged)        # Did simulation reach equilibrium?
print(result.steps)            # Number of simulation steps
print(result.final_gap_mm)     # Guide bottom to tibia top distance
```

### Low-Level API

For more control over the simulation:

```python
import Sofa
import Sofa.Simulation
from scenes.mvm_seating import create_scene, extract_guide_pose, normalize_meshes
import trimesh

# Load and prepare meshes
tibia = trimesh.load("tibia.stl")
guide = trimesh.load("guide.stl")

# IMPORTANT: Normalize coordinates to prevent numerical issues
norm_info = normalize_meshes(tibia, guide)

# Save normalized meshes and create scene
# ... (see examples/basic_seating.py for full example)
```

### Command Line

```bash
# Run with any STL files
python examples/basic_seating.py tibia.stl guide.stl

# Validate with CASE4 test meshes
python examples/test_case4.py
```

### Running Tests

```bash
# Unit tests (no SOFA required)
pytest tests/unit/

# All tests (SOFA required)
export SOFA_ROOT=/path/to/SOFA
export LD_LIBRARY_PATH=$SOFA_ROOT/lib:$LD_LIBRARY_PATH
pytest tests/
```

## Key Learnings

### SOFA Collision Detection Issues

During development, we discovered position-dependent collision failures with SOFA's default settings. Investigation revealed **two independent root causes**:

#### 1. LCPConstraintSolver / Ghost Collision Issue

The default `LCPConstraintSolver` has known numerical issues. At certain mesh positions, "ghost collisions" occur where internal triangle edges cause incorrect contact normal computation.

**Fix**: Use `GenericConstraintSolver` and/or set `angleCone >= 0.1` in `LocalMinDistance`.

#### 2. Large Coordinate Values

SOFA collision detection has numerical precision issues with coordinates far from origin (e.g., Z = -125mm).

**Fix**: Normalize mesh coordinates so the tibia centroid is at origin with top surface at Z=0.

### Working Configuration

The `find_seating_pose` function handles these issues automatically. For manual scene creation:

```python
# 1. ALWAYS normalize coordinates
from scenes.mvm_seating import normalize_meshes
norm_info = normalize_meshes(tibia_mesh, guide_mesh)

# 2. Use GenericConstraintSolver (not LCPConstraintSolver)
root.addObject("GenericConstraintSolver", maxIterations=1000, tolerance=0.001)

# 3. Use angleCone filtering for edge contact rejection
root.addObject("LocalMinDistance", alarmDistance=5.0, contactDistance=2.0, angleCone=0.1)

# 4. For rigid bodies with mesh at world coords, start at origin
guide_initial_position = [0.0, 0.0, 0.0]  # NOT the mesh centroid
```

### RigidMapping Coordinate Frame

When using `MeshSTLLoader` with meshes already positioned at world coordinates:
- Set rigid body initial position to `[0, 0, 0]`
- The mesh vertices define the starting position
- The rigid body position becomes the **delta** from initial position
- World position = `mesh_centroid + rigid_body_position`

## Project Structure

```
sofa-tibia-guide/
├── src/tibia_guide/        # Core library
│   ├── seating.py          # High-level find_seating_pose API
│   ├── assets/             # Mesh loading and transforms
│   └── metrics/            # Pose comparison
├── scenes/
│   └── mvm_seating.py      # SOFA scene creation
├── examples/
│   ├── basic_seating.py    # CLI tool
│   └── test_case4.py       # Validation script
├── tests/
│   ├── unit/               # Unit tests (no SOFA)
│   └── integration/        # Integration tests (SOFA required)
└── docs/
    ├── test-scenarios.md   # Detailed test log and findings
    └── architecture.md     # System design
```

## Documentation

- [Test Scenarios](docs/test-scenarios.md) - Comprehensive test log with 23+ tests
- [Architecture](docs/architecture.md) - System design
- [Requirements](docs/requirements.md) - Functional specifications

## API Reference

### `find_seating_pose()`

```python
def find_seating_pose(
    tibia_mesh: Union[trimesh.Trimesh, str, Path],
    guide_mesh: Union[trimesh.Trimesh, str, Path],
    max_simulation_time: float = 2.0,
    convergence_threshold: float = 0.01,
    convergence_check_interval: int = 100,
    dt: float = 0.001,
    guide_mass_kg: float = 0.1,
    collision_simplification: bool = True,
    initial_gap_mm: float = 15.0,
) -> SeatingResult
```

**Parameters:**
- `tibia_mesh`: Tibia STL path or trimesh object
- `guide_mesh`: Guide STL path or trimesh object
- `max_simulation_time`: Maximum simulation time in seconds
- `convergence_threshold`: Position change threshold for convergence (mm)
- `convergence_check_interval`: Check convergence every N steps
- `dt`: Simulation timestep in seconds
- `guide_mass_kg`: Guide mass in kg
- `collision_simplification`: Use convex hull (tibia) and OBB (guide) for stability
- `initial_gap_mm`: Minimum initial gap between meshes

**Returns:** `SeatingResult` dataclass with:
- `pose`: `Pose` object with `position` and `quaternion` in original coordinate frame
- `converged`: Whether simulation reached equilibrium
- `steps`: Number of simulation steps taken
- `final_gap_mm`: Distance from guide bottom to tibia surface

## License

Proprietary - Internal use only
