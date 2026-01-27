# Asset Pipeline

## Overview

This document describes the mesh preparation workflow for simulation assets. Properly prepared meshes are critical for accurate contact detection and simulation stability.

## Required Mesh Files

| File | Description | Typical Size |
|------|-------------|--------------|
| `tibia.stl` | Distal tibia bone surface | 50k-200k triangles |
| `tibia_collision.stl` | Decimated tibia for collision | 5k-20k triangles |
| `guide.stl` | Cutting guide visual mesh | 10k-50k triangles |
| `guide_collision.stl` | Decimated guide for collision | 1k-10k triangles |

## Naming Conventions

```
{anatomy}_{variant}_{type}.stl

Examples:
  tibia_left_visual.stl
  tibia_left_collision.stl
  guide_v2_visual.stl
  guide_v2_collision.stl
```

## Coordinate Frame Requirements

### Source Frame (CT/ICP)

Meshes from CT segmentation or ICP registration are typically in:
- **Origin**: Scanner isocenter or anatomical landmark
- **Units**: Millimeters
- **Orientation**: RAS (Right-Anterior-Superior) or LPS

### Simulation Frame

SOFA simulation uses:
- **Origin**: Tibia centroid (after transform)
- **Units**: Millimeters
- **Orientation**: Right-handed, Z-up

### Transform Specification

Transforms are specified as 4x4 homogeneous matrices in row-major order:

```yaml
# config.yaml
transforms:
  ct_to_sim:
    matrix: [
      1, 0, 0, -150.0,
      0, 1, 0, -120.0,
      0, 0, 1, -80.0,
      0, 0, 0, 1
    ]
```

Or as rotation (quaternion) + translation:

```yaml
transforms:
  icp_to_sim:
    rotation: [1, 0, 0, 0]  # w, x, y, z
    translation: [0, 0, 0]  # x, y, z in mm
```

## Mesh Quality Requirements

### Visual Meshes

- Manifold (watertight)
- No self-intersections
- No degenerate triangles (area > 0.01 mm²)
- Consistent face normals (outward-pointing)

### Collision Meshes

All visual mesh requirements, plus:
- Triangle count: 1k-20k (balance accuracy vs. performance)
- Minimum edge length: > 0.5mm
- Maximum aspect ratio: < 10:1
- No thin features (< 1mm thickness)

## Decimation Guidelines

### Recommended Tool: MeshLab

1. **Load** original mesh
2. **Filters > Remeshing > Quadric Edge Collapse Decimation**
   - Target faces: 5000-10000
   - Preserve boundary: Yes
   - Preserve topology: Yes
   - Quality threshold: 0.5
3. **Filters > Cleaning > Remove Duplicate Faces**
4. **Filters > Cleaning > Remove Unreferenced Vertices**
5. **Export** as binary STL

### Validation Script

```bash
# Validate mesh using trimesh
python -c "
import trimesh
mesh = trimesh.load('guide_collision.stl')
print(f'Triangles: {len(mesh.faces)}')
print(f'Watertight: {mesh.is_watertight}')
print(f'Volume: {mesh.volume:.2f} mm³')
"
```

## Transform Workflow

### From 6dof-mc ICP

1. Load ICP result containing `T_guide_to_tibia` (4x4 matrix)
2. Extract guide initial pose from transform
3. Apply to guide mesh in simulation

```python
import numpy as np
from tibia_guide.assets.transform import matrix_to_pose

# ICP result: guide pose relative to tibia
T_guide = np.array([...])  # 4x4 from 6dof-mc

# Extract position and quaternion
position, quaternion = matrix_to_pose(T_guide)
```

### Frame Chain

```
CT Frame ──[T_ct_to_icp]──> ICP Frame ──[T_icp_to_sim]──> Sim Frame
```

Each transform should be documented with its source (CT parameters, ICP output, manual alignment).

## Asset Storage

### Local Development

Place meshes in `assets/` directory (gitignored):

```
assets/
├── tibia.stl
├── tibia_collision.stl
├── guide.stl
├── guide_collision.stl
└── transforms.yaml
```

### Shared Assets

For team collaboration, store assets in:
- Network drive with version control
- Git LFS for binary files
- DVC (Data Version Control)

Reference by path in configuration:

```yaml
assets:
  base_path: /shared/project/assets/v1.2/
  tibia: tibia_collision.stl
  guide: guide_collision.stl
```

## Troubleshooting

### "Mesh not watertight"

- Check for holes using MeshLab (Filters > Selection > Select Non-Manifold Edges)
- Fill small holes: Filters > Remeshing > Close Holes

### "Simulation unstable"

- Reduce collision mesh complexity
- Check for self-intersections
- Increase solver iterations

### "Contact not detected"

- Verify meshes overlap in initial configuration
- Check collision margin settings
- Ensure normals point outward
