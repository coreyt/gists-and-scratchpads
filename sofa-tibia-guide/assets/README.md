# Assets

This directory contains mesh files for simulation. These files are not tracked in git due to their size.

## Required Files

| File | Description |
|------|-------------|
| `tibia.stl` | Tibia bone surface mesh |
| `tibia_collision.stl` | Decimated tibia for collision detection |
| `guide.stl` | Cutting guide visual mesh |
| `guide_collision.stl` | Decimated guide for collision detection |

## Preparation

See [Asset Pipeline](../docs/asset-pipeline.md) for detailed mesh preparation instructions.

## Quick Validation

```bash
python -c "
import trimesh
for name in ['tibia', 'guide']:
    for suffix in ['', '_collision']:
        path = f'{name}{suffix}.stl'
        try:
            m = trimesh.load(path)
            status = '✓' if m.is_watertight else '✗'
            print(f'{status} {path}: {len(m.faces)} faces')
        except FileNotFoundError:
            print(f'  {path}: NOT FOUND')
"
```

## Test Assets

For unit testing without real anatomy meshes, generate simple test shapes:

```python
import trimesh
import numpy as np

# Create a simple cube for testing
cube = trimesh.creation.box(extents=[10, 10, 10])
cube.export('test_cube.stl')

# Create a simple cylinder
cylinder = trimesh.creation.cylinder(radius=5, height=20)
cylinder.export('test_cylinder.stl')
```
