# Requirements

## Functional Requirements

### Asset Loading

**FR-001: Load STL Meshes**
- Load tibia and guide meshes from STL files
- Validate mesh integrity (watertight, no degenerate faces)
- Report clear errors for invalid or missing files

**FR-002: Apply Coordinate Transforms**
- Apply 4x4 homogeneous transformation matrices
- Support rotation via quaternion or rotation matrix
- Transform meshes between coordinate frames (CT, ICP, simulation)

### Simulation

**FR-003: Create SOFA Scene**
- Create rigid body for tibia (fixed/kinematic)
- Create rigid body for guide (dynamic)
- Configure frictional contact between bodies
- Set solver parameters (timestep, iterations)

**FR-004: Apply Disturbances**
- Apply constant or time-varying forces to guide
- Apply constant or time-varying torques to guide
- Configure disturbance onset time and duration

**FR-005: Extract Pose**
- Extract guide position (x, y, z) per timestep
- Extract guide orientation as quaternion (w, x, y, z)
- Support pose extraction at configurable intervals

### Output

**FR-006: Export Time Series**
- Export pose time series to CSV format
- Export pose time series to JSON format
- Include simulation metadata (parameters, timestamp)

**FR-007: Compare Poses**
- Compute translation error between poses (Euclidean distance)
- Compute rotation error between poses (geodesic distance)
- Report comparison against reference pose (6dof-mc ICP result)

## Non-Functional Requirements

**NFR-001: Headless Operation**
- Run without GUI for batch experiments
- Support command-line configuration
- Exit with appropriate codes (0=success, non-zero=failure)

**NFR-002: Determinism**
- Support random seed configuration
- Same seed produces identical results
- Document any sources of non-determinism

**NFR-003: Error Handling**
- Clear error messages for missing assets
- Clear error messages for missing SOFA installation
- Graceful degradation when optional features unavailable

**NFR-004: Performance**
- 2-second simulation completes in < 60 seconds wall time
- Memory usage < 4GB for typical meshes

## MVM Scope

The Minimum Viable Model (MVM) includes:
- FR-001, FR-002: Asset loading and transforms
- FR-003: Basic SOFA scene (rigid bodies, contact)
- FR-004: Simple constant force disturbance
- FR-005, FR-006: Pose extraction and CSV export
- FR-007: Pose comparison (translation and rotation error)
- NFR-001, NFR-002, NFR-003: Headless, deterministic, good errors

Deferred to future phases:
- Pin constraints (Tier 2)
- Saw blade simulation (Tier 3)
- Monte Carlo experiments
- Advanced contact models
