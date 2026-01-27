# Architecture

## Overview

The system uses SOFA Framework for physics simulation with a Python interface via SofaPython3. The architecture separates concerns into asset management, simulation execution, and metrics extraction.

## Component Diagram

```
┌─────────────────────────────────────────────────────────────────┐
│                        experiments/                              │
│                     run_disturbance.py                          │
│                    (CLI entry point)                            │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                         scenes/                                  │
│                     mvm_seating.py                              │
│                  (SOFA scene definition)                        │
└─────────────────────────────────────────────────────────────────┘
                              │
            ┌─────────────────┼─────────────────┐
            ▼                 ▼                 ▼
┌───────────────────┐ ┌───────────────┐ ┌───────────────────┐
│  tibia_guide/     │ │ tibia_guide/  │ │   tibia_guide/    │
│     assets/       │ │   metrics/    │ │     config.py     │
│  loader.py        │ │   pose.py     │ │  (configuration)  │
│  transform.py     │ │   contact.py  │ │                   │
└───────────────────┘ └───────────────┘ └───────────────────┘
```

## SOFA Scene Graph

```
Root
├── RequiredPlugins (Sofa.Component.*, SofaPython3)
├── VisualStyle (optional, for debugging)
├── DefaultAnimationLoop
├── DefaultVisualManagerLoop
├── FreeMotionAnimationLoop
├── GenericConstraintSolver
│
├── CollisionPipeline
│   ├── BruteForceBroadPhase
│   ├── BVHNarrowPhase
│   ├── CollisionResponse
│   └── LocalMinDistance
│
├── TibiaNode (fixed)
│   ├── MechanicalObject (rigid, position=[0,0,0,0,0,0,1])
│   ├── UniformMass
│   ├── FixedConstraint (indices=[0])
│   └── CollisionNode
│       ├── MeshSTLLoader (filename=tibia.stl)
│       ├── MeshTopology
│       ├── MechanicalObject
│       ├── TriangleCollisionModel
│       └── RigidMapping
│
├── GuideNode (dynamic)
│   ├── MechanicalObject (rigid, position=initial_pose)
│   ├── UniformMass (totalMass=0.05)
│   ├── UncoupledConstraintCorrection
│   ├── ConstantForceField (forces=disturbance)
│   └── CollisionNode
│       ├── MeshSTLLoader (filename=guide.stl)
│       ├── MeshTopology
│       ├── MechanicalObject
│       ├── TriangleCollisionModel (contactFriction=0.3)
│       └── RigidMapping
│
└── PoseController (Python script controller)
    └── onAnimateEndEvent -> extract_pose()
```

## Data Flow

```
                    ┌──────────────┐
                    │  STL Files   │
                    │ (tibia/guide)│
                    └──────┬───────┘
                           │
                           ▼
                    ┌──────────────┐
                    │   loader.py  │
                    │ (validation) │
                    └──────┬───────┘
                           │
                           ▼
                    ┌──────────────┐
                    │transform.py  │
                    │(frame align) │
                    └──────┬───────┘
                           │
                           ▼
┌──────────────┐    ┌──────────────┐    ┌──────────────┐
│  config.py   │───▶│mvm_seating.py│───▶│   SOFA       │
│ (parameters) │    │(scene build) │    │  Simulation  │
└──────────────┘    └──────────────┘    └──────┬───────┘
                                               │
                                               ▼
                                        ┌──────────────┐
                                        │   pose.py    │
                                        │(extraction)  │
                                        └──────┬───────┘
                                               │
                           ┌───────────────────┼───────────────────┐
                           ▼                   ▼                   ▼
                    ┌──────────────┐    ┌──────────────┐    ┌──────────────┐
                    │  pose.csv    │    │  pose.json   │    │  comparison  │
                    │ (time series)│    │ (metadata)   │    │   metrics    │
                    └──────────────┘    └──────────────┘    └──────────────┘
```

## Key Design Decisions

### 1. Rigid Body Assumption (MVM)

Both tibia and guide are modeled as rigid bodies. This simplifies the simulation and is appropriate for:
- Metal/plastic guides (negligible deformation)
- Cortical bone surface (high stiffness)

Future tiers may add deformable models for soft tissue interaction.

### 2. Frictional Contact

Using SOFA's constraint-based contact with:
- Friction coefficient: 0.3 (typical for metal-bone)
- Contact tolerance: 0.1mm
- Response type: FrictionContact

### 3. Coordinate Frames

Three coordinate frames are relevant:
- **CT frame**: Original imaging coordinate system
- **ICP frame**: Frame after ICP registration (6dof-mc output)
- **Simulation frame**: SOFA world coordinates

Transforms between frames are explicit 4x4 matrices stored in configuration.

### 4. Pose Representation

- Position: (x, y, z) in millimeters
- Orientation: Quaternion (w, x, y, z), scalar-first convention
- Consistent with SOFA's Rigid3d template

### 5. Controller Pattern

A Python controller attached to the scene handles:
- Pose extraction each timestep
- Disturbance force scheduling
- Termination conditions
