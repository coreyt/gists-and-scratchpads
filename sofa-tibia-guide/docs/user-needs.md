# User Needs

## Primary User

**Design Verification Engineer** - Responsible for validating surgical cutting guide designs meet mechanical performance requirements before clinical use.

## Use Cases

### UC-001: Validate Guide Seating Stability

**As a** design verification engineer,
**I want to** simulate guide seating under perturbation forces,
**so that** I can verify the guide maintains stable contact with the tibia surface.

**Acceptance Criteria:**
- Apply configurable disturbance forces (0-50N) and torques (0-5Nm)
- Measure guide displacement from seated position
- Report pass/fail against displacement threshold

### UC-002: Quantify Pin Angle Sensitivity

**As a** design verification engineer,
**I want to** vary pin insertion angles within tolerance bands,
**so that** I can quantify how manufacturing tolerances affect guide stability.

**Acceptance Criteria:**
- Parameterize pin angle variation (±2°)
- Run Monte Carlo simulations across angle combinations
- Report stability metrics distribution

### UC-003: Verify Saw Slot Constraint

**As a** design verification engineer,
**I want to** simulate saw blade insertion and vibration,
**so that** I can verify the slot geometry adequately constrains the blade.

**Acceptance Criteria:**
- Model blade as rigid body within slot
- Apply oscillating forces representing saw vibration
- Measure blade deviation from slot centerline

### UC-004: Cross-Validate with 6dof-mc ICP

**As a** design verification engineer,
**I want to** compare SOFA equilibrium poses with 6dof-mc ICP results,
**so that** I can sanity-check both systems against each other.

**Acceptance Criteria:**
- Load same mesh assets used in 6dof-mc
- Run simulation to equilibrium
- Export pose in compatible format
- Compare translation error < 0.1mm, rotation error < 0.1°

## Success Criteria

1. **Reproducibility**: Same inputs produce same outputs (deterministic simulation)
2. **Traceability**: All simulation parameters logged with results
3. **Comparability**: Metrics format compatible with 6dof-mc outputs
4. **Automation**: Batch runs without manual intervention
