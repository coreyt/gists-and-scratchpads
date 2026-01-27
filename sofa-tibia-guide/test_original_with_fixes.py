#!/usr/bin/env python
"""Test original CASE4 positions with both fixes (GenericSolver and angleCone)."""

import Sofa
import Sofa.Core
import Sofa.Simulation
import trimesh
import numpy as np
from datetime import datetime

print(f"Test Original Fixes - Run started: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")

def run_test(solver_type, angle_cone, normalize_coords, test_name):
    """Run collision test with original relative positions."""
    print(f"\n{'='*60}")
    print(f"TEST: {test_name}")
    print(f"Solver: {solver_type}, angleCone: {angle_cone}, Normalize: {normalize_coords}")
    print(f"{'='*60}")

    tibia_full = trimesh.load("/tmp/repaired/CASE4_CroppedTibiaBone-repaired.stl")
    guide_full = trimesh.load("/tmp/repaired/CASE4_ResectThruTibiaGuide-repaired.stl")

    tibia_hull = tibia_full.convex_hull
    guide_obb = guide_full.bounding_box_oriented.to_mesh()

    if normalize_coords:
        # Normalize but preserve relative XY positions
        # Move both to have tibia centered at origin
        offset = tibia_hull.centroid.copy()
        tibia_hull.apply_translation(-offset)
        guide_obb.apply_translation(-offset)

        # Move so tibia top is at z=0
        z_offset = tibia_hull.bounds[1][2]
        tibia_hull.apply_translation([0, 0, -z_offset])
        guide_obb.apply_translation([0, 0, -z_offset])

        print(f"  Normalized with tibia at origin")
        print(f"  Guide relative XY: ({guide_obb.centroid[0]:.1f}, {guide_obb.centroid[1]:.1f})")
    else:
        print(f"  Using original coordinates")
        print(f"  Tibia centroid: {tibia_hull.centroid}")
        print(f"  Guide centroid: {guide_obb.centroid}")

    # Ensure gap
    gap = guide_obb.bounds[0][2] - tibia_hull.bounds[1][2]
    if gap < 10:
        guide_obb.apply_translation([0, 0, 15 - gap])
        print(f"  Added Z offset for 15mm gap")

    print(f"  Tibia Z: {tibia_hull.bounds[0][2]:.1f} to {tibia_hull.bounds[1][2]:.1f}")
    print(f"  Guide Z: {guide_obb.bounds[0][2]:.1f} to {guide_obb.bounds[1][2]:.1f}")

    tibia_verts = tibia_hull.vertices.tolist()
    tibia_faces = tibia_hull.faces.tolist()
    guide_verts = guide_obb.vertices.tolist()
    guide_faces = guide_obb.faces.tolist()

    root = Sofa.Core.Node("root")
    root.gravity = [0, 0, -9810]
    root.dt = 0.001

    root.addObject("RequiredPlugin", pluginName=[
        "Sofa.Component.AnimationLoop",
        "Sofa.Component.Collision.Detection.Algorithm",
        "Sofa.Component.Collision.Detection.Intersection",
        "Sofa.Component.Collision.Geometry",
        "Sofa.Component.Collision.Response.Contact",
        "Sofa.Component.Constraint.Lagrangian.Correction",
        "Sofa.Component.Constraint.Lagrangian.Solver",
        "Sofa.Component.IO.Mesh",
        "Sofa.Component.LinearSolver.Iterative",
        "Sofa.Component.Mapping.NonLinear",
        "Sofa.Component.Mass",
        "Sofa.Component.ODESolver.Backward",
        "Sofa.Component.StateContainer",
        "Sofa.Component.Topology.Container.Constant",
    ])

    root.addObject("FreeMotionAnimationLoop")
    root.addObject("CollisionPipeline")
    root.addObject("BruteForceBroadPhase")
    root.addObject("BVHNarrowPhase")
    root.addObject("LocalMinDistance", alarmDistance=5.0, contactDistance=2.0, angleCone=float(angle_cone))
    root.addObject("CollisionResponse", response="FrictionContactConstraint")

    if solver_type == "Generic":
        root.addObject("GenericConstraintSolver", maxIterations=1000, tolerance=0.001)
    else:
        root.addObject("LCPConstraintSolver", maxIt=1000, tolerance=0.001)

    tibia_node = root.addChild("Tibia")
    tibia_node.addObject("MeshTopology", name="topology", position=tibia_verts, triangles=tibia_faces)
    tibia_node.addObject("MechanicalObject", name="mstate", src="@topology")
    tibia_node.addObject("TriangleCollisionModel", moving=False, simulated=False)
    tibia_node.addObject("LineCollisionModel", moving=False, simulated=False)
    tibia_node.addObject("PointCollisionModel", moving=False, simulated=False)

    guide_node = root.addChild("Guide")
    guide_node.addObject("EulerImplicitSolver", rayleighStiffness=0.1, rayleighMass=0.1)
    guide_node.addObject("CGLinearSolver", iterations=25, tolerance=1e-9, threshold=1e-9)
    guide_node.addObject("MechanicalObject", name="rigidDOF", template="Rigid3", position=[[0, 0, 0, 0, 0, 0, 1]])
    guide_node.addObject("UniformMass", totalMass=0.1)
    guide_node.addObject("UncoupledConstraintCorrection")

    guide_col = guide_node.addChild("Collision")
    guide_col.addObject("MeshTopology", name="topology", position=guide_verts, triangles=guide_faces)
    guide_col.addObject("MechanicalObject", name="collisionState", src="@topology")
    guide_col.addObject("TriangleCollisionModel")
    guide_col.addObject("LineCollisionModel")
    guide_col.addObject("PointCollisionModel")
    guide_col.addObject("RigidMapping", input="@../rigidDOF", output="@collisionState")

    Sofa.Simulation.init(root)
    col_mo = guide_col.getObject("collisionState")

    tibia_top = tibia_hull.bounds[1][2]

    for i in range(500):
        Sofa.Simulation.animate(root, 0.001)
        if i % 100 == 0:
            col_z = [p[2] for p in col_mo.position.value]
            rel_z = min(col_z) - tibia_top
            print(f"  Step {i}: relative_z={rel_z:.2f}")

    col_z = [p[2] for p in col_mo.position.value]
    final_z = min(col_z)
    rel_z = final_z - tibia_top
    result = "PASS" if rel_z > -10 else "FAIL"
    print(f"Final relative Z: {rel_z:.2f} - {result}")
    return rel_z, result


def main():
    print("\nTesting original relative positions with various fixes\n")

    results = []

    # Baseline: LCP, angleCone=0, not normalized (should fail)
    z, r = run_test("LCP", 0.0, False, "Baseline: LCP, angle=0, original coords")
    results.append(("LCP", 0.0, False, z, r))

    # Fix 1: GenericSolver, original coords
    z, r = run_test("Generic", 0.0, False, "Fix 1: Generic, angle=0, original coords")
    results.append(("Generic", 0.0, False, z, r))

    # Fix 2: LCP with angleCone, original coords
    z, r = run_test("LCP", 0.1, False, "Fix 2: LCP, angle=0.1, original coords")
    results.append(("LCP", 0.1, False, z, r))

    # Fix 3: GenericSolver, normalized coords (preserving relative XY)
    z, r = run_test("Generic", 0.0, True, "Fix 3: Generic, angle=0, normalized coords")
    results.append(("Generic", 0.0, True, z, r))

    # Fix 4: LCP with angleCone, normalized coords
    z, r = run_test("LCP", 0.1, True, "Fix 4: LCP, angle=0.1, normalized coords")
    results.append(("LCP", 0.1, True, z, r))

    # Summary
    print(f"\n{'='*60}")
    print("SUMMARY - Original Relative Positions")
    print(f"{'='*60}")
    print(f"{'Solver':<10} {'angleCone':<10} {'Normalized':<12} {'Rel Z':<10} {'Result':<8}")
    print("-" * 55)
    for solver, angle, norm, z, r in results:
        print(f"{solver:<10} {angle:<10} {str(norm):<12} {z:<10.2f} {r:<8}")

    print(f"\nTest Original Fixes - Run completed: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")


if __name__ == "__main__":
    main()
