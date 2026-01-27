#!/usr/bin/env python
"""H-B1: Test GenericConstraintSolver at failing position (X=0)."""

import Sofa
import Sofa.Core
import Sofa.Simulation
import trimesh
import numpy as np
from datetime import datetime

print(f"Test H-B1 - Run started: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")

def run_test(solver_type, x_pos, test_name):
    """Run collision test with specified solver."""
    print(f"\n{'='*60}")
    print(f"TEST: {test_name}")
    print(f"Solver: {solver_type}, Position: X={x_pos}")
    print(f"{'='*60}")

    tibia_full = trimesh.load("/tmp/repaired/CASE4_CroppedTibiaBone-repaired.stl")
    guide_full = trimesh.load("/tmp/repaired/CASE4_ResectThruTibiaGuide-repaired.stl")

    tibia_hull = tibia_full.convex_hull
    guide_obb = guide_full.bounding_box_oriented.to_mesh()

    # Normalize tibia
    tibia_hull.apply_translation(-tibia_hull.centroid)
    tibia_hull.apply_translation([0, 0, -tibia_hull.bounds[1][2]])

    # Normalize and position guide
    guide_obb.apply_translation(-guide_obb.centroid)
    guide_obb.apply_translation([x_pos, 0, -guide_obb.bounds[0][2] + 15])

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
    root.addObject("LocalMinDistance", alarmDistance=5.0, contactDistance=2.0, angleCone=0.0)
    root.addObject("CollisionResponse", response="FrictionContactConstraint")

    # Use specified solver
    if solver_type == "GenericConstraintSolver":
        root.addObject("GenericConstraintSolver", maxIterations=1000, tolerance=0.001)
    else:
        root.addObject("LCPConstraintSolver", maxIt=1000, tolerance=0.001)

    # Static tibia
    tibia_node = root.addChild("Tibia")
    tibia_node.addObject("MeshTopology", name="topology", position=tibia_verts, triangles=tibia_faces)
    tibia_node.addObject("MechanicalObject", name="mstate", src="@topology")
    tibia_node.addObject("TriangleCollisionModel", moving=False, simulated=False)
    tibia_node.addObject("LineCollisionModel", moving=False, simulated=False)
    tibia_node.addObject("PointCollisionModel", moving=False, simulated=False)

    # Dynamic guide
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

    for i in range(500):
        Sofa.Simulation.animate(root, 0.001)
        if i % 100 == 0:
            col_z = [p[2] for p in col_mo.position.value]
            print(f"  Step {i}: z_min={min(col_z):.2f}")

    col_z = [p[2] for p in col_mo.position.value]
    final_z = min(col_z)
    result = "PASS" if final_z > -10 else "FAIL"
    print(f"Final: {final_z:.2f} - {result}")
    return final_z, result


def main():
    print("\nComparing LCPConstraintSolver vs GenericConstraintSolver at failing positions\n")

    results = []

    # Test at X=0 (known to fail with LCP)
    z1, r1 = run_test("LCPConstraintSolver", 0, "LCP at X=0 (baseline)")
    results.append(("LCP", 0, z1, r1))

    z2, r2 = run_test("GenericConstraintSolver", 0, "Generic at X=0")
    results.append(("Generic", 0, z2, r2))

    # Test at X=10 (also known to fail)
    z3, r3 = run_test("LCPConstraintSolver", 10, "LCP at X=10 (baseline)")
    results.append(("LCP", 10, z3, r3))

    z4, r4 = run_test("GenericConstraintSolver", 10, "Generic at X=10")
    results.append(("Generic", 10, z4, r4))

    # Summary
    print(f"\n{'='*60}")
    print("SUMMARY")
    print(f"{'='*60}")
    print(f"{'Solver':<10} {'X Pos':<8} {'Final Z':<12} {'Result':<8}")
    print("-" * 40)
    for solver, x, z, r in results:
        print(f"{solver:<10} {x:<8} {z:<12.2f} {r:<8}")

    print(f"\nTest H-B1 - Run completed: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")


if __name__ == "__main__":
    main()
