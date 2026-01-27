#!/usr/bin/env python
"""H-D1: Test angleCone filtering at failing positions."""

import Sofa
import Sofa.Core
import Sofa.Simulation
import trimesh
import numpy as np
from datetime import datetime

print(f"Test H-D1 - Run started: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")

def run_test(angle_cone, solver_type, x_pos, test_name):
    """Run collision test with specified angleCone."""
    print(f"\n{'='*60}")
    print(f"TEST: {test_name}")
    print(f"angleCone: {angle_cone}, Solver: {solver_type}, X: {x_pos}")
    print(f"{'='*60}")

    tibia_full = trimesh.load("/tmp/repaired/CASE4_CroppedTibiaBone-repaired.stl")
    guide_full = trimesh.load("/tmp/repaired/CASE4_ResectThruTibiaGuide-repaired.stl")

    tibia_hull = tibia_full.convex_hull
    guide_obb = guide_full.bounding_box_oriented.to_mesh()

    # Normalize
    tibia_hull.apply_translation(-tibia_hull.centroid)
    tibia_hull.apply_translation([0, 0, -tibia_hull.bounds[1][2]])

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

    for i in range(500):
        Sofa.Simulation.animate(root, 0.001)

    col_z = [p[2] for p in col_mo.position.value]
    final_z = min(col_z)
    result = "PASS" if final_z > -10 else "FAIL"
    print(f"Final: {final_z:.2f} - {result}")
    return final_z, result


def main():
    print("\nTesting angleCone filtering effect\n")

    results = []

    # Test LCP with different angleCone values at X=0
    for angle in [0.0, 0.1, 0.3, 0.5]:
        z, r = run_test(angle, "LCP", 0, f"LCP, angleCone={angle}, X=0")
        results.append(("LCP", angle, 0, z, r))

    # Also test Generic with angleCone (should already work, but let's confirm)
    z, r = run_test(0.0, "Generic", 0, "Generic, angleCone=0.0, X=0 (baseline)")
    results.append(("Generic", 0.0, 0, z, r))

    # Summary
    print(f"\n{'='*60}")
    print("SUMMARY - angleCone Effect")
    print(f"{'='*60}")
    print(f"{'Solver':<10} {'angleCone':<12} {'X':<6} {'Final Z':<12} {'Result':<8}")
    print("-" * 50)
    for solver, angle, x, z, r in results:
        print(f"{solver:<10} {angle:<12} {x:<6} {z:<12.2f} {r:<8}")

    print(f"\nTest H-D1 - Run completed: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")


if __name__ == "__main__":
    main()
