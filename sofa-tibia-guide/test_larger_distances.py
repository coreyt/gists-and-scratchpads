#!/usr/bin/env python
"""H-C1: Test larger alarm/contact distances at failing positions."""

import Sofa
import Sofa.Core
import Sofa.Simulation
import trimesh
import numpy as np
from datetime import datetime

print(f"Test H-C1 - Run started: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")

def run_test(alarm_dist, contact_dist, x_pos, solver_type, test_name):
    """Run collision test with specified distances."""
    print(f"\n{'='*60}")
    print(f"TEST: {test_name}")
    print(f"alarm={alarm_dist}, contact={contact_dist}, X={x_pos}, Solver={solver_type}")
    print(f"{'='*60}")

    tibia_full = trimesh.load("/tmp/repaired/CASE4_CroppedTibiaBone-repaired.stl")
    guide_full = trimesh.load("/tmp/repaired/CASE4_ResectThruTibiaGuide-repaired.stl")

    tibia_hull = tibia_full.convex_hull
    guide_obb = guide_full.bounding_box_oriented.to_mesh()

    # Normalize coordinates
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
    root.addObject("LocalMinDistance", alarmDistance=float(alarm_dist), contactDistance=float(contact_dist), angleCone=0.0)
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
    print("\nTesting effect of alarm/contact distances on collision\n")

    results = []

    # Baseline: default distances at X=0 with LCP (known to fail)
    z, r = run_test(5, 2, 0, "LCP", "Baseline: alarm=5, contact=2, X=0, LCP")
    results.append(("LCP", 5, 2, 0, z, r))

    # H-C1: Larger distances at X=0 with LCP
    z, r = run_test(10, 5, 0, "LCP", "H-C1a: alarm=10, contact=5, X=0, LCP")
    results.append(("LCP", 10, 5, 0, z, r))

    z, r = run_test(15, 10, 0, "LCP", "H-C1b: alarm=15, contact=10, X=0, LCP")
    results.append(("LCP", 15, 10, 0, z, r))

    z, r = run_test(20, 15, 0, "LCP", "H-C1c: alarm=20, contact=15, X=0, LCP")
    results.append(("LCP", 20, 15, 0, z, r))

    # Also test with Generic to see if distances matter there
    z, r = run_test(5, 2, 0, "Generic", "Generic baseline: alarm=5, contact=2, X=0")
    results.append(("Generic", 5, 2, 0, z, r))

    z, r = run_test(15, 10, 0, "Generic", "Generic H-C1: alarm=15, contact=10, X=0")
    results.append(("Generic", 15, 10, 0, z, r))

    # Test at X=10 (another failing position)
    z, r = run_test(5, 2, 10, "LCP", "X=10 baseline: alarm=5, contact=2, LCP")
    results.append(("LCP", 5, 2, 10, z, r))

    z, r = run_test(15, 10, 10, "LCP", "X=10 H-C1: alarm=15, contact=10, LCP")
    results.append(("LCP", 15, 10, 10, z, r))

    # Summary
    print(f"\n{'='*70}")
    print("SUMMARY - Distance Effect on Collision")
    print(f"{'='*70}")
    print(f"{'Solver':<10} {'Alarm':<8} {'Contact':<10} {'X':<6} {'Final Z':<12} {'Result':<8}")
    print("-" * 60)
    for solver, alarm, contact, x, z, r in results:
        print(f"{solver:<10} {alarm:<8} {contact:<10} {x:<6} {z:<12.2f} {r:<8}")

    # Analysis
    print(f"\n{'='*70}")
    print("ANALYSIS")
    print(f"{'='*70}")

    # Find threshold where LCP starts working
    lcp_x0 = [(a, c, z, r) for s, a, c, x, z, r in results if s == "LCP" and x == 0]
    working_lcp = [(a, c, z) for a, c, z, r in lcp_x0 if r == "PASS"]
    if working_lcp:
        min_alarm = min(a for a, c, z in working_lcp)
        print(f"LCP at X=0 starts working at alarm={min_alarm}")
    else:
        print("LCP at X=0 does NOT work even with larger distances")

    print(f"\nTest H-C1 - Run completed: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")


if __name__ == "__main__":
    main()
