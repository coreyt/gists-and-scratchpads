#!/usr/bin/env python
"""H-B2: Test GenericConstraintSolver at original CASE4 positions."""

import Sofa
import Sofa.Core
import Sofa.Simulation
import trimesh
import numpy as np
from datetime import datetime

print(f"Test H-B2 - Run started: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")

def run_test(solver_type, test_name):
    """Run collision test with original positions."""
    print(f"\n{'='*60}")
    print(f"TEST: {test_name}")
    print(f"Solver: {solver_type}")
    print(f"{'='*60}")

    tibia_full = trimesh.load("/tmp/repaired/CASE4_CroppedTibiaBone-repaired.stl")
    guide_full = trimesh.load("/tmp/repaired/CASE4_ResectThruTibiaGuide-repaired.stl")

    tibia_hull = tibia_full.convex_hull
    guide_obb = guide_full.bounding_box_oriented.to_mesh()

    print(f"Original positions:")
    print(f"  Tibia centroid: [{tibia_hull.centroid[0]:.1f}, {tibia_hull.centroid[1]:.1f}, {tibia_hull.centroid[2]:.1f}]")
    print(f"  Guide centroid: [{guide_obb.centroid[0]:.1f}, {guide_obb.centroid[1]:.1f}, {guide_obb.centroid[2]:.1f}]")

    # Keep original XY positions, just ensure gap in Z
    gap = guide_obb.bounds[0][2] - tibia_hull.bounds[1][2]
    if gap < 10:
        guide_obb.apply_translation([0, 0, 15 - gap])
        print(f"  Added Z offset to create 15mm gap")

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
    root.addObject("LocalMinDistance", alarmDistance=5.0, contactDistance=2.0, angleCone=0.0)
    root.addObject("CollisionResponse", response="FrictionContactConstraint")

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

    tibia_top = tibia_hull.bounds[1][2]

    for i in range(500):
        Sofa.Simulation.animate(root, 0.001)
        if i % 100 == 0:
            col_z = [p[2] for p in col_mo.position.value]
            rel_z = min(col_z) - tibia_top
            print(f"  Step {i}: z_min={min(col_z):.2f}, relative_to_tibia={rel_z:.2f}")

    col_z = [p[2] for p in col_mo.position.value]
    final_z = min(col_z)
    rel_z = final_z - tibia_top
    result = "PASS" if rel_z > -10 else "FAIL"
    print(f"Final: {final_z:.2f} (relative: {rel_z:.2f}) - {result}")
    return final_z, rel_z, result


def main():
    print("\nTesting original CASE4 positions with different solvers\n")

    # Test with LCP (expected to fail based on P011)
    z1, rel1, r1 = run_test("LCPConstraintSolver", "LCP at original position (baseline)")

    # Test with Generic
    z2, rel2, r2 = run_test("GenericConstraintSolver", "Generic at original position")

    # Summary
    print(f"\n{'='*60}")
    print("SUMMARY - Original CASE4 Positions")
    print(f"{'='*60}")
    print(f"{'Solver':<12} {'Final Z':<12} {'Relative':<12} {'Result':<8}")
    print("-" * 45)
    print(f"{'LCP':<12} {z1:<12.2f} {rel1:<12.2f} {r1:<8}")
    print(f"{'Generic':<12} {z2:<12.2f} {rel2:<12.2f} {r2:<8}")

    print(f"\nTest H-B2 - Run completed: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")


if __name__ == "__main__":
    main()
