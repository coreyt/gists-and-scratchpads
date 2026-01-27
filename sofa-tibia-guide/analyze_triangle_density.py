#!/usr/bin/env python
"""H-A1: Analyze triangle density and internal edges at failing vs working positions."""

import trimesh
import numpy as np
from datetime import datetime

print(f"Test H-A1 - Run started: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")


def analyze_contact_region(mesh, x_pos, y_pos=0, region_radius=20):
    """Analyze triangles in the contact region for a given XY position."""

    # Find triangles whose centroid is within region_radius of (x_pos, y_pos)
    face_centroids = mesh.triangles_center

    # Only consider faces on top surface (highest Z values)
    z_threshold = mesh.bounds[1][2] - 10  # Top 10mm
    top_mask = face_centroids[:, 2] > z_threshold

    # Filter by XY distance
    xy_distances = np.sqrt(
        (face_centroids[:, 0] - x_pos)**2 +
        (face_centroids[:, 1] - y_pos)**2
    )
    region_mask = xy_distances < region_radius

    combined_mask = top_mask & region_mask
    region_faces = np.where(combined_mask)[0]

    if len(region_faces) == 0:
        return None

    # Compute statistics for faces in this region
    face_areas = mesh.area_faces[region_faces]
    face_normals = mesh.face_normals[region_faces]

    # Edge lengths for triangles in region
    triangles = mesh.triangles[region_faces]
    edge_lengths = []
    for tri in triangles:
        e1 = np.linalg.norm(tri[1] - tri[0])
        e2 = np.linalg.norm(tri[2] - tri[1])
        e3 = np.linalg.norm(tri[0] - tri[2])
        edge_lengths.extend([e1, e2, e3])
    edge_lengths = np.array(edge_lengths)

    # Aspect ratios (longest/shortest edge)
    aspect_ratios = []
    for tri in triangles:
        e1 = np.linalg.norm(tri[1] - tri[0])
        e2 = np.linalg.norm(tri[2] - tri[1])
        e3 = np.linalg.norm(tri[0] - tri[2])
        edges = sorted([e1, e2, e3])
        if edges[0] > 0.001:
            aspect_ratios.append(edges[2] / edges[0])
        else:
            aspect_ratios.append(float('inf'))
    aspect_ratios = np.array(aspect_ratios)

    # Normal consistency (how aligned are normals?)
    mean_normal = face_normals.mean(axis=0)
    mean_normal = mean_normal / np.linalg.norm(mean_normal)
    normal_dots = np.abs(np.dot(face_normals, mean_normal))

    # Up-facing normals (Z component > 0.5)
    up_facing = np.sum(face_normals[:, 2] > 0.5) / len(face_normals)

    # Count "internal edges" - edges shared by two triangles with similar normals
    # This is an approximation - true internal edges would require adjacency analysis

    return {
        "n_faces": len(region_faces),
        "total_area": np.sum(face_areas),
        "mean_area": np.mean(face_areas),
        "min_area": np.min(face_areas),
        "max_area": np.max(face_areas),
        "tiny_faces_pct": np.sum(face_areas < 0.1) / len(face_areas) * 100,
        "mean_edge": np.mean(edge_lengths),
        "min_edge": np.min(edge_lengths),
        "short_edges_pct": np.sum(edge_lengths < 0.5) / len(edge_lengths) * 100,
        "mean_aspect": np.mean(aspect_ratios[np.isfinite(aspect_ratios)]),
        "high_aspect_pct": np.sum(aspect_ratios > 5) / len(aspect_ratios) * 100,
        "normal_consistency": np.mean(normal_dots),
        "up_facing_pct": up_facing * 100,
        "mean_z_normal": np.mean(face_normals[:, 2]),
    }


def analyze_edge_sharing(mesh, x_pos, y_pos=0, region_radius=20):
    """Analyze edge sharing patterns in contact region."""

    face_centroids = mesh.triangles_center
    z_threshold = mesh.bounds[1][2] - 10
    top_mask = face_centroids[:, 2] > z_threshold

    xy_distances = np.sqrt(
        (face_centroids[:, 0] - x_pos)**2 +
        (face_centroids[:, 1] - y_pos)**2
    )
    region_mask = xy_distances < region_radius
    combined_mask = top_mask & region_mask
    region_face_indices = np.where(combined_mask)[0]

    if len(region_face_indices) == 0:
        return None

    # Build edge-to-face mapping for region faces
    edge_faces = {}  # edge tuple -> list of face indices

    for fi in region_face_indices:
        face = mesh.faces[fi]
        # Sort vertex indices to create canonical edge representation
        edges = [
            tuple(sorted([face[0], face[1]])),
            tuple(sorted([face[1], face[2]])),
            tuple(sorted([face[2], face[0]])),
        ]
        for edge in edges:
            if edge not in edge_faces:
                edge_faces[edge] = []
            edge_faces[edge].append(fi)

    # Count edges by sharing
    boundary_edges = 0  # 1 face
    internal_edges = 0  # 2 faces
    complex_edges = 0   # 3+ faces (non-manifold)

    # For internal edges, compute angle between adjacent face normals
    internal_angles = []

    for edge, faces in edge_faces.items():
        if len(faces) == 1:
            boundary_edges += 1
        elif len(faces) == 2:
            internal_edges += 1
            # Compute angle between normals
            n1 = mesh.face_normals[faces[0]]
            n2 = mesh.face_normals[faces[1]]
            dot = np.clip(np.dot(n1, n2), -1, 1)
            angle = np.degrees(np.arccos(dot))
            internal_angles.append(angle)
        else:
            complex_edges += 1

    internal_angles = np.array(internal_angles) if internal_angles else np.array([0])

    return {
        "total_edges": len(edge_faces),
        "boundary_edges": boundary_edges,
        "internal_edges": internal_edges,
        "complex_edges": complex_edges,
        "boundary_pct": boundary_edges / len(edge_faces) * 100,
        "internal_pct": internal_edges / len(edge_faces) * 100,
        "mean_internal_angle": np.mean(internal_angles),
        "max_internal_angle": np.max(internal_angles),
        "sharp_edges_pct": np.sum(internal_angles > 30) / len(internal_angles) * 100 if len(internal_angles) > 0 else 0,
        "very_sharp_pct": np.sum(internal_angles > 60) / len(internal_angles) * 100 if len(internal_angles) > 0 else 0,
    }


def main():
    print("\nAnalyzing triangle density and edge patterns at failing vs working positions\n")

    tibia_full = trimesh.load("/tmp/repaired/CASE4_CroppedTibiaBone-repaired.stl")
    tibia_hull = tibia_full.convex_hull

    # Normalize
    tibia_hull.apply_translation(-tibia_hull.centroid)
    tibia_hull.apply_translation([0, 0, -tibia_hull.bounds[1][2]])

    print(f"Tibia hull: {len(tibia_hull.faces)} faces, {len(tibia_hull.vertices)} vertices")
    print(f"Bounds: X=[{tibia_hull.bounds[0][0]:.1f}, {tibia_hull.bounds[1][0]:.1f}]")
    print(f"        Y=[{tibia_hull.bounds[0][1]:.1f}, {tibia_hull.bounds[1][1]:.1f}]")
    print(f"        Z=[{tibia_hull.bounds[0][2]:.1f}, {tibia_hull.bounds[1][2]:.1f}]")

    # Test positions
    positions = [
        (0, 0, "FAIL"),
        (10, 0, "FAIL"),
        (15, 0, "PASS"),
        (-15, 0, "FAIL"),  # From grid test
        (0, 3, "PASS"),    # From grid test
    ]

    print(f"\n{'='*80}")
    print("TRIANGLE DENSITY ANALYSIS (top 10mm, radius 20mm)")
    print(f"{'='*80}")

    density_results = []
    for x, y, expected in positions:
        stats = analyze_contact_region(tibia_hull, x, y)
        if stats:
            density_results.append((x, y, expected, stats))
            print(f"\nPosition ({x}, {y}) - Expected: {expected}")
            print(f"  Faces in region: {stats['n_faces']}")
            print(f"  Mean face area: {stats['mean_area']:.4f} mm²")
            print(f"  Tiny faces (<0.1mm²): {stats['tiny_faces_pct']:.1f}%")
            print(f"  Short edges (<0.5mm): {stats['short_edges_pct']:.1f}%")
            print(f"  High aspect ratio (>5): {stats['high_aspect_pct']:.1f}%")
            print(f"  Normal consistency: {stats['normal_consistency']:.3f}")
            print(f"  Up-facing normals: {stats['up_facing_pct']:.1f}%")

    print(f"\n{'='*80}")
    print("EDGE SHARING ANALYSIS")
    print(f"{'='*80}")

    edge_results = []
    for x, y, expected in positions:
        stats = analyze_edge_sharing(tibia_hull, x, y)
        if stats:
            edge_results.append((x, y, expected, stats))
            print(f"\nPosition ({x}, {y}) - Expected: {expected}")
            print(f"  Total edges: {stats['total_edges']}")
            print(f"  Boundary edges: {stats['boundary_pct']:.1f}%")
            print(f"  Internal edges: {stats['internal_pct']:.1f}%")
            print(f"  Mean internal angle: {stats['mean_internal_angle']:.1f}°")
            print(f"  Max internal angle: {stats['max_internal_angle']:.1f}°")
            print(f"  Sharp edges (>30°): {stats['sharp_edges_pct']:.1f}%")
            print(f"  Very sharp (>60°): {stats['very_sharp_pct']:.1f}%")

    # Comparison table
    print(f"\n{'='*80}")
    print("COMPARISON: FAILING vs WORKING POSITIONS")
    print(f"{'='*80}")

    fail_positions = [(x, y, s) for x, y, e, s in density_results if e == "FAIL"]
    pass_positions = [(x, y, s) for x, y, e, s in density_results if e == "PASS"]

    if fail_positions and pass_positions:
        fail_stats = {k: np.mean([s[k] for _, _, s in fail_positions]) for k in fail_positions[0][2].keys()}
        pass_stats = {k: np.mean([s[k] for _, _, s in pass_positions]) for k in pass_positions[0][2].keys()}

        print(f"\n{'Metric':<25} {'FAIL (avg)':<15} {'PASS (avg)':<15} {'Difference':<15}")
        print("-" * 70)
        for key in ['n_faces', 'mean_area', 'tiny_faces_pct', 'short_edges_pct',
                    'high_aspect_pct', 'normal_consistency', 'up_facing_pct']:
            f_val = fail_stats[key]
            p_val = pass_stats[key]
            diff = f_val - p_val
            diff_str = f"{diff:+.2f}" if abs(diff) > 0.01 else "~0"
            print(f"{key:<25} {f_val:<15.2f} {p_val:<15.2f} {diff_str:<15}")

    # Edge analysis comparison
    fail_edges = [(x, y, s) for x, y, e, s in edge_results if e == "FAIL"]
    pass_edges = [(x, y, s) for x, y, e, s in edge_results if e == "PASS"]

    if fail_edges and pass_edges:
        fail_e_stats = {k: np.mean([s[k] for _, _, s in fail_edges]) for k in fail_edges[0][2].keys()}
        pass_e_stats = {k: np.mean([s[k] for _, _, s in pass_edges]) for k in pass_edges[0][2].keys()}

        print(f"\n{'Edge Metric':<25} {'FAIL (avg)':<15} {'PASS (avg)':<15} {'Difference':<15}")
        print("-" * 70)
        for key in ['internal_pct', 'mean_internal_angle', 'sharp_edges_pct', 'very_sharp_pct']:
            f_val = fail_e_stats[key]
            p_val = pass_e_stats[key]
            diff = f_val - p_val
            diff_str = f"{diff:+.2f}" if abs(diff) > 0.01 else "~0"
            print(f"{key:<25} {f_val:<15.2f} {p_val:<15.2f} {diff_str:<15}")

    print(f"\n{'='*80}")
    print("CONCLUSIONS")
    print(f"{'='*80}")
    print("""
Based on the analysis:
- If FAIL positions have more tiny faces/short edges: geometry quality matters
- If FAIL positions have more sharp internal edges: ghost collision hypothesis supported
- If no significant difference: issue is likely numerical/algorithmic, not geometric
""")

    print(f"\nTest H-A1 - Run completed: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")


if __name__ == "__main__":
    main()
