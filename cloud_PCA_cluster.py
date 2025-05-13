import open3d as o3d
import numpy as np
from scipy.spatial import KDTree, ConvexHull
import matplotlib.pyplot as plt

# ======================
# This script is used to extract and validate fitted planes from a point cloud
# ======================

# ======================
# Global parameter configuration
# ======================
GLOBAL_SETTINGS = {
    # Low curvature filtering
    'curvature': {
        'k_neighbors': 100,   # Number of neighbors for normal estimation
        'threshold': 0.007,   # Curvature threshold (below this is considered planar candidate)
        'min_cluster': 50     # Minimum number of candidate points
    },

    # Clustering parameters
    'clustering': {
        'eps': 0.031,         # Euclidean clustering radius (in meters)
        'min_points': 50,     # Minimum points per cluster
        'max_clusters': 10    # Max clusters to display
    },

    # Visualization settings
    'visual': {
        'low_curv_color': [1, 0, 0],               # Low curvature region color
        'cluster_colors': plt.cm.tab20.colors,     # Colormap for clusters
        'other_color': [0.7, 0.7, 0.7]              # Color for non-planar points
    }
}

# ======================
# Low curvature filtering with normal direction unification
# ======================
def filter_low_curvature(pcd, settings):
    points = np.asarray(pcd.points)
    kdtree = KDTree(points)
    curvatures = []
    normals = []

    for i in range(len(points)):
        _, idx = kdtree.query(points[i], k=settings['k_neighbors'])
        neighbors = points[idx]
        centroid = np.mean(neighbors, axis=0)
        cov = np.cov((neighbors - centroid).T)
        eig_val, eig_vec = np.linalg.eigh(cov)

        curvatures.append(eig_val[0] / np.sum(eig_val))
        normals.append(eig_vec[:, 0])  # Smallest eigenvalue → normal vector

    # Unify normal direction (assuming viewpoint at origin)
    normals = np.array(normals)
    view_vector = -points
    dot_product = np.sum(normals * view_vector, axis=1)
    normals[dot_product < 0] *= -1

    curvatures = np.array(curvatures)

    candidate_idx = np.where(curvatures < settings['threshold'])[0]
    print(f"Low curvature candidates: {len(candidate_idx)} (required > {settings['min_cluster']})")

    return candidate_idx, normals

# ======================
# Clustering and plane validation
# ======================
def cluster_and_validate(pcd, candidate_idx, normals, settings):
    low_curv_pcd = pcd.select_by_index(candidate_idx)
    points = np.asarray(low_curv_pcd.points)

    labels = np.array(
        low_curv_pcd.cluster_dbscan(
            eps=settings['eps'],
            min_points=settings['min_points'],
            print_progress=True
        )
    )

    valid_labels = labels[labels != -1]
    unique_labels = np.unique(valid_labels)
    print(f"Found {len(unique_labels)} candidate clusters")

    planes = []
    colors = np.zeros((len(points), 3))
    color_table = list(GLOBAL_SETTINGS['visual']['cluster_colors'])
    color_table[0] = (1.0, 0.5, 0.0)  # Orange
    color_table[1] = (0.0, 0.0, 0.6)  # Dark blue

    for i, cluster_id in enumerate(unique_labels[:settings['max_clusters']]):
        if cluster_id == -1:
            continue

        cluster_mask = (labels == cluster_id)
        cluster_points = points[cluster_mask]
        cluster_normals = normals[candidate_idx][cluster_mask]

        cluster_color = color_table[i % len(color_table)]
        colors[cluster_mask] = cluster_color

        if validate_plane(cluster_points, cluster_normals):
            planes.append({
                'points': cluster_points,
                'center': np.mean(cluster_points, axis=0),
                'normal': np.mean(cluster_normals, axis=0),
                'area': calculate_plane_area(cluster_points),
                'color': cluster_color
            })

    low_curv_pcd.colors = o3d.utility.Vector3dVector(colors)
    return low_curv_pcd, planes

# ======================
# Plane validation criteria
# ======================
def validate_plane(points, normals, angle_thresh=50, density_thresh=0.01):
    mean_normal = np.mean(normals, axis=0)
    angles = np.degrees(np.arccos(np.abs(np.dot(normals, mean_normal))))
    if np.mean(angles) > angle_thresh:
        return False

    dist_matrix = np.sqrt(np.sum((points[:, None] - points) ** 2, axis=2))
    np.fill_diagonal(dist_matrix, np.inf)
    min_dists = np.min(dist_matrix, axis=1)
    if np.mean(min_dists) > density_thresh:
        return False

    return True

# ======================
# Plane area calculation (optimized)
# ======================
def calculate_plane_area(points):
    try:
        cov = np.cov(points.T)
        _, eig_vec = np.linalg.eigh(cov)
        projected = points @ eig_vec[:, :2]
        hull = ConvexHull(projected)
        return hull.volume
    except:
        return len(points) * 0.0001

def draw_normal_vectors(planes, scale=0.05):
    lines = []
    points = []
    colors = []

    for plane in planes:
        center = plane['center']
        normal = plane['normal']
        color = plane['color']

        endpoint = center + normal * scale
        idx = len(points)
        points.append(center)
        points.append(endpoint)
        lines.append([idx, idx + 1])
        colors.append(color)

    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(np.array(points))
    line_set.lines = o3d.utility.Vector2iVector(np.array(lines))
    line_set.colors = o3d.utility.Vector3dVector(np.array(colors))

    return line_set


if __name__ == "__main__":
    ply_file = " "
    screenshot_path = ply_file.replace(".ply", ".png")

    def capture_screenshot(vis):
        vis.capture_screen_image(screenshot_path)
        print(f"Screenshot saved: {screenshot_path}")
        return False

    print("Adjust view and press S to capture a screenshot")
    key_to_callback = {}
    key_to_callback[ord("S")] = capture_screenshot

    pcd = o3d.io.read_point_cloud(ply_file)
    points = np.asarray(pcd.points)
    o3d.visualization.draw_geometries_with_key_callbacks([pcd], key_to_callback)

    candidate_idx, normals = filter_low_curvature(pcd, GLOBAL_SETTINGS['curvature'])

    if len(candidate_idx) < GLOBAL_SETTINGS['curvature']['min_cluster']:
        raise ValueError("Too few candidate points. Try lowering the curvature threshold.")

    cluster_pcd, planes = cluster_and_validate(
        pcd,
        candidate_idx,
        normals,
        GLOBAL_SETTINGS['clustering']
    )

    other_pcd = pcd.select_by_index(candidate_idx, invert=True)
    other_pcd.paint_uniform_color(GLOBAL_SETTINGS['visual']['other_color'])

    print(f"\n== {len(planes)} valid planes detected ==")
    for i, plane in enumerate(planes):
        plane_color = np.round(np.array(plane['color']) * 255)
        print(f"Plane {i + 1}:")
        print(f"  Color (RGB): {plane_color}")
        print(f"  Center (mm): {np.round(plane['center'] * 1000, 2)}")
        print(f"  Normal: {np.round(plane['normal'], 4)}")
        print(f"  Approx. Area: {plane['area']:.4f} m² ({plane['area'] * 1e6:.0f} cm²)")

    line_set = draw_normal_vectors(planes)

    screenshot_filename = " "
    vis = o3d.visualization.VisualizerWithKeyCallback()
    vis.create_window(window_name="Planes with Normals (Press S to Screenshot)", width=1024, height=768)

    vis.add_geometry(cluster_pcd)
    vis.add_geometry(other_pcd)
    vis.add_geometry(line_set)

    def save_screenshot(vis_obj):
        vis_obj.capture_screen_image(screenshot_filename)
        return False

    def save_full_pointcloud_with_normals(vis_obj):
        save_path = " "
        line_points = np.asarray(line_set.points)
        line_colors = np.ones((line_points.shape[0], 3))  # white

        all_points = np.vstack((
            np.asarray(cluster_pcd.points),
            np.asarray(other_pcd.points),
            line_points
        ))
        all_colors = np.vstack((
            np.asarray(cluster_pcd.colors),
            np.asarray(other_pcd.colors),
            line_colors
        ))

        merged = o3d.geometry.PointCloud()
        merged.points = o3d.utility.Vector3dVector(all_points)
        merged.colors = o3d.utility.Vector3dVector(all_colors)

        o3d.io.write_point_cloud(save_path, merged)
        return False

    vis.register_key_callback(ord("P"), save_full_pointcloud_with_normals)
    vis.register_key_callback(ord("S"), save_screenshot)
    vis.run()
    vis.destroy_window()
