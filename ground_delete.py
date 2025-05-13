import open3d as o3d
import numpy as np
import os

def remove_z_plane(pcd, z_min, z_max):
    """
    Remove ground or unwanted Z-range from a point cloud.

    Args:
        pcd (o3d.geometry.PointCloud): Input point cloud.
        z_min (float): Minimum Z value to keep.
        z_max (float): Maximum Z value to keep.

    Returns:
        o3d.geometry.PointCloud: Filtered point cloud with Z in (z_min, z_max).
    """
    points = np.asarray(pcd.points)
    colors = np.asarray(pcd.colors)

    mask = np.logical_or(points[:, 2] < z_min, points[:, 2] > z_max)
    filtered_points = points[mask]
    filtered_colors = colors[mask]

    new_pcd = o3d.geometry.PointCloud()
    new_pcd.points = o3d.utility.Vector3dVector(filtered_points)
    new_pcd.colors = o3d.utility.Vector3dVector(filtered_colors)

    return new_pcd

def main():
    # ==== Set file path and Z threshold ====
    ply_path = r" "
    z_min = 0.5  # Minimum height to keep, based on your ground stats
    z_max = 5.0  # Max height (optional)

    if not os.path.exists(ply_path):
        print(f"File not found: {ply_path}")
        return

    # ==== Load original point cloud ====
    pcd = o3d.io.read_point_cloud(ply_path)
    print(f"Loaded point cloud with {len(pcd.points)} points")

    # ==== Remove ground points ====
    filtered_pcd = remove_z_plane(pcd, z_min, z_max)
    print(f"Filtered point cloud has {len(filtered_pcd.points)} points")

    # ==== Save the filtered point cloud ====
    base, ext = os.path.splitext(ply_path)
    output_ply_path = f"{base}_noground{ext}"
    output_png_path = f"{base}_noground.png"
    o3d.io.write_point_cloud(output_ply_path, filtered_pcd)
    print(f"Saved filtered .ply to: {output_ply_path}")

    # ==== Display window with S key for screenshot ====
    vis = o3d.visualization.VisualizerWithKeyCallback()
    vis.create_window(window_name="Ground Removed (Press S to Save PNG)")
    vis.add_geometry(filtered_pcd)

    def save_screenshot(vis_obj):
        vis_obj.capture_screen_image(output_png_path)
        return False  # Keep window open

    vis.register_key_callback(ord("S"), save_screenshot)
    vis.run()
    vis.destroy_window()

if __name__ == "__main__":
    main()
