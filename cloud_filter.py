import open3d as o3d
import numpy as np
from scipy.spatial import KDTree

def process_point_cloud(input_path,
                        voxel_size=0.005,
                        min_cluster_size=50,     # Minimum number of points in a cluster
                        cluster_eps=0.01,        # Radius for neighborhood (in meters)
                        show_original=False):
    """
    Point cloud processing: denoising, downsampling, and filtering by connected components.

    Parameters:
    - input_path (str): Path to input .ply point cloud file
    - voxel_size (float): Voxel downsampling size in meters
    - min_cluster_size (int): Minimum number of points to keep a cluster
    - cluster_eps (float): Radius for DBSCAN clustering
    - show_original (bool): Whether to show the original point cloud for comparison

    Returns:
    - processed_pcd (open3d.geometry.PointCloud): Processed point cloud
    """
    try:
        # **1. Load point cloud**
        pcd = o3d.io.read_point_cloud(input_path)
        if not pcd.has_points():
            raise ValueError("Point cloud contains no points!")

        print(f"Original point count: {len(pcd.points)}")

        # **2. Set all colors to black**
        n_points = len(pcd.points)
        black_colors = np.zeros((n_points, 3), dtype=np.float64)
        pcd.colors = o3d.utility.Vector3dVector(black_colors)

        # **3. Voxel downsampling**
        resampled_pcd = pcd.voxel_down_sample(voxel_size)
        print(f"Point count after voxel downsampling: {len(resampled_pcd.points)}")
        o3d.visualization.draw_geometries([resampled_pcd], window_name="After Voxel Downsampling")

        # **4. Euclidean clustering using DBSCAN**
        labels = np.array(resampled_pcd.cluster_dbscan(eps=cluster_eps, min_points=min_cluster_size))

        # Count valid clusters (exclude noise label -1)
        unique_labels = np.unique(labels)
        unique_labels = unique_labels[unique_labels != -1]
        print(f"Detected {len(unique_labels)} valid connected components")

        # **5. Filter out small clusters**
        valid_idx = labels != -1
        clustered_pcd = resampled_pcd.select_by_index(np.where(valid_idx)[0])

        print(f"Point count after cluster filtering: {len(clustered_pcd.points)}")

        # **6. Visualization**
        clustered_pcd.paint_uniform_color([0, 1, 0])  # Green

        if show_original:
            # Show original in gray for comparison
            pcd.paint_uniform_color([0.5, 0.5, 0.5])
            o3d.visualization.draw_geometries([pcd, clustered_pcd],
                                              window_name="Before and After Comparison")
        else:
            o3d.visualization.draw_geometries([clustered_pcd],
                                              window_name="Processing Result")

        return clustered_pcd

    except Exception as e:
        print(f"Error during processing: {str(e)}")
        return None

if __name__ == "__main__":
    input_file = " "

    processed_pcd = process_point_cloud(
        input_path=input_file,
        voxel_size=0.005,
        min_cluster_size=300,  # Minimum points per cluster
        cluster_eps=0.05,      # Clustering radius in meters
        show_original=False    # Show original cloud for comparison
    )

    # **Save result**
    if processed_pcd:
        o3d.io.write_point_cloud(" ", processed_pcd)
