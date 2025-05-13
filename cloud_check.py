import open3d as o3d
import numpy as np

# ======================
# This script is used to inspect a point cloud and capture a screenshot when the S key is pressed.
# ======================

# Load point cloud
ply_file = "exp2.ply"
pcd = o3d.io.read_point_cloud(ply_file)
print(f"Total points: {len(pcd.points)}")
points = np.asarray(pcd.points)
avg_distance = np.mean(np.linalg.norm(points[1:] - points[:-1], axis=1))
print(f"Average point-to-point distance: {avg_distance:.4f} meters")

# Screenshot save path, automatically matches the .ply file name
screenshot_path = ply_file.replace(".ply", ".png")

# Define keyboard callback function
def capture_screenshot(vis):
    vis.capture_screen_image(screenshot_path)
    print(f"📸 Screenshot saved: {screenshot_path}")
    return False  # Do not close the visualization window

# Show point cloud and register the S key for screenshot
print("Adjust the view and press the S key to take a screenshot")
key_to_callback = {}
key_to_callback[ord("S")] = capture_screenshot

o3d.visualization.draw_geometries_with_key_callbacks([pcd], key_to_callback)
