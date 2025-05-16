import pyrealsense2 as rs
import numpy as np
import cv2
from ultralytics import YOLO
import open3d as o3d
import os
import socket
import time
from rtde_receive import RTDEReceiveInterface
from cloud_move_arm import *
import numpy as np
import cv2

HOST = "169.254.87.23"
PORT = 30002

try:
    # Connect to the robotic arm
    rtde_receive = RTDEReceiveInterface(HOST)
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((HOST, PORT))
    print("Successfully connected to the robot arm.")

except Exception as e:
    print(f"Error occurred: {e}")

finally:
    # Release resources
    sock.close()
    print("Program terminated.")

# ====== Load YOLOv8 model ======
yolo_model_path = " "
model = YOLO(yolo_model_path)

# ====== Target classes ======
target_classes = ["Foam", "Metal", "Paper", "Plastic Bag", "Plastic Bottle", "Stone", "Wood"]

# ====== Initialize RealSense camera ======
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
profile = pipeline.start(config)

align = rs.align(rs.stream.color)

intrinsics = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
print("Camera started. Press 'q' to quit manually.")

try:
    while True:
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        if not depth_frame or not color_frame:
            continue

        color_image = np.asanyarray(color_frame.get_data())

        results = model(color_image, conf=0.8, verbose=False)
        annotated_frame = results[0].plot()

        cv2.imshow('RealSense YOLOv8 Detection', annotated_frame)
        cv2.waitKey(1)

        detected_classes = [results[0].names[int(cls)] for cls in results[0].boxes.cls]
        boxes = results[0].boxes.xyxy.cpu().numpy().astype(int)

        # Check if any of the target classes are detected
        valid_indices = [i for i, cls in enumerate(detected_classes) if cls in target_classes]

        if valid_indices:
            detected_targets = [detected_classes[i] for i in valid_indices]
            print(f"⚠️ Detected classes: {set(detected_targets)}")
            user_input = input("Save detected object's point cloud? (Yes/No): ").strip().lower()

            if user_input == "yes":
                intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics

                save_dir = " "  # Your save directory
                image_save_dir = os.path.join(save_dir, "images")  # Directory for saving images
                os.makedirs(save_dir, exist_ok=True)
                os.makedirs(image_save_dir, exist_ok=True)

                # === Save the current frame (with detection boxes) ===
                timestamp = time.strftime("%Y%m%d_%H%M%S")
                image_filename = f"frame_{timestamp}.png"
                image_filepath = os.path.join(image_save_dir, image_filename)
                cv2.imwrite(image_filepath, annotated_frame)
                print(f"📸 Saved annotated image: {image_filepath}")

                for idx in valid_indices:
                    class_name = detected_classes[idx]
                    xmin, ymin, xmax, ymax = boxes[idx]
                    xmin, ymin = max(xmin, 0), max(ymin, 0)
                    xmax, ymax = min(xmax, color_image.shape[1] - 1), min(ymax, color_image.shape[0] - 1)

                    selected_points = []
                    selected_colors = []

                    for y in range(ymin, ymax):
                        for x in range(xmin, xmax):
                            depth = depth_frame.get_distance(x, y)
                            if depth > 0:
                                point = rs.rs2_deproject_pixel_to_point(intrinsics, [x, y], depth)
                                b, g, r = color_image[y, x]
                                selected_points.append(point)
                                selected_colors.append([r / 255.0, g / 255.0, b / 255.0])

                    if selected_points:
                        pcd = o3d.geometry.PointCloud()
                        pcd.points = o3d.utility.Vector3dVector(np.array(selected_points))
                        pcd.colors = o3d.utility.Vector3dVector(np.array(selected_colors))

                        # ===== Auto-increment naming =====
                        existing = [f for f in os.listdir(save_dir) if f.startswith(class_name) and f.endswith('.ply')]
                        file_index = len(existing) + 1
                        filename = f"{class_name}_{file_index}.ply"
                        filepath = os.path.join(save_dir, filename)

                        o3d.io.write_point_cloud(filepath, pcd)
                        print(f"Saved point cloud: {filepath}")
                        joint_angles = get_joint_angles(rtde_receive)
                        print(f"Joint angles: {joint_angles}")
                    else:
                        print(f"No valid 3D points found in box for {class_name}.")

                break  # Exit after saving
            else:
                print("Skipped saving. Continuing detection...")

        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("Manual exit triggered by user.")
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
    print("Camera stopped and resources released.")
