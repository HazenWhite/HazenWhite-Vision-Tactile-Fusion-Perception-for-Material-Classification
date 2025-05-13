This code is used to verify the paper Vision-Tactile Fusion Perception for Material Classification of Deformed Sidewalk Litter via Geometry-Adaptive Contact
This repository contains the complete codebase for a multimodal robotic perception framework designed to classify deformed street waste using 3D vision and capacitive tactile sensing. The system integrates YOLOv8 for object detection, a UR5 robotic arm for contact alignment, and XGBoost for material recognition based on multimodal data.
.
├── lib/ch341/               # CH341 driver library for sensor communication
├── ultralytics-main/        # YOLOv8 source code
├── XGBoost.py               # Tactile classification using XGBoost
├── XGBoost_test.py          # Evaluation of trained XGBoost model
├── Multimodality.py         # Bayesian fusion of visual and tactile predictions
├── cap_read.py              # Sensor data acquisition script
├── class_ch341.py           # CH341 communication driver
├── class_finger.py          # Finger sensor module class
├── class_sensorcmd.py       # Sensor command format
├── sensorPara.py            # Sensor parameter configurations
├── cloud_PCA_cluster.py     # Low-curvature PCA clustering for plane fitting
├── cloud_check.py           # Normal vector and proximity validation
├── cloud_filter.py          # Point cloud voxel + connected domain filtering
├── cloud_move_arm.py        # Motion planning for UR5 to align to surface
├── ground_delete.py         # Ground plane removal from point cloud
├── train.py                 # YOLOv8 model training script
├── .gitignore
└── README.md
