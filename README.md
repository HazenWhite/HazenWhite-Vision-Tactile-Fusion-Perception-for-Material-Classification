# Vision-Tactile Fusion Perception for Material Classification of Deformed Sidewalk Litter via Geometry-Adaptive Contact
This repository contains the complete codebase for a multimodal robotic perception framework designed to classify deformed street waste using 3D vision and capacitive tactile sensing.
The system integrates YOLOv8 for object detection, a UR5 robotic arm for contact alignment, and XGBoost for material recognition based on multimodal data.  

  
  
This code is used to verify the results of our paper:

***"Vision-Tactile Fusion Perception for Material Classification of Deformed Sidewalk Litter via Geometry-Adaptive Contact"***

## Repository Structure  
.  
├── lib/ch341/　　　　　　　　　　# CH341 driver library for sensor communication  

├── ultralytics-main/　　　　　　　# YOLOv8 source code;  

├── XGBoost.py　　　　　　　　　　# Tactile classification using XGBoost  

├── XGBoost_test.py　　　　　　　# Evaluation of trained XGBoost model  

├── Multimodality.py　　　　　　　# Bayesian fusion of visual and tactile predictions  

├── cap_read.py　　　　　　　　　　# Sensor data acquisition script  

├── class_ch341.py　　　　　　　　# CH341 communication driver  

├── class_finger.py　　　　　　　# Finger sensor module class  

├── class_sensorcmd.py　　　　　　# Sensor command format  

├── sensorPara.py　　　　　　　　# Sensor parameter configurations  

├── cloud_PCA_cluster.py　　　　# Low-curvature PCA clustering for plane fitting  

├── cloud_check.py　　　　　　　　# Normal vector and proximity validation  

├── cloud_filter.py　　　　　　　# Point cloud voxel + connected domain filtering  

├── cloud_move_arm.py　　　　　　# Motion planning for UR5 to align to surface  

├── ground_delete.py　　　　　　# Ground plane removal from point cloud  

├── train.py　　　　　　　　　　　# YOLOv8 model training script  

├── .gitignore  

└── README.md  

### Key Components

- ### **ultralytics-main/**:
Contains the official implementation of YOLOv8, used for real-time object detection and classification.  
- ### **XGBoost.py**
Code for training an XGBoost classifier based on the collected tactile sensor data.  
- ### **XGBoost_test.py**
Applies and evaluates the trained tactile classification model on new data.  
- ### **Multimodality.py**
Performs Bayesian fusion based on confusion-matrix-informed probabilities to combine predictions from vision and tactile sources.
- ### **cap_read.py**, **class_ch341.py**, **class_finger.py**, **class_sensorcmd.py**, **sensorPara.py**
A suite of scripts for sensor initialization, communication, and real-time data collection.    
- ### **cloud_PCA_cluster.py**
Executes low-curvature clustering and PCA-based planar surface fitting on filtered point cloud data.  
- ### **cloud_check.py**
Provides a visualization tool to inspect the processed point cloud.  
- ### **cloud_filter.py**
Applies voxel-based and connected-domain filtering to reduce noise and background clutter in point cloud.  
- ### **ground_delete.py**
Removes the ground plane from the scene to isolate object geometry.  
- ### **cloud_move_arm.py**
Contains calibration parameters and functions to move the UR5 robotic arm to the selected surface for stable contact.  
- ### **train.py**
Training script for the YOLOv8 visual recognition model.  
