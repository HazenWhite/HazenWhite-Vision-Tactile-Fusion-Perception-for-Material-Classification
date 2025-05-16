# Vision-Tactile Fusion Perception for Material Classification of Deformed Sidewalk Litter via Geometry-Adaptive Contact
This repository contains the complete codebase for a multimodal robotic perception framework designed to classify deformed street waste using 3D vision and capacitive tactile sensing.
The system integrates YOLOv8 for object detection, a UR5 robotic arm for contact alignment, and XGBoost for material recognition based on multimodal data.  

  
  
This code is used to verify the results of our paper:

***"Vision-Tactile Fusion Perception for Material Classification of Deformed Sidewalk Litter via Geometry-Adaptive Contact"***

## Repository Structure 
- ### **ultralytics-main/**:
Contains the official implementation of YOLOv8, used for real-time object detection and classification.  
- ### **move_arm.py**:
Execute robot arm fitting according to the obtained target coordinates and normal vector.  
- ### **XGBoost.py**
Code for training an XGBoost classifier based on the collected tactile sensor data.  
- ### **XGBoost_test.py**
Applies and evaluates the trained tactile classification model on new data.  
- ### **Multimodality.py**
Performs Bayesian fusion based on confusion-matrix-informed probabilities to combine predictions from vision and tactile sources.
- ### **detect_get_cloud.py**
Call the camera to obtain the target data point cloud and the current robot arm angle status;
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

## Usage Instructions

This repository provides a complete multimodal material classification pipeline integrating vision and tactile sensing for deformed sidewalk waste. The following steps guide you through the usage of the system:

### 1. Object Detection and Point Cloud Extraction  
Run `detect_get_cloud.py`  
- **Function**:  
  Utilizes an Intel RealSense D435i depth camera and a trained YOLOv8 model to detect objects in real time and extract the corresponding RGB-D point cloud.  
- **Output**:  
  Saves point clouds and annotated images for detected waste objects.

---

### 2. Point Cloud Filtering  
Run `cloud_filter.py`  
- **Function**:  
  Applies voxel grid downsampling and connected-component filtering to reduce noise and isolate the object geometry.  
- **Output**:  
  A cleaner, denser object-level point cloud suitable for downstream processing.

---

### 3. Ground Plane Removal  
Run `ground_delete.py`  
- **Function**:  
  Removes planar ground components from the filtered point cloud using RANSAC-based segmentation.  
- **Output**:  
  A ground-free object point cloud highlighting surface geometry.

---

### 4. Planar Surface Extraction and Normal Estimation  
Run `cloud_PCA_cluster.py`  
- **Function**:  
  Performs low-curvature PCA-based clustering on the object point cloud to detect locally planar surfaces. Computes centroid (`P_cam`) and surface normal vector (`normal`) of the optimal contact region.  
- **Output**:  
  Saves planar surface coordinates and normal vectors for tactile sensor alignment.

---

### 5. Robotic Arm Alignment  
Edit and Run `move_arm.py`  
- **Instructions**:  
  Replace `P_cam` and `normals` parameters in the script with the output from the previous step.  
- **Function**:  
  Moves the UR5 robotic arm to ensure full-surface contact between the tactile sensor and the identified planar region.

---

### 6. Tactile Data Classification  
Run `XGBoost_test.py`  
- **Function**:  
  Triggers capacitive tactile data acquisition once contact is established. Uses a pre-trained XGBoost model to classify material properties based on 23-dimensional feature vectors.

---

### 7. Multimodal Fusion Prediction  
Run `Multimodality.py`  
- **Function**:  
  Combines tactile and vision-based predictions using a Bayesian conditional probability fusion strategy.  
- **Output**:  
  Final material classification result with improved robustness and accuracy.

---

>  **Note**: Please ensure the RealSense SDK, YOLOv8 dependencies, and RTDE interface for UR5 are properly configured before running the system.


