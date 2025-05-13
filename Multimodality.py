import os
import glob
import random
import numpy as np
import joblib
import xgboost as xgb
from ultralytics import YOLO
import pandas as pd

# Load confusion matrix-based conditional probability matrices
P_yolo = np.loadtxt("cm_yolo.csv", delimiter="\t")
P_xgb = np.loadtxt("cm_xgb.csv", delimiter="\t")


def predict_with_conditional_fusion(p_yolo, p_xgb, P_yolo, P_xgb):
    """
    Perform conditional probability fusion based on Bayes' total probability theorem.

    Args:
        p_yolo (np.array): Output probabilities from YOLO (shape: [n_class])
        p_xgb (np.array): Output probabilities from XGBoost (shape: [n_class])
        P_yolo (np.array): Conditional probability matrix for YOLO
        P_xgb (np.array): Conditional probability matrix for XGBoost

    Returns:
        np.array: Fused probability distribution
    """
    n_class = len(p_yolo)
    combined_prob = np.zeros(n_class)

    for r in range(n_class):
        combined_prob += (p_yolo[r] * P_yolo[r]) + (p_xgb[r] * P_xgb[r])

    combined_prob /= combined_prob.sum()
    return combined_prob


# ====== Load Models ======
# YOLOv8 model (image classification)
yolo_model = YOLO("E:\yolov8_310\exp_final2\weights/best.pt")
# XGBoost model (capacitive classification)
xgb_model = xgb.Booster()
xgb_model.load_model("E:\pythonProject\capRead_Python/xgboost_model1.json")
# StandardScaler for preprocessing capacitive features
scaler = joblib.load('E:\pythonProject\capRead_Python/scaler.pkl')

# Class mapping
CLASS_MAP = {
    0: "Foam",
    1: "Metal",
    2: "Paper",
    3: "Plastic Bag",
    4: "Plastic Bottle",
    5: "Stone",
    6: "Wood"
}


def yolo_predict_probs(image_path, num_classes=7, default_conf=0.8, paper_conf=0.8, paper_class_id=5):
    results = yolo_model(image_path, verbose=False)
    result = results[0]

    # No detection case
    if len(result.boxes) == 0:
        return np.ones(num_classes) / num_classes

    # If multiple boxes, select the one with highest confidence
    max_conf_idx = result.boxes.conf.argmax().item()
    class_id = int(result.boxes.cls[max_conf_idx].item())

    confidence = paper_conf if class_id == paper_class_id else default_conf

    probs = np.zeros(num_classes)
    probs[class_id] = confidence

    remaining_prob = 1.0 - confidence
    probs += remaining_prob / (num_classes - 1)
    probs[class_id] = confidence

    return probs


def xgb_predict_probs(cap_features):
    features_scaled = scaler.transform([cap_features])
    dmatrix = xgb.DMatrix(features_scaled)
    p_xgb = xgb_model.predict(dmatrix)[0]
    return p_xgb


# =========== Combined Fusion Prediction ===========
def multimodal_fusion_prediction(image_path, cap_features, P_yolo, P_xgb):
    p_yolo = yolo_predict_probs(image_path)
    p_xgb = xgb_predict_probs(cap_features)

    combined_prob = predict_with_conditional_fusion(p_yolo, p_xgb, P_yolo, P_xgb)

    final_class_idx = np.argmax(combined_prob)
    final_class_name = CLASS_MAP[final_class_idx]
    final_confidence = combined_prob[final_class_idx]

    # Display individual predictions
    yolo_class_idx = np.argmax(p_yolo)
    xgb_class_idx = np.argmax(p_xgb)

    print(f"📸 YOLO prediction: {CLASS_MAP[yolo_class_idx]} (conf={p_yolo[yolo_class_idx]:.3f})")
    print(f"📡 XGB prediction:  {CLASS_MAP[xgb_class_idx]} (conf={p_xgb[xgb_class_idx]:.3f})")
    print(f"🔍 Fused result:    {final_class_name} (conf={final_confidence:.3f})")

    return final_class_idx, final_confidence


# ==== Directory and Excel setup ====
data_dir = "\data_mutimodal"

excel_data_files = {
    "Foam": "data_foam.xlsx",
    "Stone": "data_rock.xlsx",
    "Metal": "data_metal.xlsx",
    "Wood": "data_wood.xlsx",
    "Plastic Bag": "data_Plastic Bag.xlsx",
    "Paper": "data_napikin.xlsx",
    "Plastic Bottle": "data_Plastic Bottle.xlsx"
}

# Preload capacitive data
excel_data = {}
for material, file in excel_data_files.items():
    df = pd.read_excel(os.path.join(data_dir, file)).iloc[:, :23].values
    excel_data[material] = df

output_lines = []

# Iterate over material categories
for class_idx, material in CLASS_MAP.items():
    folder_path = os.path.join(data_dir, material)
    image_files = glob.glob(os.path.join(folder_path, "*.jpg"))

    cap_data_array = excel_data[material]

    for image_path in image_files:
        # Randomly sample a capacitive feature
        cap_features = cap_data_array[random.randint(0, len(cap_data_array) - 1)]

        # Predict using multimodal fusion
        final_class_idx, final_conf = multimodal_fusion_prediction(
            image_path, cap_features, P_yolo, P_xgb)

        # Retrieve single-modality confidences
        p_yolo = yolo_predict_probs(image_path)
        p_xgb = xgb_predict_probs(cap_features)

        yolo_conf = p_yolo[np.argmax(p_yolo)]
        xgb_conf = p_xgb[np.argmax(p_xgb)]

        # Save the result
        line = f"{material},{yolo_conf:.3f},{xgb_conf:.3f},{CLASS_MAP[final_class_idx]}"
        output_lines.append(line)

        print(f"Processed: {os.path.basename(image_path)} -> {line}")

# Save final predictions to text file
output_txt = os.path.join(data_dir, "conditional_fusion_results.txt")
with open(output_txt, "w") as f:
    f.write("\n".join(output_lines))
