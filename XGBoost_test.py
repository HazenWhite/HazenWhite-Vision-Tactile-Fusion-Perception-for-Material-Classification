import threading
import time
import pandas as pd
from cap_read import *
from openpyxl import load_workbook
import joblib
import xgboost as xgb
import numpy as np

# ========== Load XGBoost model ==========
# Load XGBoost model from .json file
xgb_model = xgb.Booster()
xgb_model.load_model("xgboost_model1.json")

# ========== Class mapping ==========
CLASS_MAP = {
    0: "Foam",
    1: "Rock",
    2: "Metal",
    3: "Wood",
    4: "Plastic_Bag",
    5: "Paper",
    6: "Plastic_Battle"
}

# ========== Threshold & Buffers ==========
T_FORCE_THRESHOLD = 1.0
channel_data_buffer = []
normal_force_data_buffer = []
t_force_data_buffer = []
self_cap_data_buffer = []
m_cap_data_buffer = []

def wait_for_force_trigger(threshold=T_FORCE_THRESHOLD, timeout=10):
    """Wait until the normal force exceeds a specified threshold."""
    for _ in range(timeout):
        sensor_data = data_queue.get(timeout=1)
        force = sensor_data["normal_force_data"][0]
        print(f"Waiting for force trigger: normal_force_data[0] = {force:.6f}")
        if force > threshold:
            return True
        time.sleep(1)
    print("⚠ Timeout: force threshold not reached.")
    return False

def predict_material_class():
    """Predict the material class using XGBoost."""
    avg_channel = [sum(x) / len(x) for x in zip(*channel_data_buffer)]
    avg_normal_force = [sum(x) / len(x) for x in zip(*normal_force_data_buffer)]
    avg_t_force = [sum(x) / len(x) for x in zip(*t_force_data_buffer)]
    avg_self_cap = [sum(x) / len(x) for x in zip(*self_cap_data_buffer)]
    avg_m_cap = [sum(x) / len(x) for x in zip(*m_cap_data_buffer)]

    features = avg_channel + avg_normal_force + avg_t_force + avg_self_cap + avg_m_cap
    scaler = joblib.load('scaler.pkl')
    features_scaled = scaler.transform(pd.DataFrame([features]))
    dmatrix = xgb.DMatrix(features_scaled)

    # Predict probability
    probabilities = xgb_model.predict(dmatrix)[0]
    prediction = int(np.argmax(probabilities))

    # Print probability for each class
    print("\nClass prediction confidence:")
    for idx, prob in enumerate(probabilities):
        class_name = CLASS_MAP.get(idx, "Unknown")
        print(f" - {class_name:<15}: {prob * 100:.2f}%")

    print(f"\nFinal classification: Class {prediction} ({CLASS_MAP.get(prediction, 'Unknown')}), Confidence: {probabilities[prediction] * 100:.2f}%\n")

    return prediction, probabilities

if __name__ == "__main__":
    # Start the capacitive sensor reading thread
    cap_thread = threading.Thread(target=capReadThread, daemon=True)
    cap_thread.start()

    while True:
        input("Press Enter to start a new material recognition round...")
        if not wait_for_force_trigger():
            continue

        # Clear previous data buffers
        channel_data_buffer.clear()
        normal_force_data_buffer.clear()
        t_force_data_buffer.clear()
        self_cap_data_buffer.clear()
        m_cap_data_buffer.clear()

        print("Collecting 50 sensor data samples...")
        while len(channel_data_buffer) < 50:
            sensor_data = data_queue.get(timeout=1)
            channel_data_buffer.append(sensor_data["channel_data"])
            normal_force_data_buffer.append(sensor_data["normal_force_data"])
            t_force_data_buffer.append(sensor_data["t_force_data"])
            self_cap_data_buffer.append(sensor_data["self_cap_data"])
            m_cap_data_buffer.append(sensor_data["m_cap_data"])

        print("Data collection complete. Starting classification...")
        predict_material_class()

        print("Ready for the next recognition cycle...\n")
