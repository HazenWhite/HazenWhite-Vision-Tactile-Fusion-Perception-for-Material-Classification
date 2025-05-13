import pandas as pd
import numpy as np
import xgboost as xgb
from sklearn.preprocessing import StandardScaler
from sklearn.metrics import accuracy_score, confusion_matrix
import matplotlib.pyplot as plt
import seaborn as sns
import joblib


def plot_confusion_matrix_percentage(y_true, y_pred, classes, title='Confusion Matrix', save_path=None):
    cm = confusion_matrix(y_true, y_pred)
    cm_percentage = cm.astype('float') / cm.sum(axis=1)[:, np.newaxis]
    cm_percentage = np.nan_to_num(cm_percentage)  # Handle division by zero

    # Format as percentage strings, keep two decimals, and leave 0.00 as blank
    annot = np.where(cm_percentage == 0, "", np.round(cm_percentage, 2).astype(str))

    plt.figure(figsize=(12, 10))  # Set figure size (larger than default)
    sns.heatmap(cm_percentage,
                annot=annot,
                fmt='',
                cmap='Greens',
                xticklabels=classes,
                yticklabels=classes,
                cbar_kws={'label': 'Percentage'},
                square=True,           # Make each cell square
                linewidths=0,          # No gaps
                linecolor='white')     # Fallback grid color

    plt.title(title, fontsize=18, pad=15)
    plt.xlabel('Predicted Label', fontsize=15)
    plt.ylabel('True Label', fontsize=15)
    plt.xticks(fontsize=12, rotation=45)
    plt.yticks(fontsize=12, rotation=0)
    plt.tight_layout()

    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight', transparent=True)
    plt.show()


# File paths for training and validation data
file_paths = {
    "Foam": ["data_foam_train.xlsx", "data_foam_val.xlsx"],
    "Metal": ["data_metal_train.xlsx", "data_metal_val.xlsx"],
    "Paper": ["data_napikin_train.xlsx", "data_napikin_val.xlsx"],
    "Plastic Bag": ["data_bag_train.xlsx", "data_bag_val.xlsx"],
    "Plastic Bottle": ["data_plastic_train.xlsx", "data_plastic_val.xlsx"],
    "Stone": ["data_rock_train.xlsx", "data_rock_val.xlsx"],
    "Wood": ["data_wood_train.xlsx", "data_wood_val.xlsx"],
}

# Label encoding
label_mapping = {"Foam": 0, "Metal": 1, "Paper": 2, "Plastic Bag": 3, "Plastic Bottle": 4, "Stone": 5, "Wood": 6}

X_train_list, y_train_list = [], []
X_val_list, y_val_list = [], []

# Load data from each class
for material, files in file_paths.items():
    file = files[0]
    df = pd.read_excel(file).iloc[:, :23]
    X_train_list.append(df.values)
    y_train_list.append(np.full(df.shape[0], label_mapping[material]))

    df_val = pd.read_excel(files[1]).iloc[:, :23]
    X_val_list.append(df_val.values)
    y_val_list.append(np.full(df_val.shape[0], label_mapping[material]))

X_train = np.vstack(X_train_list)
y_train = np.hstack(y_train_list)
X_val = np.vstack(X_val_list)
y_val = np.hstack(y_val_list)

# ======================
# Data Preprocessing
# ======================
scaler = StandardScaler()
X_train_scaled = scaler.fit_transform(X_train)
joblib.dump(scaler, 'scaler.pkl')
X_val_scaled = scaler.transform(X_val)

dtrain = xgb.DMatrix(X_train_scaled, label=y_train)
dval = xgb.DMatrix(X_val_scaled, label=y_val)

# ======================
# Train XGBoost and Monitor Loss
# ======================
param = {
    "objective": "multi:softprob",
    "num_class": 7,
    "max_depth": 10,
    "learning_rate": 0.01,
    "min_child_weight": 10,
    "subsample": 0.9,
    "colsample_bytree": 0.9,
    "tree_method": "hist",
    "device": "cuda",
}

num_round = 1000
evals_result = {}
model = xgb.train(
    param,
    dtrain,
    num_boost_round=num_round,
    evals=[(dtrain, "train"), (dval, "val")],
    evals_result=evals_result,
    verbose_eval=True
)

# Make predictions
y_val_pred_probs = model.predict(dval)
y_val_pred = np.argmax(y_val_pred_probs, axis=1)
val_accuracy = accuracy_score(y_val, y_val_pred)

print(f"XGBoost (GPU) classification accuracy on validation set: {val_accuracy:.4f}")

# ======================
# Plot Confusion Matrix (Overridden)
# ======================
def plot_confusion_matrix_percentage(y_true, y_pred, classes, title='Confusion Matrix', save_path=None):
    from sklearn.metrics import confusion_matrix
    import matplotlib.pyplot as plt
    import seaborn as sns
    import numpy as np

    cm = confusion_matrix(y_true, y_pred)
    cm_percentage = cm.astype('float') / cm.sum(axis=1)[:, np.newaxis]
    cm_percentage = np.nan_to_num(cm_percentage)
    cm_percentage = cm_percentage.T  # Make X-axis as True label, Y-axis as Predicted label

    annot = np.where(cm_percentage == 0, "", np.round(cm_percentage * 100, 1).astype(str) + "%")

    plt.figure(figsize=(12, 10))
    sns.heatmap(
        cm_percentage,
        annot=annot,
        fmt='',
        cmap='Greens',
        xticklabels=classes,
        yticklabels=classes,
        cbar_kws={'label': 'Percentage (%)'},
        square=True,
        linewidths=0
    )

    plt.title(title, fontsize=18, pad=15)
    plt.xlabel('True Label', fontsize=15)
    plt.ylabel('Predicted Label', fontsize=15)
    plt.xticks(rotation=0, fontsize=12)
    plt.yticks(rotation=90, fontsize=12)
    plt.tight_layout()

    if save_path:
        plt.savefig(save_path, dpi=1200, bbox_inches='tight', transparent=True)
    plt.show()


# Plot confusion matrix
class_names = sorted(label_mapping.keys(), key=lambda x: label_mapping[x])

plot_confusion_matrix_percentage(
    y_val,
    y_val_pred,
    classes=class_names,
    title='Validation Set Confusion Matrix (Percentage)',
    save_path='confusion_matrix_validation.png'
)

# ======================
# Plot Training & Validation Loss
# ======================
train_loss = evals_result["train"]["mlogloss"]
val_loss = evals_result["val"]["mlogloss"]

plt.figure(figsize=(8, 5))
plt.plot(train_loss, label="Train Loss", color="blue")
plt.plot(val_loss, label="Validation Loss", color="red")
plt.xlabel("Epoch")
plt.ylabel("Log Loss")
plt.title("Training & Validation Loss Curve")
plt.legend()
plt.grid()
plt.show()

# Save model
model.save_model("xgboost_model.json")
