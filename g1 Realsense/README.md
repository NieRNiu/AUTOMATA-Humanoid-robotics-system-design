
# Football Detection with YOLOv11s

This project implements object detection for football field elements (Ball, Goalpost, L, T, X) using the [Ultralytics YOLOv11s](https://github.com/ultralytics/ultralytics) model. The training is conducted on a custom dataset with strong detection performance for most classes.

---

## 📂 Dataset

- **Dataset Path**: `./yolo_dataset`
- **Classes**:
  - `0`: Ball
  - `1`: Post
  - `2`: L
  - `3`: T
  - `4`: X

```
dataset/
├── train/
│    └── images/, labels/
└── val/
     └── images/, labels/
```

- **Annotation Format**: YOLO (txt)
- **Dataset Config YAML**:
```yaml
path: /absolute/path/to/yolo_dataset
train: train/images
val: val/images
names:
  0: Ball
  1: Post
  2: L
  3: T
  4: X
```

---

## 🏗️ Dataset Collection Process

- **Image Collection**: Images were captured by subscribing to topics from a horizontal and a tilted camera. Frames containing football, goalposts, or field lines were selectively saved based on distance or angle criteria.
- **Annotation**: The images were annotated using the `labelImg` tool and split into training and validation sets.

---

## ⚙️ Training Configuration

| Parameter       | Value           |
| --------------- | --------------- |
| Model           | `yolov11s.pt`   |
| Image Size      | `640x640`       |
| Batch Size      | 16              |
| Epochs          | 600             |
| Optimizer       | `AdamW`         |
| LR Initial      | 0.01            |
| LR Final        | 0.01 × 0.1      |
| LR Schedule     | Cosine Decay    |
| AMP             | ✅ Enabled       |
| Augmentation    | ✅ Enabled       |
| Save Checkpoint | Every 10 Epochs |
| Device          | GPU:0           |

Command used:

```python
model.train(**train_params)
```

---

## 📈 Evaluation Summary

After training, the model achieved the following validation metrics:

### ✅ Global Performance

- **mAP@0.5**: **0.845**
- **Best F1 Score**: **0.80 @ conf=0.521**
- **Best Precision**: **1.00 @ conf=0.893**
- **Best Recall**: **0.96 @ conf=0.000**

### 📊 Per-Class Performance (mAP@0.5)

| Class | Score     | Comment            |
| ----- | --------- | ------------------ |
| Ball  | 0.995     | ✅ Excellent        |
| Post  | 0.891     | ✅ Very good        |
| L     | 0.857     | ✅ Very good        |
| X     | 0.930     | ✅ Excellent        |
| **T** | **0.550** | ❗Needs improvement |

---

## 📉 Observations from Loss & Metric Curves

- Loss curves (`box`, `cls`, `dfl`) show **smooth and stable convergence**.
- Validation losses drop consistently, indicating no overfitting.
- mAP and precision metrics steadily increase, especially after epoch 100.
- Class imbalance not severe, but **T-class confusion is high**, requiring attention.

---

## 📌 Key Improvements from Previous Version

| Change Made                 | Description                                      |
| --------------------------- | ------------------------------------------------ |
| ✅ Switched to `yolov11s.pt` | Improved backbone capacity over `n` version      |
| ✅ Optimizer: `AdamW`        | Stable convergence and better generalization     |
| ✅ `cos_lr=True`             | Smooth learning rate decay                       |
| ✅ Higher image size         | From `320x320` to `640x640`                      |
| ✅ Confidence tuning         | Identified optimal F1 at `conf ≈ 0.52`           |
| ✅ Better augmentation       | Default mosaic + HSV + translation, etc.         |
| ✅ Saved checkpoints         | `save_period=10` for easy resuming/checkpointing |

---

## ⚠️ Known Issues / Next Steps

### T-Class Weakness

- Confusion with `L` and `X` classes (seen in confusion matrix)
- Precision and recall curve for T shows unstable detection quality
- Possible cause: structural similarity + insufficient unique features

### Suggested Improvements

1. **Fine-Tune on T-Class**  
   Freeze backbone, only train head with lower LR on T-heavy subset.

   ```python
   model.freeze()  
   model.model.model[-1].train()
   ```

2. **Class-Weighted Loss (optional)**  
   Increase T-class classification penalty (requires loss.py editing)

3. **Augmentation Focused on T-Class**  
   Increase contrast, deform shape to help differentiate from L/X

---

## 📁 Output Folder Structure

```bash
runs/train/football_detection_v2/
├── weights/
│   ├── best.pt
│   └── last.pt
├── results.png           # loss & metric curves
├── PR_curve.png          # PR for each class
├── confusion_matrix.png
├── confusion_matrix_normalized.png
└── ...                   # more plots and logs
```
