from ultralytics import YOLO
from pathlib import Path
import torch
import multiprocessing

DATASET = "data_v2.yaml"
EPOCHS = 100
IMAGE_SIZE = 640
BATCH_SIZE = 8
RUN_NAME = "jerison_face_v2"

def main():
    print()
    print("======================================")
    print("JERISON FACE MODEL TRAINING")
    print("======================================")
    print()

    if torch.cuda.is_available():
        device = 0
        print("GPU detected:")
        print(torch.cuda.get_device_name(0))
    else:
        device = "cpu"
        print("No CUDA GPU detected.")
        print("Training on CPU.")

    print()
    print("Loading YOLOv8 Nano...")

    model = YOLO("yolov8n.pt")

    print()
    print("Starting training...")
    print()

    project_folder = Path.cwd() / "training_runs"

    model.train(
        data=DATASET,
        epochs=EPOCHS,
        imgsz=IMAGE_SIZE,
        batch=BATCH_SIZE,
        device=device,
        project=str(project_folder),
        name=RUN_NAME,
        patience=20,
        workers=0,
        cache=False,
        plots=True,
        save=True,
        exist_ok=True
    )

    best_model_path = Path(model.trainer.best)

    print()
    print("======================================")
    print("TRAINING COMPLETE")
    print("======================================")
    print()
    print(f"Best model: {best_model_path}")

    if not best_model_path.exists():
        print()
        print("ERROR: Could not find best.pt")
        return

    print()
    print("Loading best model...")

    best_model = YOLO(str(best_model_path))

    print()
    print("Exporting ONNX...")

    onnx_path = best_model.export(
        format="onnx",
        imgsz=IMAGE_SIZE,
        simplify=True,
        dynamic=False,
        opset=12
    )

    print()
    print("======================================")
    print("FINISHED")
    print("======================================")
    print()
    print(f"Best PyTorch model: {best_model_path}")
    print(f"ONNX model: {onnx_path}")

if __name__ == "__main__":
    multiprocessing.freeze_support()
    main()