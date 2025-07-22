from ultralytics import YOLO
import sys
import os

def run_inference(model_path, image_path, save=True, show=False):
    # Load the YOLOv8 model (can be .pt or .onnx)
    model = YOLO(model_path)

    # Run inference on the input image
    results = model(image_path, save=save, show=show)

    # Print results
    for r in results:
        print("Detected classes:", r.names)
        print("Boxes:")
        for box in r.boxes:
            cls_id = int(box.cls[0])
            conf = float(box.conf[0])
            coords = box.xyxy[0].tolist()
            print(f"Class {cls_id} ({r.names[cls_id]}), Confidence: {conf:.2f}, Box: {coords}")

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python run_yolo_on_image.py <model_path> <image_path>")
        sys.exit(1)

    model_path = "weights.pt"

    image_path = sys.argv[1]

    if not os.path.exists(model_path):
        print(f"Model file not found: {model_path}")
        sys.exit(1)

    if not os.path.exists(image_path):
        print(f"Image file not found: {image_path}")
        sys.exit(1)

    run_inference(model_path, image_path, save=True, show=False)
