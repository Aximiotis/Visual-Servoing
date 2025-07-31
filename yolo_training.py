from ultralytics import YOLO

# Φόρτωσε pretrained YOLOv8 μοντέλο
model = YOLO("yolov8n.pt")  # Μπορείς να βάλεις yolov8s.pt, m.pt, l.pt κλπ.

# Κάνε retrain στο δικό σου dataset
model.train(
    data="/home/roboticslab/Downloads/yolo_detection/data.yaml",
    epochs=50,
    imgsz=640,
    batch=16,
    name="yolo_retrain_single_class1"
)
