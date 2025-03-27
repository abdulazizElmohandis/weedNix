from ultralytics import YOLO
import cv2

# تحميل الموديل المدرب
model = YOLO("D:/computer_vision_UNI/runs/detect/train5/weights/best.pt")

# فتح الكاميرا
cap = cv2.VideoCapture(3)  # استخدم الرقم الصح حسب كاميرا الموبايل

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    # تشغيل YOLO على الفريم
    results = model(frame, conf=0.7)

    # عرض الفريم
    cv2.imshow("YOLOv8 Live Detection", frame)

    # الخروج عند الضغط على "q"
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
