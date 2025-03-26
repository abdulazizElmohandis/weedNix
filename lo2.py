""""
from ultralytics import YOLO

model = YOLO("yolov8n.pt")  # تحميل الموديل
model.train(data=r"D:/computer_vision_UNI/logo/logo_detect.v2i.yolov8/data.yaml", epochs=10, imgsz=640)  # تدريب '

from ultralytics import YOLO

# تحميل الموديل المدرب
model = YOLO("runs/detect/train/weights/best.pt")  # تأكد من المسار الصحيح للوزنات المدربة

# تقييم الموديل على مجموعة الاختبار
evaluation_results = model.val(data="D:/computer_vision_UNI/logo/logo_detect.v2i.yolov8/data.yaml")

# طباعة النتائج
print(evaluation_results)

#عمل ترين وتيست بعد تعديل بعض الهايبر برامتر

from ultralytics import YOLO
import torch

# تحميل الموديل المدرب
model = YOLO("runs/detect/train/weights/best.pt")  # تأكد من المسار الصحيح للوزنات المدربة

# ضبط معلمات التدريب لمنع الـ overfitting
training_params = {
    "epochs": 10,  # زيادة عدد الـ Epochs لتحسين الاستقرار
    "lr0": 0.001,  # تقليل معدل التعلم قليلاً
    "augment": True,  # تفعيل Augmentation لتحسين تعميم النموذج
    "patience": 5,  # تفعيل Early Stopping لمنع الـ overfitting
    "weight_decay": 0.0005  # إضافة Regularization
    
}

# إعادة تدريب الموديل مع التحسينات
model.train(data="D:/computer_vision_UNI/logo/logo_detect.v2i.yolov8/data.yaml", **training_params)

# تقييم الموديل على مجموعة الاختبار بعد التحسين
evaluation_results = model.val(data="D:/computer_vision_UNI/logo/logo_detect.v2i.yolov8/data.yaml")

# طباعة النتائج
print(evaluation_results)

####################################################################################################################

#تشغيل الموديل ريل تايم 

from ultralytics import YOLO
import cv2

# تحميل الموديل المدرب
model = YOLO("D:/computer_vision_UNI/runs/detect/train5/weights/best.pt")

# فتح الكاميرا (0 لكاميرا اللاب، 1 لو كاميرا USB خارجية)
cap = cv2.VideoCapture(0)

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    # تشغيل الموديل على الفريم الحالي
    results = model(frame, show=True,conf=0.7)

    # عرض النتيجة
    cv2.imshow("YOLOv8 Live Detection", frame)

    # الخروج عند الضغط على "q"
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
"""

#تشغيل الموديل ريل تايم 

from ultralytics import YOLO
import cv2

# تحميل الموديل المدرب
model = YOLO("D:/computer_vision_UNI/runs/detect/train8/weights/best.pt")

# فتح الكاميرا (0 لكاميرا اللاب، 1 لو كاميرا USB خارجية)
cap = cv2.VideoCapture(0)

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    # تشغيل الموديل على الفريم الحالي
    results = model(frame, show=True,conf=0.6)

    # عرض النتيجة
    cv2.imshow("YOLOv8 Live Detection", frame)

    # الخروج عند الضغط على "q"
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
