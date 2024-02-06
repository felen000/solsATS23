import cv2
import yolopy  # библиотека для инференса моделей Yolo на NPU

CAMERA_ID = '/dev/video0'# Идентификатор камеры

# Загрузка модели детектора для обнаружения объектов YOLOv4-tiny
model_file = '../anything_yolov4-tiny-obj_uint8.tmfile'
model = yolopy.Model(model_file, use_uint8=True, use_timvx=True, cls_num=1)

# Инициализация работы с камерой
cap = cv2.VideoCapture(CAMERA_ID, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

while True:
    ret, frame = cap.read()  # чтение кадра с камеры
    if not ret:
        break

    classes, scores, boxes = model.detect(frame)  # передаём изображение детектору
    # Выводим в консоль количество детектированных объектов
    print(len(classes))
