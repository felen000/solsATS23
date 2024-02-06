import time
import yolopy  # библиотека для инференса моделей Yolo на NPU
import cv2
from arduino import Arduino  # класс из базового кода для общения с Arduino
import numpy as np
from utils import *  # набор функций, предоставленных участникам


CAR_SPEED = 1435  # Начальная нулевая скорость автомобиля
ARDUINO_PORT = '/dev/ttyUSB0'  # Порт, к которому подключена Arduino
CAMERA_ID = '/dev/video0'  # Идентификатор камеры

# Инициализация обмена данными с Arduino через UART
arduino = Arduino(ARDUINO_PORT, baudrate=115200, timeout=10)
time.sleep(1)

# Инициализация работы с камерой
cap = cv2.VideoCapture(CAMERA_ID, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

# параметры ПД-регулятора
KP = 0.55
KD = 0.25
last_err = 0  # переменная для хранения предыдущего значения ошибки

SIZE = (533, 300)  # размер изображения при работе с дорожной разметкой
SIZE2 = (640, 640)  # размер изображения для обработки детектором
# Параметры прямоугольника, в который мы преобразуем трапецивидную область перед колёсами беспилотника
RECT = np.float32([[0, SIZE[1]],
                   [SIZE[0], SIZE[1]],
                   [SIZE[0], 0],
                   [0, 0]])

# Параметры трапеции, ограничивающей на кадре область перед колёсами атомобиля
TRAP = np.float32([[10, 299],
                   [523, 299],
                   [440, 200],
                   [93, 200]])

# Параметры трапеции приведённые к типу int, для функции рисования
src_draw = np.array(TRAP, dtype=np.int32)

THRESHOLD = 200  # порог бинаризации для поиска линий разметки

# Чтение списка имён классов детектора
with open('classes.txt') as file:
    class_names = file.read().splitlines()

# Загрузка модели детектора для обнаружения объектов YOLOv4-tiny
model_file = 'test.tmfile'
model = yolopy.Model(model_file, yolopy.YOLOV4_TINY, use_uint8=True, use_timvx=True, cls_num=10)

# Инициализация обмена данными с Arduino через UART
arduino = Arduino(ARDUINO_PORT, baudrate=115200, timeout=10)
time.sleep(1)
arduino.stop()  # задание нулевой стартовой скорости

# Инициализация работы с камерой
cap = cv2.VideoCapture(CAMERA_ID, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
print(cap.isOpened())  # сообщение об успешности захвата видеопотока с камеры

try:
    while True:
        ret, frame = cap.read()  # чтение кадра с камеры
        if not ret:
            break

        # Изменение размера кадра для обработки детектором
        img2 = cv2.resize(frame, SIZE2)
        # Обнаружение объектов на изображении с помощью детектора
        classes, scores, boxes = model.detect(img2)
        # преобразование числовых значений классов в соответствующие им имена
        detected = [class_names[cls] for cls in classes]
        # вывод классов детектированных объектов
        print('Detected:', ','.join(detected))
        # вывод количества детектированных объектов
        print("Count:", len(detected))

        # В этой части программы был размещён алгоритм движения по разметке,
        # но результаты его работы не использовались,
        # т.е. беспилотный автомобиль стоял на месте.

except KeyboardInterrupt as e:  # обработка остановки программы сочетанием клавиш Ctrl+C,
    print('Program stopped!', e)  # необходима для корректного завершения программы при остановке через терминал

# Остановка автомобиля и разблокировка доступа к камере.
arduino.stop()
arduino.set_angle(90)
cap.release()

