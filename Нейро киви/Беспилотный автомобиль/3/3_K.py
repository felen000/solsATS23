import time
import yolopy  # библиотека для инференса моделей Yolo на NPU
import cv2
from arduino import Arduino  # класс из базового кода для общения с Arduino
import numpy as np
from utils import *  # набор функций, предоставленных участникам

CAR_SPEED = 1500  # Начальная нулевая скорость автомобиля
ARDUINO_PORT = '/dev/ttyUSB0'  # Порт, к которому подключена Arduino
CAMERA_ID = '/dev/video0'  # Идентификатор камеры

# параметры ПД-регулятора
KP = 0.55  # 0.22 0.32 0.42
KD = 0.25  # 0.17
last_err = 0  # переменная для хранения предыдущего значения ошибки

SIZE = (533, 300)  # размер изображения при работе с дорожной разметкой
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

THRESHOLD = 200   # порог бинаризации для поиска линий разметки

# Чтение списка имён классов детектора
with open('classes.txt') as file:
    class_names = file.read().splitlines()

# Загрузка модели детектора для обнаружения объектов YOLOv4-tiny
model_file = 'yolov4-tiny_uint8.tmfile'
model = yolopy.Model(model_file,  use_uint8=True, use_timvx=True)

# Инициализация обмена данными с Arduino через UART
arduino = Arduino(ARDUINO_PORT, baudrate=115200, timeout=10)
time.sleep(1)

# Инициализация работы с камерой
cap = cv2.VideoCapture(CAMERA_ID, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

# начинаем движение с заданной скоростью, нулевой
arduino.set_speed(CAR_SPEED)

try:
    while True:
        ret, frame = cap.read()  # чтение кадра с камеры
        if not ret:
            break

        img = cv2.resize(frame, SIZE) # приводим изображение к удобному для вычислений размеру
        binary = binarize(img, THRESHOLD)  # бинаризуем изображение для поиска белых линий
        perspective = trans_perspective(binary, TRAP, RECT, SIZE)  # извлекаем область перед колёсами
        left, right = centre_mass(perspective)  # получаем координаты левой и правой линии разметки

        # вычислем отклонение центра полосы движения от центра кадра
        err = 0 - ((left + right) // 2 - SIZE[0] // 2)
        if abs(right - left) < 100:
            err = last_err
        # преобразуем ошибку в угол поворота колёс через ПД-регулятор
        angle = int(90 + KP * err + KD * (err - last_err))
        last_err = err

        # проверка на превышение допустимых углов поворота колёс
        if angle < 60:
            angle = 60
        elif angle > 120:
            angle = 120

        start = time.monotonic()  # сохраняем время старта обработки кадра детектором
        classes, scores, boxes = model.detect(img)  # передаём изображение детектору
        # classes хранит список из id классов всех сдетектированных объектов
        # scores уверенность в точности распознавания объектов
        # boxes - список рамок для всех объектов

        end = time.monotonic()  # сохраняем время старта обработки кадра детектором
        inference_time = end - start  # рассчитываем время обработки одного кадра
        inference_fps = 1/inference_time  # вычисляем максимально возмоный FPS обработки кадров

        # Вывод в консоль количество детектированных объектов
        print("Total of " + str (len(classes)) + "objects were detected")

        # Изменяем угол поворота руевых колёс и выводим его значение в текстовом виде
        print(f'angle={angle}')
        arduino.set_angle(angle)

except KeyboardInterrupt as e:  # обработка остановки программы сочетанием клавиш Ctrl+C,
    print('Program stopped!', e)  # необходима для корректного завершения программы при остановке через терминал

# Остановка автомобиля и разблокировка доступа к камере.
arduino.stop()
arduino.set_angle(90)
cap.release()
