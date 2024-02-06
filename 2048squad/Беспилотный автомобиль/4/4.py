import time
import yolopy  # библиотека для инференса моделей Yolo на NPU
import cv2
import numpy as np
from arduino import Arduino  # класс из базового кода для общения с Arduino
from utils import *  # набор функций, предоставленных участникам


CAR_SPEED = 1446   # Начальная скорость автомобиля
CAR_SPEED_SLOW = 1440  # Скорость автомобиля в режиме медленного движения
CAR_SPEED_BACK = 1550  # Скорость автомобиля для движения назад
ARDUINO_PORT = '/dev/ttyUSB0'  # Порт, к которому подключена Arduino
CAMERA_ID = '/dev/video0'  # Идентификатор камеры

KP = 0.55  # коэффициент пропорциональной составляющей ПИД-регулятора
KD = 0.25  # коэффициент дифференциальной составляющей ПИД-регулятора
last_err = 0  # последнее значение ошибки

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

THRESHOLD = 200  # порог бинаризации для поиска линий разметки

# Чтение списка имён классов детектора
with open('classes.txt') as file:
    class_names = file.read().splitlines()

# Загрузка модели детектора для обнаружения объектов YOLOv4-tiny
model_file = 'yolov4-tiny_uint8.tmfile'
model = yolopy.Model(model_file, use_uint8=True, use_timvx=True)

# Инициализация обмена данными с Arduino через UART
arduino = Arduino(ARDUINO_PORT, baudrate=115200, timeout=10)
time.sleep(1)

# Инициализация работы с камерой
cap = cv2.VideoCapture(CAMERA_ID, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

# Установка начальной скорости автомобиля
arduino.set_speed(CAR_SPEED)

try:
    while True:
        ret, frame = cap.read()  # чтение кадра с камеры
        if not ret:
            break

        img = cv2.resize(frame, SIZE)  # приводим изображение к удобному для вычислений размеру
        binary = binarize(img, THRESHOLD)  # бинаризуем изображение для поиска белых линий
        perspective = trans_perspective(binary, TRAP, RECT, SIZE)  # извлекаем область перед колёсами
        left, right = centre_mass(perspective)  # получаем координаты левой и правой линии разметки

        # вычислем отклонение центра полосы движения от центра кадра
        err = 0 - ((left + right) // 2 - SIZE[0] // 2)
        if abs(right - left) < 100:
            err = last_err
        # преобразуем ошибку в угол поворота колёс через ПД-регулятор
        angle = int(90 + KP * err + KD * (err - last_err))

        # проверка на превышение допустимых углов поворота колёс
        if angle < 60:
            angle = 60
        elif angle > 120:
            angle = 120

        last_err = err  # запоминаем текущую ошибку, для Д-состовляющей регулятора

        # Обнаружение объектов на изображении с помощью модели YOLOv4-tiny
        classes, scores, boxes = model.detect(img)

        # Единственный объект, который обучен распознавть детектор - знак "парковка"
        if len(classes) > 0:  # если знак обнаружен, то
            # получаем координаты левого верхнего угла, ширины и высоты знака
            box = boxes[0]
            x, y, w, h = box
            xc = x + w // 2  # вычисление координаты центра знака

            if xc > SIZE[0] * 0.8:  # если знак парковки находится в правой части изображения, то
                parking = True  # устанавливаем флаг начала манёвра
                star_time = time.monotonic() # охраняем время начала манёвра

        if parking:  # если манёвр начался, то отслеживаем его длительность
            now_time = time.monotonic()  # сохраняем время в данный момент
            if now_time - star_time <= 2:  # первые 2 секунды движемся назад
                arduino.set_speed(CAR_SPEED_BACK)
            # от 3 до 5,5 секунд поворачиваем налево, т.е. занимаем парковочное место
            elif now_time - star_time  <= 4: # ещё 2 секунды поворачиваем направо
                arduino.set_angle(60)  # поворачиваем рулевые колуса максимально направо
                arduino.set_speed(CAR_SPEED_SLOW)  # медленно едем вперёд
            elif now_time - star_time  <= 6:  # ещё 2 секунды поворачиваем налево
                arduino.set_angle(110)  # поворачиваем рулевые колуса максимально направо
            else:
                arduino.stop() # станавливаем беспилотный автомобиль
                break  # выходим из цикла обработки кадров

        else:  # если знак парковки не обнаружен, то
            arduino.set_angle(angle)  # поворачиваем рулевые колёса на угол рассчитанный ранее

except KeyboardInterrupt as e:  # обработка остановки программы сочетанием клавиш Ctrl+C,
    print('Program stopped!', e)  # необходима для корректного завершения программы при остановке через терминал

# Остановка автомобиля и разблокировка доступа к камере.
arduino.stop()
arduino.set_angle(90)
cap.release()
