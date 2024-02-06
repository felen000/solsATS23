import time
import yolopy  # библиотека для инференса моделей Yolo на NPU
import cv2
import numpy as np
from arduino import Arduino  # класс из базового кода для общения с Arduino
from utils import *  # набор функций, предоставленных участникам


CAR_SPEED = 1430   # Начальная скорость автомобиля
ARDUINO_PORT = '/dev/ttyUSB0'  # Порт, к которому подключена Arduino
CAMERA_ID = '/dev/video0'  # Идентификатор камеры

# параметры ПД-регулятора
KP = 0.37
KD = 0.23
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

# Функция бинаризации изображения, для поиска линий разметки на изображении
def binarize2(img, threshold):
    hls = cv2.cvtColor(img, cv2.COLOR_BGR2HLS)  # преобразование в цветовое пространство HLS
    binary_h = cv2.inRange(hls, (0, 0, 45), (160, 255, 255))  # бинаризация HLS изображения
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)  # преобразование в оттенки серого
    binary_g = cv2.inRange(gray, threshold, 255)  # бинаризация серого изображения
    binary = cv2.bitwise_and(binary_g, binary_h)  # сложение двух результатов бинаризации
    return binary


THRESHOLD = 200  # порог бинаризации для поиска линий разметки
# Инициализация обмена данными с Arduino через UART
arduino = Arduino(ARDUINO_PORT, baudrate=115200, timeout=10)
time.sleep(1)

# Инициализация работы с камерой
cap = cv2.VideoCapture(CAMERA_ID, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

# Загрузка модели детектора для обнаружения объектов YOLOv4-tiny
model_file = 'yolov4-tiny-obj_best_uint8.tmfile'
model = yolopy.Model(model_file, use_uint8=True, use_timvx=True, cls_num=10)

classes_list = [  # список названий классов детектора
    'artificial roughness',
    'give way',
    'human',
    'movement prohibition',
    'no entry',
    'parking',
    'pedestrian crossing',
    'road works',
    'stop',
    'traffic lights',
]

# Флаг первого детектирования светофора. Как только светофор детектирован переключается в True
tr_lig_flag = False
# Флаг обнаружения красного сигнала светофора. Если сигнал обнаружен принимает значение True
light_flag = False

# Установка начальной скорости автомобиля
arduino.set_speed(CAR_SPEED)

try:
    while True:
        ret, frame = cap.read()  # чтение кадра с камеры
        if not ret:
            print("Have't frame")
            break

        # Изменение размера кадра для обработки детектором
        yol = cv2.resize(frame, (640, 640))
        # Обнаружение объектов на изображении с помощью модели YOLOv4-tiny
        obj = []
        classes, scores, boxes = model.detect(yol)
        traffic_light_box = []
        for i in range(len(classes)):  # перебираем все детектированные объекты
            if classes[i] == "traffic_light":  # если находим светофор, то
                traffic_light_box = boxes[i]   # запоминаем его координаты,
                tr_lig_flag = True             # переключаем флаг и
                arduino.set_speed(1500)        # останавливаемся
                break

        if tr_lig_flag == True:  # если светофор детектирован, то
            # выделяем область изображения с красным сигналом исправного светофора
            light = yol[traffic_light_box[1]:traffic_light_box[1] - 40,
                        traffic_light_box[0] - 70:traffic_light_box[0]]

            # переводим выделенное изображение в цвветофове пространство HSV
            hsv = cv2.cvtColor(light, cv2.COLOR_BGR2HSV_FULL)

            # каждая маска соответствует оттенку красного цвета
            mask = cv2.inRange(hsv, (302, 0, 251), (360, 173, 255))
            mask_down = cv2.inRange(hsv, (0, 0, 251), (37, 173, 255))
            # объединяем "красные" пиксели из двух диапозонов в одном изображении
            binary_light = cv2.bitwise_or(mask, mask_down)

            # если красных пикселей больше некого порога, то
            if np.count_nonzero(binary_light) >= 40:
                light_flag = True    # считаем, что обнаружили красный сигнал
                light_time = time.time()  # выставляем флаг и засекаем время

            # если красный сигнал обнаружен, то движемся через перекрёсток
            # совершая манёвр из двух частей
            if light_flag == True:
                arduino.set_speed(1420)
                if time.time() - light_time <= 0.2:
                    arduino.set_angle(80)
                elif time.time() - light_time <= 1.6:
                    arduino.set_angle(83)
                elif time.time() - light_time > 1.6:
                    # после окончания манёвра переходим к обычному движению по разметке
                    # для этого возвращаем флаги в исходное состояние
                    light_flag = False
                    tr_lig_flag = False
                    arduino.set_speed(1436)

        else:
            img = cv2.resize(frame, SIZE)  # приводим изображение к удобному для вычислений размеру
            binary = binarize2(img, THRESHOLD)  # бинаризуем изображение для поиска белых линий
            # ...
            # ... Алгоритм движения по разметке, аналогичный алгоритму из предыдущей подзадачи
            # ...
            last_err = err  # запоминаем текущую ошибку, для Д-состовляющей регулятора
            arduino.set_angle(angle)  # поворачиваем колёса на рассчитанный угол

except KeyboardInterrupt as e:  # обработка остановки программы сочетанием клавиш Ctrl+C,
    print('Program stopped!', e)  # необходима для корректного завершения программы при остановке через терминал

# Остановка автомобиля и разблокировка доступа к камере.
arduino.stop()
arduino.set_angle(90)
cap.release()
