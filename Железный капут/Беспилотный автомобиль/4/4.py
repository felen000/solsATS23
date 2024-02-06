import time
import yolopy  # библиотека для инференса моделей Yolo на NPU
import cv2
import numpy as np
from arduino import Arduino  # класс из базового кода для общения с Arduino
from utils import *  # набор функций, предоставленных участникам

CAR_SPEED = 1446   # Начальная скорость автомобиля
ARDUINO_PORT = '/dev/ttyUSB0'  # Порт, к которому подключена Arduino
CAMERA_ID = '/dev/video0'  # Идентификатор камеры

# параметры ПД-регулятора
KP = 0.33
KD = 0.25
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


# Функция бинаризации изображения, для поиска линий разметки на изобраении
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


# Установка начальной скорости автомобиля
arduino.set_speed(CAR_SPEED)
p_flag = False  # флаг начала манёвра. Как только замечен знак парковки флаг переключается в True
try:
    while True:
        # Чтение кадра с камеры
        ret, frame = cap.read()
        # Изменение размера кадра для обработки детектором
        yol = cv2.resize(frame, (640, 640))
        # Обнаружение объектов на изображении с помощью модели YOLOv4-tiny
        obj = []
        classes, scores, boxes = model.detect(yol)
        if len(classes) != 0:  # если детектировано больше 0 классов,
            obj = [classes_list[i] for i in classes]  # получаем список их имён
            
        if ('parking' in obj) and (p_flag == False):  # если знак парковки замечен впервые,
            line_time = time.time()                   # то засекаем время начала манёвра
            p_flag = True                             # и меняем значение флага на True

        # если манёвр уже начался, т.е. мы уже видели знак парковки, то проверяем текущую длительность манёвра
        if p_flag == True:
            if time.time() - line_time <= 3:  # до 3 секунд поворачиваем направо, т.е. выезжаем на обочину
                arduino.set_angle(60)
            # от 3 до 5,5 секунд поворачиваем налево, т.е. занимаем парковочное место
            elif time.time() - line_time <= 5.5:
                arduino.set_angle(115)
            elif time.time() - line_time > 5.5:  # после 5,5 секунд прекращаем манёвр и останавливаемся
                break
        else:  # если мы ещё не видели знак парковки, то детектируем линии разметки и движемся между ними
            img = cv2.resize(frame, SIZE)  # приводим изображение к удобному для вычислений размеру
            binary = binarize2(img, THRESHOLD)  # бинаризуем изображение для поиска белых линий
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
            arduino.set_angle(angle)  # поворачиваем колёса на рассчитанный угол

except KeyboardInterrupt as e:  # обработка остановки программы сочетанием клавиш Ctrl+C,
    print('Program stopped!', e)  # необходима для корректного завершения программы при остановке через терминал

# Остановка автомобиля и разблокировка доступа к камере.
arduino.stop()
arduino.set_angle(90)
cap.release()
