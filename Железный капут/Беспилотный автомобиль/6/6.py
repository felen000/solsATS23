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

# Установка начальной скорости автомобиля
arduino.set_speed(CAR_SPEED)

# Флаг наличия пешехода или знака "пешеходный переход" в кадре.
s_flag = False
p_flag = False  # Флаг наличия пешехода на дороге.

try:
    while True:
        ret, frame = cap.read()  # чтение кадра с камеры
        if not ret:
            print("Have't frame")
            break

        # Изменение размера кадра для обработки детектором
        yol = cv2.resize(frame, (640, 640))
        # Обнаружение объектов на изображении с помощью модели YOLOv4-tiny
        classes, scores, boxes = model.detect(yol)
        obj = []
        if len(classes) != 0:
            obj = [classes_list[i] for i in classes]

        # Если обнаруже пешеход или знак "пешеходный переход",
        if 'pedestrian crossing' in obj or 'human' in obj:
            s_flag = True  # то выставляем соответствующий флаг в True
            # Если площадь пешехода больше некого порогового значения, то пешеход близко
            if 'human' in obj and (boxes[classes.index(2)][2] * boxes[classes.index(2)][3]) > 15500:
                # если координата X пешехода лежит в определённом интервале,
                if 380 > boxes[classes.index(2)][0] > 260:
                    # то считаем, что пешеход на дороге и
                    arduino.stop()  # останавливаемся
                    p_flag = True   # выставляем соответствующий флаг
                else:
                    # пешеход не обочине, возвращаем флаг в исходное положение
                    p_flag = False

        else:  # в кадре нет ни пешеходов, ни знака "пешеходный переход"
            s_flag = False
        if 'human' not in obj:  # если пешеход исчез из кадра
            p_flag = False  # возвращаем флаг в исходное положение

        # Если пешехода на дороге нет
        if not p_flag:
            if not s_flag:  # и на обочине объектов нет, то
                arduino.set_speed(1435)  # движемся с нормальной скоростью
            else:  # объект на обочине
                arduino.set_speed(1445)  # снижаем скорость

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

