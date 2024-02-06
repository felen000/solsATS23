import time
import yolopy  # библиотека для инференса моделей Yolo на NPU
import cv2
from arduino import Arduino  # класс из базового кода для общения с Arduino


CAR_SPEED = 1500  # Начальная нулевая скорость автомобиля
angle = 90  # Начальный нулевой угол поворота рулевых колёс автомобиля
ARDUINO_PORT = '/dev/ttyUSB0'  # Порт, к которому подключена Arduino
CAMERA_ID = '/dev/video0'  # Идентификатор камеры


# Инициализация обмена данными с Arduino через UART
arduino = Arduino(ARDUINO_PORT, baudrate=115200, timeout=10)
time.sleep(1)

# Инициализация работы с камерой
cap = cv2.VideoCapture(CAMERA_ID, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

# Загрузка модели детектора для обнаружения объектов YOLOv4-tiny
model_file = 'yolov4-tiny-obj_best_uint8.tmfile'  # Относительный путь к квантованной модели детектора
model = yolopy.Model(model_file, use_uint8=True, use_timvx=True, cls_num=10)
#                                тип квантования,     бэкэнд,    число классов

classes_list = [   # список названий классов детектора
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

# Установка начальной скорости автомобиля и угла поворота колёс, равных 0
arduino.set_speed(CAR_SPEED)
arduino.set_angle(angle)

try:
    while True:
        # Чтение кадра с камеры
        ret, frame = cap.read()
        # Изменение размера кадра для обработки детектором
        yol = cv2.resize(frame, (640, 640))
        # Обнаружение объектов на изображении с помощью модели YOLOv4-tiny
        obj = []
        classes, scores, boxes = model.detect(yol)
        print(len(classes))  # вывод количества детектированных классов
        if len(classes) != 0:  # если детектировано больше 0 классов,
            obj = [classes_list[i] for i in classes]
            print(classes)     # то выводим название каждого детектоированного класса

except KeyboardInterrupt as e:  # обработка остановки программы сочетанием клавиш Ctrl+C,
    print('Program stopped!', e)  # необходима для корректного завершения программы при остановке через терминал

# Остановка автомобиля и разблокировка доступа к камере.
arduino.stop()
arduino.set_angle(90)
cap.release()
