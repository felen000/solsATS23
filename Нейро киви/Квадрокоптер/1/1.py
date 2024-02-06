#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import rospy
from gs_flight import FlightController, CallbackEvent
from gs_board import BoardManager
from gs_module import CargoController


rospy.init_node("task_1") # инициализируем ноду


coordinates = [ # массив точек маршрута
   [3.83, 2.8, 1.9],
   [3, 2.8, 1.9],
   [3, 3.9, 1.9],
   [1.8, 3.9, 1.9],
   [3, 3.9, 1.9],
   [3, 1.64, 1.9],
   [1.82, 1.64, 1.9]
]


pic1 = input() # ввод названия первой пиктограммы
pic2 = input() # ввод названия второй пиктограммы
dir = input() # ввод направления пиктограмм


nnow = [255, 255, 255] # текущий цвет индикации


def recipient(p1, p2):
# """ Функция, отвечающая за определение подразделения получателя в зависимости
#        от введённых пиктограмм.  Возвращает число - номер подразделения получателя.
#        Если введена неверная комбинация пиктограмм, то возвращает -1
# """


    color = -1
    if (p1 == 'battery' and p2 == 'battery') or (p1 == 'battery' and p2 == 'map') or (p2 == 'battery' and p1 == 'map'):
       color = 4
    elif (p1 == 'map' and p2 == 'map') or (p1 == 'map' and p2 == 'eye') or (p2 == 'map' and p1 == 'eye'):
       color = 2
    elif (p1 == 'map' and p2 == 'brain') or (p2 == 'map' and p1 == 'brain') or (p1 == 'brain' and p2 == 'brain'):
       color = 1
    elif (p1 == 'eye' and p2 == 'eye') or (p1 == 'eye' and p2 == 'brain') or (p2 == 'eye' and p1 == 'brain'):
       color = 3
    return color


run = True # переменная отвечающая за работу программы
position_number = 0 # счетчик точек


def callback(event): # функция обработки событий Автопилота
   global ap
   global run
   global coordinates
   global position_number


   event = event.data
   # блок обработки события запуска двигателя
   if event == CallbackEvent.ENGINES_STARTED:
       ap.takeoff() # отдаем команду взлета
   # блок обработки события завершения взлета
   elif event == CallbackEvent.TAKEOFF_COMPLETE:
       cl = recipient(pic1, pic2) # получаем номер подразделения получателя
       # задаём цвет индикации в соответствии с номером подразделения получателя груза
       if cl == 1:
           nnow = [0, 0, 255]
       elif cl == 2:
           nnow = [0, 255, 0]
       elif cl == 3:
           nnow = [255, 0, 255]
       elif cl == 4:
           nnow = [255, 0, 0]


       # мигаем индикацией 4 раза
       for i in range(4):
          # включаем индикацию
           cargo.changeAllColor(nnow[0], nnow[1], nnow[2])
           rospy.sleep(4) # выключаем индикацию
           cargo.changeAllColor() # включаем индикацию
           rospy.sleep(4) # выключаем индикацию


       position_number = 0 # обнуляем счетчик точек в маршруте
       # перелет в первую точку
       ap.goToLocalPoint(coordinates[position_number][0],
                                             coordinates[position_number][1],
                                             coordinates[position_number][2])
   # блок обпатоки события завершения перелета в точку
   elif event == CallbackEvent.POINT_REACHED:
        position_number += 1 # увеличиваем значение счетчика точек в маршруте
        if position_number < len(coordinates):
        # если в маршруте еще есть точки, то летим к следюущей,
           ap.goToLocalPoint(coordinates[position_number][0],
                                                  coordinates[position_number][1],
                                                  coordinates[position_number][2])
        else:         # если точек в маршруте больше нет, то
           ap.landing() # отдаем команду посадки
   elif event == CallbackEvent.COPTER_LANDED:
       print("finish programm")
       run = False # прекращаем программу


board = BoardManager() # создаем объект бортового менеджера
ap = FlightController(callback) # создаем объект управления полета
cargo = CargoController() # создаем объект управления модуля магнитного захвата


once = False # переменная отвечающая за первое вхождение в начало программы


# цикл работает пока ROS включен или пока переменная run равна True
# цикл необходим для того, чтобы можно было получать события от автопилота
while not rospy.is_shutdown() and run:
   if board.runStatus() and not once: # проверка подключения RPi к Пионеру
       cargo.on() # включаем магнитный захват
       print("start programm")
       ap.preflight() # отдаем команду выполнения предстартовой подготовки
       once = True
   pass


