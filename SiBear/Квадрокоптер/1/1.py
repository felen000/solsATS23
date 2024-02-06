#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from gs_flight import FlightController, CallbackEvent
from gs_board import BoardManager
from gs_module import CargoController


rospy.init_node("test_node") # инициализируем ноду
run = True # переменная отвечающая за работу программы
coord = [ # массив точек маршрута
   [3.83, 3.28, 1.2],
   [3.1, 3.28, 1.2],
   [3.1, 3.9, 1.2],
   [2.2, 3.9, 1.2]
]
pos = 0 # счетчик точек


# участники вводили вместо названия меток их номер:
# 1 - глаз
# 2 - мозг
# 3 - карта
# 4 - солнечная батарея
pin1 = int(input()) # ввод названия первого логотипа
pin2 = int(input()) # ввод названия второго логотипа
tabl = [ # таблица соответствия двух пиктограмм и получателя
   [3, 3, 2, 0],
   [3, 1, 1, 0],
   [2, 1, 2, 4],
   [0, 0, 4, 4]
]


color = tabl[pin1][pin2] # получаем номер текущего подразделения получателя




def callback(event): # функция обработки событий Автопилота
   global ap
   global run
   global cargo
   global pos
   event = event.data
   if event == CallbackEvent.ENGINES_STARTED: # блок обработки события запуска двигателя
       ap.takeoff() # отдаем команду взлета
   elif event == CallbackEvent.TAKEOFF_COMPLETE:
       pos = 0 # обнуляем счетчик точек в маршруте


       # производим мигание 4 раза
       for i in range(4):
           #  устанавливаем цвет индикации в соответствии с номером получателя груза
           if color == 1:
               cargo.changeAllColor(0.0, 0.0, 255.0)
           if color == 2:
               cargo.changeAllColor(0.0, 255.0, 0.0)
           if color == 3:
               cargo.changeAllColor(255.0, 0.0, 255.0)
           if color == 4:
               cargo.changeAllColor(255.0, 0.0, 0.0)
           rospy.sleep(4) # включаем светодиоды на 4 секунды
           cargo.changeAllColor() # выключаем светодиоды
           rospy.sleep(4) # выключаем светодиоды на 4 секунды
       ap.goToLocalPoint(coord[0][0], coord[0][1], coord[0][2]) # перелет в первую точку
   # блок обработки события завершения перелета в точку
   elif event == CallbackEvent.POINT_REACHED:
       pos += 1 # увеличиваем значение счетчика точек в маршруте
       # если в маршруте еще есть точки, то летим на следующую, если нет выполняем посадку
       if pos < len(coord):
           # перелет в следующую точку маршрута
           ap.goToLocalPoint(coord[pos][0], coord[pos][1], coord[pos][2])
       else:
           ap.landing() # отдаем команду посадки
           cargo.off() # выключаем магнит
           run = False # выключаем программу


ap = FlightController(callback) # создаем объект управления полетом
cargo = CargoController() # создаем объект управления модулем магнитного захвата
board = BoardManager() # создаем объект бортового менеджера
once = False # переменная отвечающая за первое вхождение в начало программы


# цикл работает пока ROS включен или пока переменная run равна True
# цикл необходим для того чтобы можно было получать события от автопилота
while not rospy.is_shutdown() and run:
   if board.runStatus() and not once: # проверка подключения RPi к Пионеру
       cargo.on() # включаем магнитный захват
       ap.preflight() # отдаем команду выполнения предстартовой подготовки
       once = True
   pass
