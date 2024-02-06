#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import rospy
from gs_flight import FlightController, CallbackEvent
from gs_module import CargoController
from gs_board import BoardManager
import time
from threading import Thread

rospy.init_node("flight_test_node")  # инициализируем ноду

run = True  # переменная отвечающая за работу программы
first_building_point = [1.8, 3.9, 2]  # координаты первого здания

coordinates = [[4.27, 3.3, 2.5],  # первая точка маршрута
               [3.5, 3.3, 2.5],  # вторая точка маршрута
               [3.5, 4, 2.5],  # третья точка маршрута
               [2.3, 4, 2.5]]  # четвертая точка маршрута
position_number = 0  # счетчик точек


# функция, определяющая подразделение получатель в зависимости от полученных пиктограмм
def color(logo_1, logo_2):
    if (logo_1 == "eye" and logo_2 == "eye") or (logo_1 == "eye" and logo_2 == "brain"):
        return 3
    if (logo_1 == "eye" and logo_2 == "map") or (logo_1 == "map" and logo_2 == "map"):
        return 2
    if (logo_1 == "brain" and logo_2 == "brain") or (logo_1 == "brain" and logo_2 == "map"):
        return 1
    if (logo_1 == "map" and logo_2 == "battery") or (logo_1 == "map" and logo_2 == "battery"):
        return 4


logo_1 = input()  # ввод названия первой пиктограммы
logo_2 = input()  # ввод названия второй пиктограммы
direction = input()  # ввод направления пиктограмм


def target():  # функция, которую будет выполнять поток
    global cargo
    while True:
        if color(logo_1, logo_2) == 3:
            cargo.changeAllColor(255.0, 0.0, 255.0)  # включаем индикацию фиолетовым цветом
        if color(logo_1, logo_2) == 2:
            cargo.changeAllColor(0.0, 255.0, 0.0)  # включаем индикацию зеленым цветом
        if color(logo_1, logo_2) == 1:
            cargo.changeAllColor(0.0, 0.0, 255.0)  # включаем индикацию синим цветом
        if color(logo_1, logo_2) == 4:
            cargo.changeAllColor(255.0, 0.0, 0.0)  # включаем индикацию красным цветом
        time.sleep(4)  # останавливаем поток на 4 секунду
        cargo.changeAllColor()  # выключаем индикацию
        time.sleep(4)  # останавливаем поток на 4 секунду


indication = Thread(target=target)  # поток, отвечающий за индикацию нужного цвета


def callback(event):  # функция обработки событий Автопилота
    global ap
    global run
    global position_number

    event = event.data
    # блок обработки события запуска двигателя
    if event == CallbackEvent.ENGINES_STARTED:
        print("Copter armed")
        ap.takeoff()  # отдаем команду взлета
    # блок обработки события завершения взлета
    elif event == CallbackEvent.TAKEOFF_COMPLETE:
        print("Lightings on")
        indication.start()  # запускаем индикацию нужного цвета
        position_number = 0  # обнуляем счетчик точек в маршруте
        # перелет в первую точку
        ap.goToLocalPoint(coordinates[position_number][0],
                          coordinates[position_number][1],
                          coordinates[position_number][2])


    # блок обработки события завершения перелета в точку
    elif event == CallbackEvent.POINT_REACHED:
        print("point {} reached".format(position_number))
        position_number += 1  # увеличиваем значение счетчика точек в маршруте
        # если в маршруте еще есть точки, то летим на следующую
        if position_number < len(coordinates):
            # перелет в следующую точку маршрута
            ap.goToLocalPoint(coordinates[position_number][0],
                              coordinates[position_number][1],
                              coordinates[position_number][2])
        else:
            run = False  # завершаем работу программы


    # блок обработки события приземления
    elif event == CallbackEvent.COPTER_LANDED:
        print("Flight plan finished")
        run = False  # завершаем работу программы


board = BoardManager()  # создаем объект бортового менеджера
cargo = CargoController()  # создаем объект управления модуля магнитного захвата
ap = FlightController(callback)  # создаем объект управления полета

once = False  # переменная отвечающая за первое вхождение в начало программы

# цикл работает пока ROS включен или пока переменная run равна True
# цикл необходим для того чтобы можно было получать события от автопилота
while not rospy.is_shutdown() and run:
    if board.runStatus() and not once:  # проверка подключения RPi к Пионеру
        print("Starting...")
        print("Cargo on")
        cargo.on()  # включаем магнитный захват
        ap.preflight()  # отдаем команду выполнения предстартовой подготовки
        once = True
    pass
