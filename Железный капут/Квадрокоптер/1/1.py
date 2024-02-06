import rospy
from gs_module import CargoController
from gs_flight import FlightController, CallbackEvent
from gs_board import BoardManager
from time import sleep


rospy.init_node("flight_test_node") # инициализируем ноду


board = BoardManager() # создаем объект бортового менеджера
cargo = CargoController() # создаем объект управления модуля магнитного захвата


navigate = input() # ввод направления пиктограмм
first_obj = input() # ввод названия первой пиктограммы
second_obj = input() # ввод названия второй пиктограммы


# переменные отвечающие за текущий цвет светодиодов в формате RGB
color1 = 0 # красный канал
color2 = 0 # зеленый канал
color3 = 0 # синий канал


# константы красного цвета светодиодов в формате RGB
color_red1 = 255
color_red2 = 0
color_red3 = 0


# константы зеленого цвета светодиодов в формате RGB
color_green1 = 0
color_green2 = 255
color_green3 = 0


# константы фиолетового цвета светодиодов в формате RGB
color_purple1 = 255
color_purple2 = 0
color_purple3 = 255


# константы синего цвета светодиодов в формате RGB
color_blue1 = 0
color_blue2 = 0
color_blue3 = 255


# Определяем цвет индикации:
# Проверяем направление меток - вертикальное и горизонтальное
# В зависимости от направления проверяем название меток.
# присваиваем переменным текущего цвета значения констант нужного цвета
if navigate == "vertically":
   if first_obj == "eye":
       if second_obj == "eye" or second_obj == "brain":
           color1 = color_purple1
           color2 = color_purple2
           color3 = color_purple3
       elif second_obj == 'map':
           color1 = color_green1
           color2 = color_green2
           color3 = color_green3
   elif first_obj == 'brain':
       if second_obj == 'eye':
           color1 = color_purple1
           color2 = color_purple2
           color3 = color_purple3
       elif second_obj == 'brain' or second_obj == 'map':
           color1 = color_blue1
           color2 = color_blue2
           color3 = color_blue3
   elif first_obj == 'map':
       if second_obj == 'eye' or second_obj == 'map':
           color1 = color_green1
           color2 = color_green2
           color3 = color_green3
       elif second_obj == 'brain':
           color1 = color_blue1
           color2 = color_blue2
           color3 = color_blue3
       elif second_obj == 'battery':
           color1 = color_red1
           color2 = color_red2
           color3 = color_red3
   elif first_obj == 'battery':
       if second_obj == 'map' or second_obj == 'battery':
           color1 = color_red1
           color2 = color_red2
           color3 = color_red3
   else:
       print("NONE")


elif navigate == "horizontally":
   if first_obj == "eye":
       if second_obj == "eye" or second_obj == "brain":
           color1 = color_purple1
           color2 = color_purple2
           color3 = color_purple3
       elif second_obj == 'map':
           color1 = color_green1
           color2 = color_green2
           color3 = color_green3
   elif first_obj == 'brain':
       if second_obj == 'eye':
           color1 = color_purple1
           color2 = color_purple2
           color3 = color_purple3
       elif second_obj == 'brain' or second_obj == 'map':
           color1 = color_blue1
           color2 = color_blue2
           color3 = color_blue3
   elif first_obj == 'map':
       if second_obj == 'eye' or second_obj == 'map':
           color1 = color_green1
           color2 = color_green2
           color3 = color_green3
       elif second_obj == 'brain':
           color1 = color_blue1
           color2 = color_blue2
           color3 = color_blue3
       elif second_obj == 'battery':
           color1 = color_red1
           color2 = color_red2
           color3 = color_red3
   elif first_obj == 'battery':
       if second_obj == 'map' or second_obj == 'battery':
           color1 = color_red1
           color2 = color_red2
           color3 = color_red3
   else:
       print("NONE")


coordinates = [ # массив точек, составляющих маршрут
   [3.7, 3.6, 1.4],
   [3.8, 2.8, 2.1],
   [3, 2.7, 1.8],
   [2.8, 3.8, 1.6],
   [2, 3.9, 1.6],
   [2.1, 4, 1.6]
]


run = True # переменная отвечающая за работу программы
position_number = 0 # счетчик точек


cargo.on() # включаем магнитный захват


def callback(event): # функция обработки событий Автопилота
   global ap
   global run
   global coordinates
   global position_number


   event = event.data


   # блок обработки события запуска двигателя
   if event == CallbackEvent.ENGINES_STARTED:
       print("engine started - - - - - - - - -")
       ap.takeoff() # отдаем команду взлета


   # блок обработки события завершения взлета
   elif event == CallbackEvent.TAKEOFF_COMPLETE:
       print("takeoff complite")


       # магание подсветкой 4 раза
       for i in range(4):
           cargo.changeAllColor(color1, color2, color3)
           sleep(4)
           cargo.changeAllColor()
           sleep(4)


       position_number = 0 # обнуляем счетчик точек в маршруте
       # перелет в следующую точку маршрута
       ap.goToLocalPoint(coordinates[position_number][0],
                                             coordinates[position_number][1],
                                             coordinates[position_number][2])




    # блок обработки события завершения перелета в точку
   elif event == CallbackEvent.POINT_REACHED:
       print("point {} reached".format(position_number))
       position_number += 1 # увеличиваем значение счетчика точек в маршруте
       # если в маршруте еще есть точки, то летим на следующую, если нет выполняем посадку
       if position_number < len(coordinates):
           # перелет в следующую точку маршрута
           ap.goToLocalPoint(coordinates[position_number][0],
                                                 coordinates[position_number][1],
                                                 coordinates[position_number][2])




       else:
           ap.landing() # отдаем команду посадки


   elif event == CallbackEvent.COPTER_LANDED: # блок обработки события приземления
       print("finish programm") # прекращаем программу
       run = False


ap = FlightController(callback) # создаем объект управления полета

once = False # переменная отвечающая за первое вхождение в начало программы

# цикл работает пока ROS включен или пока переменная run равна True
# цикл необходим для того чтобы можно было получать события от автопилота
while not rospy.is_shutdown() and run:
   if board.runStatus() and not once: # проверка подключения RPi к Пионеру
       print("start programm")
       ap.preflight() # отдаем команду выполнения предстартовой подготовки
       once = True
   pass