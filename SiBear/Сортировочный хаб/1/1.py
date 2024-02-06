
import numpy as np
import cv2

# Определение диапазонов цветов для каждого цвета
h_min_red = np.array((0, 203, 227), np.uint8)
h_max_red = np.array((33, 255, 255), np.uint8)

h_min_green = np.array((42, 54, 109), np.uint8)
h_max_green = np.array((90, 182, 255), np.uint8)

h_min_blue = np.array((102, 102, 182), np.uint8)
h_max_blue = np.array((144, 173, 255), np.uint8)

h_min_white = np.array((0, 0, 236), np.uint8)
h_max_white = np.array((255, 26, 255), np.uint8)

cap = cv2.VideoCapture(0)

# Флаг для определения начала работы программы
flg_is_nachalo = True


# 1-red  2-green 3-blue 4-white

# Функция для поиска ближайшего центра полосы
def find_closest(x, y, group_id):
    global centers, groups, in_group
    # Инициализируем переменные для хранения минимального расстояния, координат и цвета полос
    mn = 100000
    mn_x = 0
    mn_y = 0
    mn_color = ""
    # Проходим по всем центрам масс полос каждого цвета
    for i in centers:
        for x1, y1 in centers[i]:
            # Если координаты текущего центра масс не равны координатам исходной полоски
            if x1 != x and y1 != y:
                # Вычисляем расстояние между их центрами
                val = abs(x - x1) + abs(y - y1)
                # Если расстояние меньше минимального и меньше 50 и центр масс не входит в другую группу,
                # то обновляем минимальное расстояние, координаты и цвет кубика
                if val < mn and val < 50 and not in_group[str(x1) + str(y1)]:
                    mn = val
                    mn_color = i
                    mn_x = x1
                    mn_y = y1

    # Если минимальное расстояние не было обновлено, то выходим из функции
    if mn == 100000:
        return
    # Если цвет полосы не был добавлен в текущую группу, то добавляем его и создаем список для хранения координат полос этого цвета
    if mn_color not in groups[group_id]:
        groups[group_id][mn_color] = []
    # Добавляем координаты полосы в список полос этого цвета в текущей группе
    groups[group_id][mn_color].append([mn_x, mn_y])
    # Помечаем центр полосы, как входящий в группу
    in_group[str(mn_x) + str(mn_y)] = True


# Функция для проверки, заполнена ли группа полос. Возвращает количество полос в группе
def chech_group_is_full(group_id):
    global groups
    cnt = 0
    for i in groups[group_id]:
        cnt += len(groups[group_id][i])
    return cnt


# Основной цикл программы
while cv2.waitKey(1) != 27:
    # Получение изображения с камеры и преобразование его в формат HSV
    flg, img = cap.read()
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Инициализация списков для хранения центров полос и групп центров полос
    centers = {"red": [], "green": [], "blue": [], "white": []}
    groups = [{}]
    in_group = {}

    # Создание бинарных масок для каждого цвета
    mask_red = cv2.inRange(hsv, h_min_red, h_max_red)
    mask_green = cv2.inRange(hsv, h_min_green, h_max_green)
    mask_blue = cv2.inRange(hsv, h_min_blue, h_max_blue)
    mask_white = cv2.inRange(hsv, h_min_white, h_max_white)
    # cv2.imshow("RED", mask_red)
    # cv2.imshow("GREEN", mask_green)
    # cv2.imshow("BLUE", mask_blue)
    # cv2.imshow("WHITE", mask_white)

    # Нахождение контуров в каждой маске
    contours_red, hierarchy_red = cv2.findContours(mask_red, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours_green, hierarchy_green = cv2.findContours(mask_green, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours_blue, hierarchy_blue = cv2.findContours(mask_blue, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours_white, hierarchy_white = cv2.findContours(mask_white, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # Нахождение центров полос для каждого контура и добавление их в соответствующий список
    for cnt in contours_red:
        # Проверяем площадь контура и если она находится в заданном диапазоне, то продолжаем обработку
        if 20 < cv2.contourArea(cnt) < 700:
            # Вычисляем моменты контура
            M = cv2.moments(cnt)
            # Вычисляем координаты центра масс контура
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            # Если центр масс находится ниже 300 пикселей по вертикали, то продолжаем обработку
            if cy > 300:
                continue
            # Добавляем координаты центра масс полосы к списку центров масс красных полос
            centers["red"].append([cx, cy])
            # Помечаем центр масс, как не входящий в группу
            in_group[str(cx) + str(cy)] = False

    for cnt in contours_blue:
        if 20 < cv2.contourArea(cnt) < 700:

            M = cv2.moments(cnt)
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            if cy > 300:
                continue
            centers["blue"].append([cx, cy])
            in_group[str(cx) + str(cy)] = False
    for cnt in contours_green:
        if 20 < cv2.contourArea(cnt) < 700:

            M = cv2.moments(cnt)
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            if cy > 300:
                continue
            centers["green"].append([cx, cy])
            in_group[str(cx) + str(cy)] = False
    for cnt in contours_white:
        if 20 < cv2.contourArea(cnt) < 700:
            M = cv2.moments(cnt)
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            if cy > 300:
                continue
            centers["white"].append([cx, cy])
            in_group[str(cx) + str(cy)] = False

    # Группировка центров полос на основе их близости друг к другу и их цвета
    group_id = 0
    for i in centers:
        # Проходим по всем центрам масс полосок каждого цвета
        for x, y in centers[i]:
            # Если центр масс уже входит в какую-то группу, то пропускаем его
            if in_group[str(x) + str(y)]:
                continue
            # Создаем новую группу и добавляем в нее текущую полосу
            groups[group_id][i] = []
            groups[group_id][i].append([x, y])
            # Помечаем центр масс полосы, как входящую в группу
            in_group[str(x) + str(y)] = True
            # Ищем две ближайших полосы и добавляем их в текущую группу
            find_closest(x, y, group_id)
            find_closest(x, y, group_id)
            # Увеличиваем идентификатор группы на 1 и добавляем пустой словарь для следующей группы
            group_id += 1
            groups.append({})

    # Вывод в консоль маркировок и отметка на изображении их точкой
    print(groups)
    for i in groups:
        for j in i:
            print(i[j][0][0])
            cv2.circle(img, (i[j][0][0], i[j][0][1]), radius=5, color=(0, 255, 255), thickness=-1)
            break
    cv2.imshow("IMG", img)
