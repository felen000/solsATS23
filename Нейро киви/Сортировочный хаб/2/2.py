
import cv2
import numpy as np

# Задаем координаты углов квадрата,
# в который будем преобразовывать прямоугольное изображение найденных грузов
sdr = np.float32([
    [0, 0],
    [300, 0],
    [0, 300],
    [300, 300]
])

cap = cv2.VideoCapture(0)
font = cv2.FONT_HERSHEY_COMPLEX
i = 0
while i < 20:
    ret, frame = cap.read()
    if not ret:
        break

    img = cv2.resize(frame, (480, 400))  # изменяем размер изображения
    img = img[:250, :]  # оставляем изображение только зоны разгрузки
    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)  # переводим изображение в HSV формат
    # зона разгрузки имеет преимущественно чёрный цвет
    # бинаризуем изображение так, чтобы грузы и цветные полосы стали белыми,
    # а фон стал чёрным
    black_m = cv2.inRange(img, (55, 55, 55), (255, 255, 255))
    # бинаризуем HSV-изображение так, чтобы цветные полосы стали чёрными, а фон и грузы белым
    binimg = cv2.inRange(hsv_img, (0, 0, 0), (255, 140, 255))
    # находим пересечение масок, на нём останутся только очертания грузов
    binimg = cv2.bitwise_and(binimg, black_m)
    # ищем на изображении контуры
    contours, _ = cv2.findContours(binimg, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # сортируем контуры по убыванию площади
    contours = sorted(contours, key=cv2.contourArea, reverse=True)

    # Инициализируем переменные для подсчета количества грузов, полос маркировки и
    # хранения координат их центров
    kubs = 0
    lines = []
    x_coordinates = []
    y_coordinates = []

    # Перебираем все контуры и проверяем, соответствует ли их размер заданным параметрам для груза.
    for j in range(len(contours)):
        (x, y, w, h) = cv2.boundingRect(contours[j])
        #  Высота и ширина контура должны лежать в определённых пределах и
        #  не сильно отличаться друг от друга, т.е. быть близким к квадрату
        if 25 < w < 60 and 25 < h < 60 and abs(w - h) <= 7:
            # Устанавливаем флаг, обнаружения груза
            # Если контур соответствует уже детектированному грузу, то флаг = True
            same = False
            # сравниваем координаты "центра" контура с координатами "центров" уже обнаруженных грузов
            for ii in range(len(x_coordinates)):
                if abs((x + w // 2) - x_coordinates[ii]) < 10 and abs((y + h // 2) - y_coordinates[ii]) < 10:
                    # если центры ближе 10 пикселей по вертикали и
                    # ближе 10 пикселей по горизонтали, то контур указывает на уже обнаруженный груз
                    same = True  # выставляем соответствующий флаг
                    break  # прекращаем перебор координат уже найденных грузов

            if not same:  # Если контур указывает на ранее не обнаруженый груз,
                # то увеличиваем счетчик обнаруженных грузов
                kubs += 1
                # сохраняем координаты центра груза
                x_coordinates.append(x + w // 2)
                y_coordinates.append(y + h // 2)

                # Находим минимальный прямоугольник, описывающий контур
                # Прямоугольник может быть наклонённым
                rect = cv2.minAreaRect(contours[j])
                box = cv2.boxPoints(rect)  # получаем координаты вершин прямоугольника
                box = np.int0(box)  # переобразуем координаты в целочисленные значения
                box = list(box)
                box[2], box[3] = box[3], box[2]
                box = np.float32(box)
                # переобразуем прямоугольник в квадрат,
                # за счёт этого полосы станут ориентированы горизонтально, либо вертикально
                mm = cv2.getPerspectiveTransform(box, sdr)
                marker = cv2.warpPerspective(img, mm, (300, 300), flags=cv2.INTER_LINEAR)

                # Создаем различные маски по цветам
                # Это реально находит контуры цветных полос
                red_mask = cv2.inRange(marker, (0, 0, 140), (90, 110, 255))
                green_mask = cv2.inRange(marker, (50, 110, 0), (180, 255, 90))
                blue_mask = cv2.inRange(marker, (180, 90, 0), (255, 210, 100))
                mask = cv2.bitwise_or(green_mask, red_mask)
                mask = cv2.bitwise_or(blue_mask, mask)
                white_mask = cv2.inRange(marker, (230, 230, 230), (255, 255, 255))
                mask = cv2.bitwise_or(white_mask, mask)
                hsv_mask = mask

                # Находим контуры полос в маркировке
                contours1, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                contours1 = sorted(contours1, key=cv2.contourArea, reverse=True)

                for k in range(len(contours1)):
                    (x1, y1, w1, h1) = cv2.boundingRect(contours1[k])
                    if (w1 > 40 and h1 > 40):
                        cv2.rectangle(marker, (x1, y1), (x1 + w1, y1 + h1), (0, 255, 0), 2)
                        mx1, my1 = min(x1, mx1), min(y1, my1)
                        mx2, my2 = max(x1 + w1, mx2), max(y1 + h1, my2)
                # Рисуем маркер на изображении
                cv2.rectangle(marker, (mx1, my1), (mx2, my2), (255, 0, 255), 1)
                marker = marker[my1:my2, mx1:mx2]

                # Поворачиваем маркер, если он неправильно повёрнут
                w2 = abs(mx1 - mx2)
                h2 = abs(my1 - my2)
                if abs(my2 - my1) > (mx2 - mx1):
                    marker = cv2.rotate(marker, cv2.ROTATE_90_COUNTERCLOCKWISE)
                    w2, h2 = h2, w2

                # Инициализуруем переменные для подсчета количество цветных полос: красных, синих и зелёных соответсвенно
                rl = 0
                bl = 0
                gl = 0

                # Считаем количество красных полос
                # Если сумма пикселей на красной маске больше определённого порога,
                # считаем, что красных полос 3, если сумма пикселей меньше этого порога,
                # но больше другого, считаем, что полос 2 и так далее.
                if np.sum(red_mask) > 6500000:
                    rl = 3
                elif 6500000 > np.sum(red_mask) > 4000000:
                    rl = 2
                elif np.sum(red_mask) > 1500000:
                    rl = 1
                else:
                    rl = 0

                # Считаем количество зелёных полос
                # Если сумма пикселей на зеленой маске больше определённого порога,
                # считаем, что зеленых полос 3, если сумма пикселей меньше этого порога,
                # но больше другого, считаем, что полос 2 и так далее.
                if np.sum(green_mask) > 6500000:
                    gl = 3
                elif 6500000 > np.sum(green_mask) > 4000000:
                    gl = 2
                elif np.sum(green_mask) > 600000:
                    gl = 1
                else:
                    gl = 0

                # Считаем количество синих полос
                # Если сумма пикселей на синей маске больше определённого порога,
                # считаем, что синих полос 3, если сумма пикселей меньше этого порога,
                # но больше другого, считаем, что полос 2 и так далее.
                if np.sum(blue_mask) > 5500000:
                    bl = 3
                elif 5500000 > np.sum(blue_mask) > 3000000:
                    bl = 2
                elif np.sum(blue_mask) > 600000:
                    bl = 1
                else:
                    bl = 0

                # Если есть 2 полосы с одинаковым цветом
                if gl == 2 or rl == 2 or bl == 2:
                    # Выделяем груз на изображении красных цветом и подписываем, что он относится к 1 группе
                    cv2.rectangle(img, (x, y), (x + w, y + h), (0, 0, 255), 2)
                    cv2.putText(img, '1', (x + 1, y + 1), font, 0.5, (0, 0, 255))
                # Если на изображении груза много пикселей белого цвета
                elif np.sum(white_mask) > 100000:
                    # Выделяем груз на изображении желтым цветом и подписываем, что он относится ко 2 группе
                    cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 255), 2)
                    cv2.putText(img, '2', (x + 1, y + 1), font, 0.5, (0, 255, 255))
                # Если на маркере нет зелёных полос
                elif gl == 0:
                    # Выделяем груз на изображении фиолетовым цветом и подписываем, что он относится к 3 группе
                    cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 255), 2)
                    cv2.putText(img, '3', (x + 1, y + 1), font, 0.5, (255, 0, 255))
                # Если условия выше не соблюдается
                else:
                    # Выделяем груз на изображении синим цветом и подписываем, что он не относится ни к одной из групп
                    # и должен быть поднят к захвату квадрокоптера
                    cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 0), 2)
                    cv2.putText(img, 'lift', (x + 1, y + 1), font, 0.5, (255, 0, 0))
                print(str(j) + '   ' + str(gl) + '     ' + str(np.sum(green_mask)))

                cv2.imshow("mask" + str(j), hsv_mask)
                cv2.imshow("mmmark" + str(j), marker)
    cv2.imshow("bin", binimg)
    cv2.imshow("img", img)
    key = cv2.waitKey(1)
    if key == ord('n'):
        cv2.destroyAllWindows()
        i += 1
    if key == ord('q'):
        cv2.destroyAllWindows()
        break
