
import cv2
import numpy as np
import math

# параметры для фильтрации контуров грузов
min_area = 50 * 50
max_side_error = 20
max_edge_error = 5
rotating_side = 5

# Класс для вычисления расстояния Левейнштейна между 2 строками
class Levenshtein(object):
    def __init__(self, source_word, target_word):
        self.insert_cost = 1.5
        self.delete_cost = 1
        self.substitute_cost = 2

        self.grid = []

        self.source_word = source_word
        self.target_word = target_word

        self.minimum_cost = -1

    def __init_grid(self):
        del self.grid[:]

        for row in range(0, (len(self.source_word) + 1)):

            self.grid.append([0] * (len(self.target_word) + 1))

            self.grid[row][0] = row

            if row == 0:

                for column in range(0, (len(self.target_word) + 1)):
                    self.grid[row][column] = column

        self.minimum_cost = -1

    def calculate(self):
        self.__init_grid()

        for sourceletter in range(0, len(self.source_word)):

            for targetletter in range(0, len(self.target_word)):

                if self.target_word[targetletter] != self.source_word[sourceletter]:
                    total_substitution_cost = self.grid[sourceletter][targetletter] + self.substitute_cost
                else:
                    total_substitution_cost = self.grid[sourceletter][targetletter]

                total_deletion_cost = self.grid[sourceletter][targetletter + 1] + self.delete_cost

                total_insertion_cost = self.grid[sourceletter + 1][targetletter] + self.insert_cost

                self.grid[sourceletter + 1][targetletter + 1] = min(total_substitution_cost, total_deletion_cost,
                                                                    total_insertion_cost)

        self.minimum_cost = self.grid[len(self.source_word)][len(self.target_word)]


# функция, возвращающая список расшифровок грузов на изображении
def get_marks(img):
    # Преобразуем изображение в цветовое пространство HSV и создаем маску hsv_mask
    hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    hsv_mask = cv2.inRange(hsv, (0, 0, 70), (255, 255, 255))

    # Находим контуры на маске и отбираем только те, которые соответствуют маркерам
    contours, _ = cv2.findContours(hsv_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Создаем список, в котором будем хранить контуры
    detected = []
    for c in contours:
        # Находим ограничивающий прямоугольник для контура
        rect = cv2.minAreaRect(c)
        area = rect[1][0] * rect[1][1]
        # Фильтруем контуры по площади и соотношению сторон
        if area < min_area or area > 100 * 100:
            continue
        rect_w, rect_h = rect[1]
        if abs(rect_w - rect_h) > max_side_error:
            continue
        detected.append(rect)

    marks = []  # список координат грузов и расшифровок их цветовых маркировок
    # перебираем найденные грузы
    for j, rect in enumerate(detected):
        w_rect, h_rect = rect[1]

        # формируем изображение груза
        bbox = cv2.boxPoints(rect)
        bbox = np.int0(bbox)
        mask = np.zeros_like(hsv_mask)  # чёрная маска
        # Рисуем белый прямоугольник на чёрной маске, прямоугольник полностью закрывает груз
        cv2.drawContours(mask, [np.int0(bbox)], -1, 255, -1)
        # формируем изображения где все пиксели, кроме пикселей груза чёрные
        cube = cv2.bitwise_and(img, img, mask=mask).astype(np.uint8)
        # вырезаем цветное изображение груза
        cube = cube[min(bbox[:, 1]):max(bbox[:, 1]), min(bbox[:, 0]):max(bbox[:, 0])]

        # Применяем перспективное преобразование, чтобы привести изображение груза к прямоугольному виду
        box = np.copy(bbox)
        box[:, 1] -= min(box[:, 1])
        box[:, 0] -= min(box[:, 0])
        corners = np.float32([[0, 0], [w_rect, 0], [w_rect, h_rect], [0, h_rect]])
        matrix = cv2.getPerspectiveTransform(box.astype(np.float32), corners)
        cube = cv2.warpPerspective(cube, matrix, (int(w_rect), int(h_rect)))

        h_cube, w_cube, _ = cube.shape
        original_cube = np.copy(cube)
        # обрезаем изображение куба со всех сторон
        cube = cube[5:-5, 5:-5]

        # Применяем алгоритм k-средних к пикселям изображения, ищем 4 "средних" цвета
        # Пиксель рассматривается как точка или вектор в трёхмерном пространстве
        Z = cube.reshape((-1,3))
        Z = np.float32(Z)
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
        K = 4
        ret, label, center=cv2.kmeans(Z, K, None, criteria, 10, cv2.KMEANS_RANDOM_CENTERS)
        center = np.uint8(center)
        res = center[label.flatten()]
        # res2 - изображение груза, где все цвета приведены к четырём "средним"
        res2 = res.reshape((cube.shape))
        cube = original_cube

        # Формируем список "средних"цветов, отсортированных по яркости
        unique_colors = set([tuple(x) for y in range(len(res2)) for x in res2[y]])
        unique_colors = list(sorted(unique_colors, key=lambda x: sum(x) / len(x)))
        black_color = unique_colors[0] # самый тёмный цвет, наиболее приближенный к чёрному

        # Создаем маску где чёрные пиксели соответствуют пикселям самого тёмного цвета, остальные пиксели имеют значение 1
        cube_mask = (res2[:, :, 0] == black_color[0]) & (res2[:, :, 1] == black_color[1]) & (res2[:, :, 2] == black_color[2])
        cube_mask = cube_mask.astype(np.uint8)

        # Находим контуры на маске
        contours, _ = cv2.findContours(cube_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Проверяем, что были найдены контуры на маске.
        # Если контуры не найдены, вызываем исключение.
        if len(contours) == 0:
            raise Exception('contours 0')

        # Находим наибольший по площади контур , который не находится на краю груза.
        best_contour = contours[0]
        best_area = -1
        for c in contours:  # перебираем контуры
            points = []
            is_edge = False
            for point in c[:, 0, :]:  # перебираем все точки контура и проверяем,
                                      # что они находятся на удалении от границ изображения
                is_edge = (abs(point[0] - max_edge_error) < 0 or\
                           abs(point[0] + max_edge_error) > w_cube or\
                           abs(point[1] - max_edge_error) < 0 or\
                           abs(point[1] + max_edge_error) > h_cube)
                if not is_edge:
                    points.append([point])  # список точек далёких от края изображения
            # обновляем контур, включив в него только точки далёкие от края изображения
            c = np.array(points).astypвe(np.int64)
            if not c.any():
                continue

            # Ищем самый большой по площади контур
            area = cv2.contourArea(c)
            if area > best_area:
                best_area = area
                best_contour = c

        # Вырезаем изображение из куба
        x, y, w, h = cv2.boundingRect(best_contour)
        mark = cube[y:y+h, x:x+w]

        # Создаем маску, на которой контур маркера выделен белым цветом, а все остальное черным.
        # Вырезаем маску, чтобы она соответствовала размерам маркера.
        contoured = np.zeros_like(cube_mask)
        cv2.drawContours(contoured, [best_contour], -1, 255, -1)
        contoured = contoured[y:y+h, x:x+w]

        # Поворачиваем маркер до тех пор,
        # пока в левом верхнем углу изображения груза не будет достигнута максимальная средняя яркость пикселей.
        # Такое положение груза достигается если маркировку можно читать слева на право.
        best_mean = -1
        best_part = 1
        for part in [2, 3, 4, 1]:
            mean = (contoured[:, :rotating_side].mean() + contoured[:rotating_side, :].mean()) / 2
            contoured = cv2.rotate(contoured, cv2.ROTATE_90_COUNTERCLOCKWISE)
            if mean > best_mean:
                best_mean = mean
                best_part = part
        part = best_part
        # Поворачиваем маркер на нужный угол.
        for _ in range(part):
            mark = cv2.rotate(mark, cv2.ROTATE_90_COUNTERCLOCKWISE)

        # Применяем алгоритм k-средних к пикселям изображения, ищем 5 "средних" цвета
        # Пиксель рассматривается как точка или вектор в трёхмерном пространстве
        Z = mark.reshape((-1,3))
        Z = np.float32(Z)
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
        K = 5
        ret, label, center=cv2.kmeans(Z, K, None, criteria, 10, cv2.KMEANS_RANDOM_CENTERS)
        center = np.uint8(center)
        res = center[label.flatten()]
        res2 = res.reshape((mark.shape))
        hsv = cv2.cvtColor(res2, cv2.COLOR_RGB2HSV)
        unique_colors = set([tuple(x) for y in range(len(hsv)) for x in hsv[y]])
        unique_colors = list(sorted(unique_colors, key=lambda x: sum(x) / len(x)))


        # Далее определена функция, которая
        # определяет код цвета на основе его значения H в цветовом пространстве HSV.
        # Если цвет ближе к черному, возвращаем код 1 (черный).
        # Если цвет ближе к белому, возвращаем код 0 (белый).
        # В противном случае, сопоставляем цвет с ближайшим цветом
        # из словаря h_colors и возвращаем его код.
        h_colors = {0: 0,    # 0 white
                    1: 0,    # 1 black
                    2: 22,   # 2 red
                    3: 78,   # 5 green
                    4: 144}  # 7 blue
        from_h_colors = {0: '0', 1: '1', 2: 'R', 3: 'G', 4: 'B'}
        def get_color_code(color: np.ndarray) -> int:
            color = np.int32(color)
            h, s, v = color
            if v < (255*0.3):  # most likely black
                return 1
            elif s < (255*0.2) and v > (255*0.8):  # most likely white
                return 0
            else:
                for key, hue in h_colors.items():
                    if h < hue:
                        return key
            return 2

        # Формируем строку colors, представляющую цвета на грузе.
        colors = [from_h_colors[get_color_code(np.array(color).astype(np.int32))] for color in unique_colors[1:]]
        #  и добавляем кортеж (координаты грузов, расшифровка маркировки) в список маркеров.
        marks.append((bbox, ''.join(colors)))
        cv2.imshow(str(j), cv2.cvtColor(res2, cv2.COLOR_BGR2RGB))
    return marks


inp = input()  # Ввод расшифровки маркировки груза, который необходимо обнаружить

# Инициализация работы с камерой
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    raise Exception('Camera is not opened')

# Считываем 10 кадров с камеры
# таким образом пропускаются кадры на которых ещё не настроен фокус и баланс белого
for _ in range(10):
    ret, frame = cap.read()
    if not ret:
        print('\nError: Cannot read frame')
        break

# конвертируем кадр в формат RGB
frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

# Вызываем функцию для детектирования и расшифровки маркировок
marks = get_marks(frame)
if len(marks) == 0:
    raise Exception('no marks')

# Инициализируем переменные для хранения координат и
# расшифровки наиболее близкой к введённой пользователем маркировки груза
best_bbox, best_mark_str = marks[0]
# переменная для лучшего расстояния Левенштейна
best_val = math.inf

# Перебираем все обнаруженные маркировки
# Для каждой маркировки вычисляем расстояние Левенштейна до пользовательской маркировки
for bbox, mark_str in marks:
    mark_str = mark_str#[:-1]
    lev = Levenshtein(inp, mark_str)
    lev.calculate()
    val = lev.minimum_cost
    #  Сохраняем параметры маркировки, наиболее близкой к пользовательской
    if val <= best_val:
        best_val = val
        best_bbox = bbox
        best_mark_str = mark_str

# отмечаем на изображении пользовательскую маркировку
cv2.drawContours(frame, [best_bbox], -1, (255, 0, 0), 1)

cv2.imshow('frame', cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
cv2.waitKey(0) # ожидаем нажатия любой клавиши и завершаем работу программы
