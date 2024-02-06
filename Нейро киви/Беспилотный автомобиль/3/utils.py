# coding: utf-8
import cv2
import numpy as np


def binarize(img, threshold, d=0):
    hls = cv2.cvtColor(img, cv2.COLOR_BGR2HLS)
    binary_h = cv2.inRange(hls, (0, 0, 30), (255, 255, 255))

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    binary_g = cv2.inRange(gray, threshold, 255)  # 130

    binary = cv2.bitwise_and(binary_g, binary_h)

    if d:
        cv2.imshow('hls', hls)
        cv2.imshow('hlsRange', binary_h)
        cv2.imshow('grayRange', binary_g)
        cv2.imshow('gray', gray)
        cv2.imshow('bin', binary)

    # return binary
    return binary


def binarize_exp(img, d=0):
    hls = cv2.cvtColor(img, cv2.COLOR_BGR2HLS)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    hls_s_channel = hls[:, :, 2]
    hls_l_channel = hls[:, :, 1]
    hls_h_channel = hls[:, :, 0]
    hsv_h_channel = hsv[:, :, 2]
    hsv_s_channel = hsv[:, :, 1]
    hsv_v_channel = hsv[:, :, 0]
    binary_h = cv2.inRange(hls, (0, 0, 30), (255, 255, 205))

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    binary_g = cv2.inRange(gray, 130, 255) #130
    binary = cv2.bitwise_and(binary_g, binary_h)

    if d:
        cv2.imshow('hls', hls)
        cv2.imshow('bgr_b', img[:, :, 0])
        cv2.imshow('bgr_g', img[:, :, 1])
        cv2.imshow('bgr_r', img[:, :, 2])
        cv2.imshow('hls_s', hls_s_channel)
        cv2.imshow('hls_l', hls_l_channel)
        cv2.imshow('hls_h', hls_h_channel)
        cv2.imshow('hsv_h', hsv_h_channel)
        cv2.imshow('hsv_s', hsv_s_channel)
        cv2.imshow('hsv_v', hsv_v_channel)
        cv2.imshow('hlsRange', binary_h)
        cv2.imshow('grayRange', binary_g)
        cv2.imshow('gray', gray)
        cv2.imshow('bin', binary)

    # return binary
    return binary

def trans_perspective(binary, trap, rect, size, d=0):
    matrix_trans = cv2.getPerspectiveTransform(trap, rect)
    perspective = cv2.warpPerspective(binary, matrix_trans, size, flags=cv2.INTER_LINEAR)
    if d:
        cv2.imshow('perspective', perspective)
    return perspective


def find_left_right(perspective, d=0):
    hist = np.sum(perspective[perspective.shape[0] // 3:, :], axis=0)
    mid = hist.shape[0] // 2
    left = np.argmax(hist[:mid])
    right = np.argmax(hist[mid:]) + mid
    if left <= 10 and right - mid <= 10:
        right = 399

    if d:
        cv2.line(perspective, (left, 0), (left, 300), 50, 2)
        cv2.line(perspective, (right, 0), (right, 300), 50, 2)
        cv2.line(perspective, ((left + right) // 2, 0), ((left + right) // 2, 300), 110, 3)
        cv2.imshow('lines', perspective)

    return left, right


def centre_mass(perspective, d=0):
    hist = np.sum(perspective, axis=0)
    if d:
        cv2.imshow("Perspektiv2in", perspective)

    mid = hist.shape[0] // 2
    i = 0
    centre = 0
    sum_mass = 0
    while (i <= mid):
        centre += hist[i] * (i + 1)
        sum_mass += hist[i]
        i += 1
    if sum_mass > 0:
        mid_mass_left = centre / sum_mass
    else:
        mid_mass_left = mid-1

    centre = 0
    sum_mass = 0
    i = mid
    while i < hist.shape[0]:
        centre += hist[i] * (i + 1)
        sum_mass += hist[i]
        i += 1
    if sum_mass > 0:
        mid_mass_right = centre / sum_mass
    else:
        mid_mass_right = mid+1

    # print(mid_mass_left)
    # print(mid_mass_right)
    mid_mass_left = int(mid_mass_left)
    mid_mass_right = int(mid_mass_right)
    if d:
        cv2.line(perspective, (mid_mass_left, 0), (mid_mass_left, perspective.shape[1]), 50, 2)
        cv2.line(perspective, (mid_mass_right, 0), (mid_mass_right, perspective.shape[1]), 50, 2)
        # cv2.line(perspective, ((mid_mass_right + mid_mass_left) // 2, 0), ((mid_mass_right + mid_mass_left) // 2, perspective.shape[1]), 110, 3)
        cv2.imshow('CentrMass', perspective)

    return mid_mass_left, mid_mass_right



def detect_stop(perspective):
    hist = np.sum(perspective, axis=1)
    maxStrInd = np.argmax(hist)
    # print("WhitePixCual" + str(hist[maxStrInd]//255))
    if hist[maxStrInd]//255 > 150:
        # print("SL detected. WhitePixselCual: "+str(int(hist[maxStrInd]/255)) + "Ind: " + str(maxStrInd))
        if maxStrInd > 120:  # 100
            # print("Time to stop")
            # cv2.line(perspective, (0, maxStrInd), (perspective.shape[1], maxStrInd), 60, 4)
            # cv2.imshow("STOP| ind:"+str(maxStrInd)+"IndCual"+str(hist[maxStrInd]//255), perspective)
            return True
    return False

def cross_center_path_v1(bin):
    # bin = bin[:bin.shape[0]-50, :]

    hist = np.zeros((2, bin.shape[1]), dtype=np.int32)
    begin_black_part = 0
    len_black_part = 0

    for i in range(bin.shape[1]):  # перебираем столбцы
        for j in range(bin.shape[0]):  # перебираем строки
            if bin[j, i] == 255:
                if len_black_part > hist[1, i]:  # max_len_black_part:
                    hist[1, i] = len_black_part
                    hist[0, i] = begin_black_part
                len_black_part = 0
                begin_black_part = j
                print(begin_black_part is j)
                # print("***")
                # print(begin_black_part, len_black_part)
                # print(begin_max_black_part, max_len_black_part)
                # print("***")
            else:
                len_black_part += 1
        if len_black_part > hist[1, i]:
            hist[1, i] = len_black_part
            hist[0, i] = begin_black_part

        begin_black_part = 0
        len_black_part = 0

    # рисую самые длинные чёрные отрезки
    bin_viz = bin.copy()
    for i in range(bin.shape[1]):
        bin_viz[hist[0, i]:(hist[0, i]+hist[1, i]), i] = 100
    cv2.imshow("cross_path", bin_viz)

    return False


def cross_center_path_v2(bin):
    hist = np.zeros(bin.shape[1], dtype=np.int32)
    bin_viz = bin.copy()
    for i in range(bin.shape[1]):  # columns
        for j in range(bin.shape[0] - 1, -1, -1):  # string
            if bin_viz[j, i] == 255:
                bin_viz[:j, i] = 255
                hist[i] = j
                break
            else:
                bin_viz[j, i] = 100
    cv2.imshow("cross_path", bin_viz)

    return False

def cross_center_path_v3(bin):
    """
    Для каждого столбца выбирает координату самого нижнего белого пикселя.
    Выбирает 30 столбцов, в которых белый пиксель максимально высоко.
    Рулит на среднее арифметическое от координат этих столбцов.
    """
    bin_viz = bin.copy()
    bin = np.flip(bin, axis=0)
    cv2.imshow("bin_invert", bin)
    hist = np.argmax(bin, axis=0)
    hist = bin.shape[0] - hist

    for i in range(hist.shape[0]):
        bin_viz[hist[i]:, i] = 100
    cv2.imshow("cross_path", bin_viz)

    hist = bin.shape[0] - hist
    ind_max_elem = np.argsort(hist)
    ind_max_elem = ind_max_elem[-30:]
    for i in ind_max_elem:
        bin_viz[:, i] = 50
    #cv2.imshow("cross_path_pulling", bin_viz)

    err = (bin.shape[1] // 2) - int(np.mean(ind_max_elem))

    bin_viz[:, bin.shape[1]//2] = 255
    cv2.line(bin_viz, (bin.shape[1]//2, bin.shape[0]), (bin.shape[1]//2-err, bin.shape[0]//2), 255, 4)
    cv2.imshow("cross_path_3", bin_viz)

    return err


def cross_center_path_v4(bin):  # таже третья, но без мишуры и визуализации
    """
        Для каждого столбца выбирает координату самого нижнего белого пикселя.
        Выбирает 30 столбцов, в которых белый пиксель максимально высоко.
        Рулит на среднее арифметическое от координат этих столбцов.
    """
    bin = np.flip(bin, axis=0)
    hist = np.argmax(bin, axis=0)
    ind_max_elem = np.argsort(hist)[-30:]
    err = (bin.shape[1] // 2) - int(np.mean(ind_max_elem))
    return err



def cross_center_path_v5(bin): # дорабатываем v3, чтобы рулило на всех наших перекрёстках.
    """
    Для каждого столбца выбирает координату самого нижнего белого пикселя.
    Выбирает Х (pull_size) столбцов, в которых белый пиксель максимально высоко.
    Х (pull_size) - зависит от отношения максимального к среднему.
    Рулит на среднее арифметическое от координат этих столбцов.

    """
    bin_viz = bin.copy()
    bin = np.flip(bin, axis=0)
    cv2.imshow("bin_invert", bin)
    hist = np.argmax(bin, axis=0)
    hist = bin.shape[0] - hist

    for i in range(hist.shape[0]):
        bin_viz[hist[i]:, i] = 100
    cv2.imshow("cross_path", bin_viz)



    hist = bin.shape[0] - hist

    max_columh = np.max(hist)
    mean_column = np.mean(hist)
    pull_size = max_columh / mean_column
    print(max_columh)
    print(mean_column)
    print(pull_size)
    print("*----")
    if pull_size >= 1.6:  # 1.6
        pull_size = int(50 * pull_size)
    else:
        pull_size = 30
    ind_max_elem = np.argsort(hist)
    ind_max_elem = ind_max_elem[-pull_size:]
    for i in ind_max_elem:
        bin_viz[:, i] = 50
    #cv2.imshow("cross_path_pulling", bin_viz)

    err = (bin.shape[1] // 2) - int(np.mean(ind_max_elem))

    bin_viz[:, bin.shape[1]//2] = 255
    cv2.line(bin_viz, (bin.shape[1]//2, bin.shape[0]), (bin.shape[1]//2-err, bin.shape[0]//2), 255, 4)
    cv2.imshow("cross_path_5", bin_viz)

    return err


def cross_center_path_v6(bin):  # таже пятая, но без мишуры и визуализации
    """
        Для каждого столбца выбирает координату самого нижнего белого пикселя.
        Выбирает 30 столбцов, в которых белый пиксель максимально высоко.
        Рулит на среднее арифметическое от координат этих столбцов.
    """
    bin = np.flip(bin, axis=0)
    hist = np.argmax(bin, axis=0)

    pull_size = np.max(hist) / np.mean(hist)
    if pull_size >= 1.6:  # 1.6
        pull_size = int(50 * pull_size)
    else:
        pull_size = 30

    ind_max_elem = np.argsort(hist)[-pull_size:]
    err = (bin.shape[1] // 2) - int(np.mean(ind_max_elem))
    return err




def detect_road_begin(perspective):  # для переключения с пересеченипея перекрёстка на следование по разметке
    left_corner = np.sum(perspective[-50:, :perspective.shape[1] // 3])
    right_corner = np.sum(perspective[-50:, perspective.shape[1] // 3 * 2:])
    print(left_corner)
    print(right_corner)
    print("**----**")
    # if left_corner >= 500000 and right_corner >= 170000:
    # if left_corner >= 170000 and right_corner >= 170000:
    if left_corner >= 170000 and right_corner >= 170000:
        return True
    else:
        return False