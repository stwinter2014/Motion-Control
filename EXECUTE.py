import InputPlanner  #классы данных
import PreAnalisys   #анализ геометрии
import AccDecControl #контроль разгона/торможения
import Interpolation #интерполяция
import Graphs        #вывод графиков

from math import floor, fabs
from geomdl import NURBS
"""Системные параметры"""
rapid_feed = 100                  #скорость при перемещении по G00, мм/с
v_max = 150                       #допустимая скорость, мм/с
acc_max = 2000                    #допустимое ускорение, мм/с2
acc_max_axis = [2000, 2000, 2000] #допустимое ускорение по осям, мм/с2
jerk_max = 20000                   #допустимый толчок, мм/с3
T = 0.001                         #период интерполяции, с
#Ошибки траектории
err_max = 0.1                     #допустимая линейная ошибка, мм
err_con = 0.1                     #допустимая контурная ошибка, мм
err_ch = 0.1                      #допустимая ошибка хорды, мм
N = 3                             #число участков предпросмотра

"""Входные данные"""
path = [[10, 0, 0],
        [10, 10, 0],
        [0, 10, 0],
        [0, 2, 0],
        [8, 2, 0],
        [8, 8, 0],
        [2, 8, 0]]                     #точки траектории
feedrate = [35, 35, 35, 35, 35, 25]    #заданная подача

epsilon = [0.1, 0.1, 0.1, 0.1, 0.1, 0] #допустимая угловая ошибка
n = [0, 0, 0, 0, 0, 0]                 #параметр скругления (если 0, то требуется расчет)
stopMode = [1, 1, 1, 1, 1, 1]          #тип траектории в угле
center = [[], [], [], [], [], []]      #координаты центра дуг
radius = [0, 0, 0, 0, 0, 0]            #радиусы дуг
NURBS_par = [[[], [], []], [[], [], []], [[], [], []], [[], [], []], [[], [], []],[[], [], []] ,[[], [], []]] #параметры NURBS кривой
path_type = [1, 1, 1, 1, 1, 1]         #тип траектории (0-1 - линейная, 2-3 - дуга по часовой и против, 4 - кривая)


"""Организация данных участков"""
segments = []
for i in range (len(path)-1):
    seg = InputPlanner.SegmentInterpreter(path_type[i], path[i], path[i+1], center[i], radius[i], NURBS_par[i], feedrate[i], 0, stopMode[i], [epsilon[i], n[i]], i)
    #print(seg.GCode, seg.Start, seg.End, seg.Center, seg.Radius, seg.Feed, seg.Status, seg.StopMode, seg.CornPar, seg.Num)
    segments.append(seg)
segments[0].Status = 1
segments[-1].Status = 2

#ТЕЛО АНАЛИЗА ГЕОМЕТРИИ
print("НАЧАЛО АНАЛИЗА ГЕОМЕТРИИ")
print("______________________________________________________")
segments_GeomAn = []
for i in range (len(segments)):
    print("______________________________________________________")
    if i < len(segments)-1:
        end_2 = segments[i+1].End
        gCode_2 = segments[i+1].GCode
    elif i == len(segments)-1:
        end_2 = []
        gCode_2 = 0
    #вызов модуля первичного анализа геометрии
    seg = PreAnalisys.GeomAnalisys(segments[i], end_2, gCode_2, rapid_feed, acc_max, acc_max_axis, jerk_max, err_max, T)
    if segments[i].GCode == 4:
        for i in range (len(seg[0])):
            segments_GeomAn.append(seg[0][i])
    else:
        if i < len(segments)-1:
            segments[i+1].Start = seg[-1]
        if len(seg) == 3:
            segments_GeomAn.append(seg[0])
            segments_GeomAn.append(seg[1])
        elif len(seg) == 2:
            segments_GeomAn.append(seg[0])

#Корректировка номеров сегментов
for i in range (len(segments_GeomAn)):
    if i == 0:
        segments_GeomAn[i].Status = 1
    elif i == len(segments_GeomAn)-1:
        segments_GeomAn[i].Status = 2
    else:
        segments_GeomAn[i].Status = 0
    segments_GeomAn[i].Num = i

"""
#Вывод графиков
#Исходная траектория
path_x = []
path_y = []
for i in range (len(path)):
    path_x.append(path[i][0])
    path_y.append(path[i][1])
Graphs.Graph_01 (path_x, path_y, "X, мм", "Y, мм", "Исходная траектория", "Траектория инструмента") #график
#скорректированная траектория
path_x_GeomAm = []
path_y_GeomAm = []
for i in range (len(segments_GeomAn)):
    path_x_GeomAm.append(segments_GeomAn[i].Start[0])
    path_y_GeomAm.append(segments_GeomAn[i].Start[1])
path_x_GeomAm.append(segments_GeomAn[len(segments_GeomAn)-1].End[0])
path_y_GeomAm.append(segments_GeomAn[len(segments_GeomAn)-1].End[1])
Graphs.Graph_02(path_x, path_y, path_x_GeomAm, path_y_GeomAm, "X, мм", "Y, мм", "Скорректированная траектория", "Исходная траектория", "Скорректированная траектория") #график
"""
"""
v_list = []
t_list = []
for i in range(1):
    Vel_p = AccDecControl.BlockDetermine (0, segments_GeomAn[i].Feed, 0, segments_GeomAn[i].Length, acc_max, jerk_max, T)
    v_list += Vel_p
for i in range(len(v_list)):
    t_list.append(T*i)
Graphs.Graph_01 (t_list, v_list, "X, мм", "Y, мм", "Исходная траектория", "Траектория инструмента") #график
"""


#ТЕЛО КОНТРОЛЯ СКОРОСТИ
print("______________________________________________________")
print("НАЧАЛО ПЛАНИРОВАНИЯ СКОРОСТИ")
print("______________________________________________________")
segments_FControl = [] #список сегментов после контроля скорости
velList = []           #для графиков
tList = []             #для графиков
vst_i = 0              #стартовая скорость
for i in range (len(segments_GeomAn)):
    print("______________________________________________________")
    print("Участок " + str(segments_GeomAn[i].Num+1) + ".")
    if len(segments_GeomAn) - i < N:
        N -= 1
    #функция построения профиля скорости и предпросмотра, формирует профиль скорости на участке
    #передаются: N участков, начальная скорость, системные параметры
    seg = AccDecControl.FeedControl (vst_i, segments_GeomAn[i:i+N], N, T, v_max, acc_max, jerk_max, err_max, err_con, err_ch)
    segments_FControl.append(seg)
    vst_i = seg.Vel[-1]
    velList += seg.Vel

#Вывод графиков
#профиль скорости
for i in range (len(velList )):
    tList.append(T*i)
Graphs.Graph_01 (tList, velList , "T, с", "V, мм/c", "Профиль скорости", "Скорость инструмента", "Профиль скорости") #график


#ТЕЛО ИНТЕРПОЛЯЦИИ
print("______________________________________________________")
print("НАЧАЛО ИНТЕРПОЛЯЦИИ")
print("______________________________________________________")
segments_Inter = [] #список сегментов после интерполяции
xList = []
yList = []
for i in range (len(segments_FControl)):
    print("______________________________________________________")
    print("Участок " + str(segments_FControl[i].Num+1))
    #вызов функции интерполяции
    seg = Interpolation.InterpolationStart (segments_FControl[i], T)
    segments_Inter.append(seg)
    xList += seg.X #для графика
    yList += seg.Y #для графика
if segments[0].GCode == 4:
    path_x = []
    path_y = []
    for i in range (len(NURBS_p)):
        path_x.append(NURBS_p[i][0])
        path_y.append(NURBS_p[i][1])
    Graphs.Graph_02 (path_x, path_y, xList, yList, "X, мм", "Y, мм", "Траектория после интерполяции", "Опорный многоугольник", "Интерполяция", "Интерполяция") #график
elif segments[0].GCode in [2,3]:
    path_x = []
    path_y = []
    path_x += segments_Inter[0].X
    path_y += segments_Inter[0].Y
    for i in range (1, len(path)):
        path_x.append(path[i][0])
        path_y.append(path[i][1])
    Graphs.Graph_02(path_x, path_y, xList, yList, "X, мм", "Y, мм", "Траектория после интерполяции", "Исходная траектория", "Интерполяция", "Интерполяция") #график
else:
    path_x = []
    path_y = []
    for i in range (len(path)):
        path_x.append(path[i][0])
        path_y.append(path[i][1])
    Graphs.Graph_02 (path_x, path_y, xList, yList, "X, мм", "Y, мм", "Траектория после интерполяции", "Исходная траектория", "Интерполяция", "Интерполяция") #график
    #Graphs.Graph_01 (xList, yList, "X, мм", "Y, мм", "Траектория", "Заданная траектория", "Интерполяция") #график

