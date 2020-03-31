import InputPlanner
from math import fabs, sqrt, pow, cos, sin
from geomdl import BSpline, NURBS

"""Интерполяция участков. Выбор интерполятора в зависимости от типа траектории.
Входные данные:
1. сегмент траектории (объект класса SegmentAccDecControl);
2. системный параметр (период интерполяции).
Выходные данные:
сегмент траектории после интерполяции (объект класса SegmentInterpolation).
"""
def InterpolationStart (segment, Tint):
    if segment.GCode in [0, 1]:
        interpolationRes = InterpolationLinear(segment.Start, segment.End, segment.Vel, segment.Length, Tint)
    elif segment.GCode in [2, 3]:
        #print("Длина пути по контролю скорости: " + str(segment.Iter[0] + segment.Iter[1] + segment.Iter[2]))
        error_com = ErrorCompensation (segment.Iter[0] + segment.Iter[1] + segment.Iter[2], segment.Length, len(segment.Vel))
        #print("Ошибка траектории: " + str(error_com))
        interpolationRes = InterpolationCircular(segment.Center, segment.Radius, segment.Vel, Tint, segment.GCode, segment.Curve, 0)
    elif segment.GCode == 4:
        #error_com = ErrorCompensation (segment.Iter[0] + segment.Iter[1] + segment.Iter[2], segment.Length, len(segment.Vel))
        #print("Ошибка траектории: " + str(error_com))
        interpolationRes = InterpolationNURBS(segment.Curve[0], segment.Curve[1], segment.Curve[2], segment.Curve[3], segment.Vel, Tint, 0, segment.Iter[0] + segment.Iter[1] + segment.Iter[2], segment.Length, len(segment.Vel))
    elif segment.GCode == 5:
        interpolationRes = InterpolationAngle(segment.Start, segment.End, segment.Vel, segment.Length, segment.Curve, Tint)
    print("Длина пути по интерполятору Lint = " + str(round(interpolationRes[2], 4)) + " мм.")
    #print("Длина пути исходная: " + str(segment.Length) + " мм.")
    print("Ошибка обработки: " + str(fabs(round(segment.Length - interpolationRes[2],4))) + " мм.")
    #формирование данных об участке после контроля скорости
    seg = InputPlanner.SegmentInterpolation(#segmentList[0].GCode,    #Тип G-кода как в исходном участке
                                            #segmentList[0].Start,    #Начальная точка сегмента
                                            #segmentList[0].End,      #Конечная точка сегмента, корректируется скруглением
                                            #segmentList[0].Center,   #Координаты центра окружности (при движении по дуге)
                                            #segmentList[0].Radius,   #Радиус окружности (при движении по дуге)
                                            segment.Vel,              #Список скоростей на участке
                                            interpolationRes[0],      #Координаты X
                                            interpolationRes[1],      #Координаты Y
                                            segment.Iter,             #Список периодов интерполяции
                                            segment.Status,           #Статус участка (будет корректироваться)
                                            #segmentList[0].StopMode, #Наличие скругления в угле после данного участка
                                            #segmentList[0].CornPar,  #Параметры скругления (e и n)
                                            segment.Num,              #Номер участка (будет корректироваться)
                                            segment.Length            #Длина участка
                                            #segmentList[0].Curve     #Контрольные точки (только для скруглений)
                                            )
    return seg

""""""
def ErrorCompensation (L, length, len_vellist):
    e = length - (L)
    delta_e = e/len_vellist
    return delta_e

"""Линейная интерполяция.
Входные данные:
1. координаты начальной точки [x, y, z];
2. координаты конечной точки [x, y, z];
3. список скоростей, мм/с;
4. длина блока, мм;
5. время интерполяции, с.
Выходные данные:
1. список координат оси X, мм;
2. список координат оси Y, мм."""
def InterpolationLinear(p_start, p_finish, vellist, length, Tint):
    S = 0
    x_list = []
    y_list = []
    #print(p_start)
    #print(p_finish)
    x_list.append(p_start[0])
    y_list.append(p_start[1])
    x_tmp = p_start[0]
    y_tmp = p_start[1]
    for i in range (len(vellist)-1):
        #расчет единичного перемещения
        Si = Tint*(vellist[i]+vellist[i+1])/2
        #расчет приращений для каждой координаты
        delta_x = Si*((p_finish[0]-p_start[0])/length)
        delta_y = Si*((p_finish[1]-p_start[1])/length)
        x_tmp += delta_x
        y_tmp += delta_y
        x_list.append(x_tmp)
        y_list.append(y_tmp)
        S += Si
    return x_list, y_list, S

"""Круговая интерполяция.
Входные данные:
1. координаты начальной точки [x, y, z];
2. координаты конечной точки [x, y, z];
3. список скоростей, мм/с;
4. длина блока, мм;
5. время интерполяции, с.
Выходные данные:
1. список координат оси X, мм;
2. список координат оси Y, мм."""
def InterpolationCircular(center, radius, vellist, Tint, GCode, par, e):
    i = 0
    x_list = []
    y_list = []
    si = 0
    s = 0
    d = 1
    if par[0] == int(0) and GCode == 2:
        par[0] = 6.28
        d = -1
    u = par[0]
    if par[0] > par[1]:
        while u >= par[1]:
            x = center[0] + radius*cos(u)
            y = center[1] + radius*sin(u)
            x_list.append(x)
            y_list.append(y)
            if i > 0:
                si = sqrt((pow((x_list[-1] - x_list[-2]), 2))+(pow((y_list[-1] - y_list[-2]), 2)))
                s += si
            u += d*vellist[i]*Tint/radius
            i += 1
    elif par[0] < par[1]:
        while u <= par[1]:
            x = center[0] + radius*cos(u)
            y = center[1] + radius*sin(u)
            x_list.append(x)
            y_list.append(y)
            if i > 0:
                si = sqrt((pow((x_list[-1] - x_list[-2]), 2))+(pow((y_list[-1] - y_list[-2]), 2)))
                s += si
            u += d*(vellist[i]*Tint+fabs(e))/radius
            i += 1
    return x_list, y_list, s

"""Параметрическая интерполяция (NURBS).
Входные данные:
1. координаты начальной точки [x, y, z];
2. координаты конечной точки [x, y, z];
3. список скоростей, мм/с;
4. длина блока, мм;
5. время интерполяции, с.
Выходные данные:
1. список координат оси X, мм;
2. список координат оси Y, мм."""
def InterpolationNURBS(points, weights, knots, par, vellist, Tint, err, L, length, it_num):
    crv = NURBS.Curve()    #задание кривой
    crv.degree = 3         #степень
    crv.ctrlpts = points   #контрольные точки
    crv.weights = weights  #весы опорных точек
    crv.knotvector = knots #узловой вектор
    x_list = []
    y_list = []
    t = par[0]
    s = 0
    i = 0
    print("Параметр u начальный и конечный: " + str(par))
    while t <= par[1] and i < len(vellist) and s < length:
        result = crv.evaluate_single(t)
        der = crv.derivatives(t, order=2)
        x_list.append(result[0])
        y_list.append(result[1])
        
        duds1 = 1/(sqrt(der[1][0]*der[1][0]+der[1][1]*der[1][1]))
        duds2 = (der[1][0]*der[2][0] + der[1][1]*der[2][1])/(pow(der[1][0]*der[1][0]+der[1][1]*der[1][1], 2))
        
        if i > 0:
            si = sqrt((pow((x_list[-1] - x_list[-2]), 2))+(pow((y_list[-1] - y_list[-2]), 2)))
            s += si
        t = t + (vellist[i]*Tint+err)*duds1 + pow(vellist[i]*Tint+err, 2)*duds2/2
        t = t
        i += 1
    return x_list, y_list, s

"""Параметрическая интерполяция (углы).
Входные данные:
1. координаты начальной точки [x, y, z];
2. координаты конечной точки [x, y, z];
3. список скоростей, мм/с;
4. длина блока, мм;
5. контрольные точки [x, y, z];
5. время интерполяции, с.
Выходные данные:
1. список координат оси X, мм;
2. список координат оси Y, мм."""
def InterpolationAngle(p_start, p_finish, vellist, length, curve, Tint):
    p0 = curve[0]
    p1 = curve[1]
    p2 = curve[2]
    p3 = curve[3]
    p4 = curve[4]
    p5 = curve[5]
    L = 0
    xCoord = []
    yCoord = []
    u05 = []
    u = 0
    while round(u, 3) <= 1: #формула № из отчета
        x_d = 5*pow((1-u), 4)*(p1[0] - p0[0]) + 20*u*pow((1-u), 3)*(p2[0] - p1[0]) + 30*u*u*pow((1-u), 2)*(p3[0] - p2[0]) + 20*pow(u, 3)*(1-u)*(p4[0] - p3[0]) + 5*pow(u, 4)*(p5[0] - p4[0])
        y_d = 5*pow((1-u), 4)*(p1[1] - p0[1]) + 20*u*pow((1-u), 3)*(p2[1] - p1[1]) + 30*u*u*pow((1-u), 2)*(p3[1] - p2[1]) + 20*pow(u, 3)*(1-u)*(p4[1] - p3[1]) + 5*pow(u, 4)*(p5[1] - p4[1])
        x = round(pow((1-u), 5)*p0[0] + 5*u*pow((1-u), 4)*p1[0] + 10*u*u*pow((1-u), 3)*p2[0] + 10*pow(u, 3)*pow((1-u), 2)*p3[0] + 5*pow(u, 4)*(1-u)*p4[0] + pow(u, 5)*p5[0], 4)
        y = round(pow((1-u), 5)*p0[1] + 5*u*pow((1-u), 4)*p1[1] + 10*u*u*pow((1-u), 3)*p2[1] + 10*pow(u, 3)*pow((1-u), 2)*p3[1] + 5*pow(u, 4)*(1-u)*p4[1] + pow(u, 5)*p5[1], 4)
        if u > 0:
            L += sqrt((pow((x-x_prev), 2))+(pow((y-y_prev), 2)))
        #дополнительная проверка угловой ошибки
        #if round(u, 3) == 0.5:
            #u05 = [x, y]
        xCoord.append(x)
        yCoord.append(y)
        u += vellist[0]/sqrt(x_d*x_d + y_d*y_d)*Tint
        x_prev = x
        y_prev = y
    return xCoord, yCoord, L
