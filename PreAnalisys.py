#библиотеки python
from decimal import Decimal
from geomdl import BSpline, NURBS
from math import asin, acos, sqrt, pow, degrees, fabs, sin, cos, pi
#разработанные модули
import InputPlanner #модуль с классами участков
import Graphs #модуль с функциями показа графиков

"""Подсчет длины сегмента траектории.
Входные данные
1. Тип траектории (0: линейная, ускоренное, 1: линейная, 2: круговая по часовой, 3: круговая против часовой)
1. Координаты начальной точки [x, y, z], мм;
2. Координаты конечной точки [x, y, z], мм;
3. Координаты центра окружности (при круговом движении), мм;
4. Радиус окружности (при круговом движении), мм.
Выходные данные
1. Длина траектории, мм."""
def PathLength(typet, start, end, center, radius):
    #при линейном движении
    if typet == 0 or 1:
        length = sqrt((pow((end[0]-start[0]), 2))+(pow((end[1]-start[1]), 2)))
    #при круговом движении по часовой стрелке
    if typet == 2:
        circle = pi*2*radius
        length_chord = sqrt((pow((end[0]-start[0]), 2))+(pow((end[1]-start[1]), 2)))
        alpha = 2*asin(length_chord/(2*radius))
        if center[3] == -1:
            length = circle - alpha*radius
        elif center[3] in [0,1]:
            length = alpha*radius
    #при круговом движении против часовой стрелки
    if typet == 3:
        circle = pi*2*radius
        length_chord = sqrt((pow((end[0]-start[0]), 2))+(pow((end[1]-start[1]), 2)))
        alpha = 2*asin(length_chord/(2*radius))
        if center[3] == -1:
            length = circle - alpha*radius
        elif center[3] in [0,1]:
            length = alpha*radius
    #length_chord = sqrt((pow((end[0]-start[0]), 2))+(pow((end[1]-start[1]), 2)))
    #print(length_chord)
    #alpha = 2*asin(length_chord/(2*radius))
    #print(alpha)
    #length = alpha*radius
    return round(length, 4)

"""Подсчет угла между участками траектории A и B.
Входные данные
1. Тип траектории участка 1 (0: линейная, ускоренное, 1: линейная, 2: круговая с точкой центра, 3: круговая с радиусом);
2. Стартовая точка траектории A в формате [x, y, z];
3. Конечная точка траектории A в формате [x, y, z];
4. Тип траектории участка 2 (0: линейная, ускоренное, 1: линейная, 2: круговая с точкой центра, 3: круговая с радиусом);
5. Стартовая точка траектории B в формате [x, y, z];
6. Конечная точка траектории B в формате [x, y, z];
7. Длина участка 1;
8. Длина участка 2.
Выходные данные
1. Угол, рад."""
def Angle(Gcode1, st_point1, fin_point1, Gcode2, st_point2, fin_point2, length1, length2):
    if Gcode1 in [0, 1] and Gcode2 in [0, 1]:
        v1 = [st_point1[0] - fin_point1[0], st_point1[1] - fin_point1[1], st_point1[2] - fin_point1[2]]
        v2 = [fin_point2[0] - st_point2[0], fin_point2[1] - st_point2[1], fin_point2[2] - st_point2[2]]
        vScalarM = v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2]
        cosa = (vScalarM)/(length1*length2)
        angle = acos(cosa)
    return angle

"""Подсчет оптимального параметра n = c/d.
Входные данные: угол между участками траектории.
Выходные данные: значение параметра n."""
def nOptimal (angle):
    #print(degrees(angle))
    nOpt = round(1/(2.0769)*pow(angle, 0.9927), 4) #формула № из отчета
    return nOpt

"""Расчет параметров скругления (c, d и контрольные точки).
Входные данные:
1. Начальная точка траектории блока 1, [x, y, z] мм;
2. Конечная точка траектории блока 1, [x, y, z] мм;
3. Начальная точка траектории блока 2, [x, y, z] мм;
4. Конечная точка траектории блока 2, [x, y, z] мм;
5. Допустимая угловая ошибка e, мм;
6. Параметр скругления n.
Выходные данные:
1. Параметр c, мм;
2. Параметр d, мм;
3. Контрольные точки P0, P1, ..., P5 [x, y, z] мм."""
def CornerParam (stPoint1, finPoint1, stPoint2, finPoint2, epsilon, n):
    #Расчет единичных векторов касательной
    ts1 = [finPoint1[0] - stPoint1[0], finPoint1[1] - stPoint1[1], finPoint1[2] - stPoint1[2]]
    te1 = [finPoint2[0] - stPoint2[0], finPoint2[1] - stPoint2[1], finPoint2[2] - stPoint2[2]]
    ts = [ts1[0]/(sqrt(ts1[0]*ts1[0]+ts1[1]*ts1[1])), ts1[1]/(sqrt(ts1[0]*ts1[0]+ts1[1]*ts1[1]))]
    te = [te1[0]/(sqrt(te1[0]*te1[0]+te1[1]*te1[1])), te1[1]/(sqrt(te1[0]*te1[0]+te1[1]*te1[1]))]
    tse = [ts[0] + te[0], ts[1] + te[1]]
    t = sqrt(tse[0]*tse[0] + tse[1]*tse[1])
    #расчет параметров c и d
    d = round((32*epsilon)/(t*(7*n+16)), 4)
    c = round(n*d, 4)
    #расчет контрольных точек
    point_0 = [finPoint1[0] - (2*c+d)*ts[0], finPoint1[1] - (2*c+d)*ts[1], 0]
    point_1 = [point_0[0] + c*ts[0], point_0[1] + c*ts[1], 0]
    point_2 = [point_0[0] + 2*c*ts[0], point_0[1] + 2*c*ts[1], 0]
    point_5 = [finPoint1[0] + (2*c+d)*te[0], finPoint1[1] + (2*c+d)*te[1], 0]
    point_3 = [point_5[0] - 2*c*te[0], point_5[1] - 2*c*te[1], 0]
    point_4 = [point_5[0] - c*te[0], point_5[1] - c*te[1], 0]
    #print("point_0 " + str(point_0))
    #print("point_1 " + str(point_1))
    #print("point_2 " + str(point_2))
    #print("point_3 " + str(point_3))
    #print("point_4 " + str(point_4))
    #print("point_5 " + str(point_5))
    return c, d, point_0, point_1, point_2, point_3, point_4, point_5

"""Расчет допустимой скорости в углу.
Входные данные:
1. Параметр d, мм
2. Параметр n;
3. Угол между сегментами, рад;
4. Допустимое осевое ускорение, [Amax_x, Amax_y, Amax_z] мм/с2.
Выходные данные:
1. Список значений кривизны для каждого u;
2. Список значений u;
3. Максимальная кривизна и соответствующий u;
4. Угловая скорость, мм/с."""
def CornerFeed (d, n, angleA, Amax):
    curve_list = []
    u_list = []
    u = 0
    M = 100
    p0 = [d*(2*n+1), 0]
    p1 = [d*(n+1), 0]
    p2 = [d, 0]
    p3 = [d*cos(angleA), d*sin(angleA)]
    p4 = [d*(n+1)*cos(angleA), d*(n+1)*sin(angleA)]
    p5 = [d*(2*n+1)*cos(angleA), d*(2*n+1)*sin(angleA)]
    while round(u, 3) <= 1: #формулы № и № из отчета
        x_d = 5*pow((1-u), 4)*(p1[0] - p0[0]) + 20*u*pow((1-u), 3)*(p2[0] - p1[0]) + 30*u*u*pow((1-u), 2)*(p3[0] - p2[0]) + 20*pow(u, 3)*(1-u)*(p4[0] - p3[0]) + 5*pow(u, 4)*(p5[0] - p4[0])
        y_d = 5*pow((1-u), 4)*(p1[1] - p0[1]) + 20*u*pow((1-u), 3)*(p2[1] - p1[1]) + 30*u*u*pow((1-u), 2)*(p3[1] - p2[1]) + 20*pow(u, 3)*(1-u)*(p4[1] - p3[1]) + 5*pow(u, 4)*(p5[1] - p4[1])
        x_dd = 20*pow((1-u), 3)*(p2[0] - 2*p1[0] + p0[0]) + 60*u*pow((1-u), 2)*(p3[0] - 2*p2[0] + p1[0]) + 60*pow(u, 2)*(1-u)*(p4[0] - 2*p3[0] + p2[0]) + 20*pow(u, 3)*(p5[0] - 2*p4[0] + p3[0])
        y_dd = 20*pow((1-u), 3)*(p2[1] - 2*p1[1] + p0[1]) + 60*u*pow((1-u), 2)*(p3[1] - 2*p2[1] + p1[1]) + 60*pow(u, 2)*(1-u)*(p4[1] - 2*p3[1] + p2[1]) + 20*pow(u, 3)*(p5[1] - 2*p4[1] + p3[1])
        curvature = fabs((x_d*y_dd - y_d*x_dd)/pow((x_d*x_d + y_d*y_d), 1.5)) #формула № из отчета
        curve_list.append(round(curvature, 4))
        u_list.append(u)
        u += 1/M
    maxk = max(curve_list)
    maxu = u_list[curve_list.index(maxk)]
    feed = round(min(sqrt(Amax[0]/maxk), sqrt(Amax[1]/maxk), sqrt(Amax[2]/maxk)), 4)
    return curve_list, u_list, [maxk, maxu], feed
    
"""Построение скругления кривой Безье
Входные данные:
1. Контрольные точки список из 6 шт., [x, y, z] мм.
Выходные данные:
1. Координаты оси x, мм;
2. Координаты оси y, мм;
3. Координаты оси z, мм."""
def SplineLength (controlP):
    L = 0
    xCoord = []
    yCoord = []
    zCoord = []
    u05 = []
    M = 100
    u = 0 #параметр кривой, изменяется от 0 до 1.
    while round(u, 3) <= 1: #формула № из отчета
        x = round(pow((1-u), 5)*controlP[0][0] + 5*u*pow((1-u), 4)*controlP[1][0] + 10*u*u*pow((1-u), 3)*controlP[2][0] + 10*pow(u, 3)*pow((1-u), 2)*controlP[3][0] + 5*pow(u, 4)*(1-u)*controlP[4][0] + pow(u, 5)*controlP[5][0], 4)
        y = round(pow((1-u), 5)*controlP[0][1] + 5*u*pow((1-u), 4)*controlP[1][1] + 10*u*u*pow((1-u), 3)*controlP[2][1] + 10*pow(u, 3)*pow((1-u), 2)*controlP[3][1] + 5*pow(u, 4)*(1-u)*controlP[4][1] + pow(u, 5)*controlP[5][1], 4)
        #z = round(pow((1-u), 5)*controlP[0][2] + 5*u*pow((1-u), 4)*controlP[1][2] + 10*u*u*pow((1-u), 3)*controlP[2][2] + 10*pow(u, 3)*pow((1-u), 2)*controlP[3][2] + 5*pow(u, 4)*(1-u)*controlP[4][2] + pow(u, 5)*controlP[5][2], 4)
        if u > 0:
            L += sqrt((pow((x-x_prev), 2))+(pow((y-y_prev), 2)))
        #дополнительная проверка угловой ошибки
        if round(u, 3) == 0.5:
            u05 = [x, y]
        xCoord.append(x)
        yCoord.append(y)
        #zCoord.append(z)
        u += 1/M
        x_prev = x
        y_prev = y
    return round(L, 4), xCoord, yCoord, zCoord, u05

def ArcParam (start, end, center, radius):
    circle_len = 2*radius*pi
    #точка начала окружности
    startArc_p = [center[0]+radius, center[1], center[2]] 
    #дуга от начала окружности до стартовой точки
    length_chord = sqrt((pow((start[0]-startArc_p[0]), 2))+(pow((start[1]-startArc_p[1]), 2)))
    startArc_len = 2*asin(length_chord/(2*radius))*radius
    if start[1] < startArc_p[1]:
        startArc_len = circle_len - 2*asin(length_chord/(2*radius))*radius
    #угол до стартовой точки
    startArc_a = (startArc_len*2)/(2*radius)
    #дуга от стартовой до конечной точки
    length_chord = sqrt((pow((end[0]-startArc_p[0]), 2))+(pow((end[1]-startArc_p[1]), 2)))
    finArc_len = 2*asin(length_chord/(2*radius))*radius
    if end[1] < startArc_p[1]:
        finArc_len = circle_len - 2*asin(length_chord/(2*radius))*radius
    #угол до конечной точки
    finArc_a = (finArc_len*2)/(2*radius)
    length_chord = sqrt((pow((end[0]-start[0]), 2))+(pow((end[1]-start[1]), 2)))
    len_arc = 2*asin(length_chord/(2*radius))*radius    
    return [round(startArc_a, 2), round(finArc_a, 2)]

"""Расчет длины пути на NURBS кривой.
Входные данные: точки кривой (список), [x, y, z] мм.
Выходные данные: аппроксимированная длина кривой, мм."""
def NURBSLength (points):
    si = 0
    s = 0
    for i in range (len(points)-1):
        si = sqrt((pow((points[i+1][0]-points[i][0]), 2))+(pow((points[i+1][1]-points[i][1]), 2)))
        s += si
    return s

"""Предварительный анализ NURBS кривой.
Входные данные:
1. Контрольные точки кривой (список), [x, y, z]
2. Веса контрольных точек (список)
3. Узловой вектор (список)
4. Системные параметры (макс. ускорение, макс. рывок, макс. контурная ошибка, подача, период интерполяции)
Выходные данные:
Длины подкривых, мм; начальные и конечные значения параметра для подкривых (список).
"""
def NURBS_PrePros (points, weights, knots, A, J, E, F, T):
    crv = NURBS.Curve()    #задание кривой
    crv.degree = 3         #степень
    crv.ctrlpts = points   #контрольные точки
    crv.weights = weights  #весы опорных точек
    crv.knotvector = knots #узловой вектор
    MAXcurve = []
    curve = []             #список точек кривой
    curvature_list = []    #список значений кривизны
    kThres_list = []       #список значений порога кривизны
    t_list = []            #список значений параметра t
    candPC_sublist = []    #список временный
    candP_sublist = []     #список временный
    candT_sublist = []     #список временный
    candPC_list = []       #список значений кривизны кандидатных точек
    candP_list = []        #список кандидатных точек
    candT_list = []        #список параметров t кандидатных точек
    subCurve_list = []     #Список точек подкривой
    subCurve = []          #Список точек всех подкривых
    t_out = []             #Список начальных и конечных параметров t для подкривых
    #начало расчета
    t = knots[0]           #начальное значение параметра
    M = 100                #число итераций
    curve = crv.evalpts
    while round(t, 3) <= knots[-1]:
        ders = crv.derivatives(t, order=2) #расчет производных кривой (исходной точки кривой, 1 и 2 производной)
        der_1 = ders[1]
        der_2 = ders[2]
        #кривизна
        curvature = fabs((der_1[0]*der_2[1]-der_1[1]*der_2[0]))/pow((sqrt(der_1[0]*der_1[0]+ der_1[1]*der_1[1])), 3)
        curvature_list.append(curvature)
        #порог кривизны
        kThres = min(8*E/(pow(F*T, 2)+4*E*E), A/(F*F), sqrt(J/(pow(F, 3))))
        kThres_list.append(kThres)
        #параметр t
        t_list.append(t)
        #проверка на кандидатную точку
        if curvature > kThres:
            candPC_sublist.append(curvature) #сохранение кандидатной точки
            candP_sublist.append(ders[0])    #сохранение кандидатной точки
            candT_sublist.append(t)          #сохранение кандидатной точки
        elif curvature <= kThres and candPC_sublist != []:
            candPC_list.append(candPC_sublist)
            candP_list.append(candP_sublist)
            candT_list.append(candT_sublist)
            candPC_sublist = []
            candP_sublist = []
            candT_sublist = []
        #инкремент параметра t
        t += 1/M
        t = round(t, 3)
    if candPC_sublist != []:
        candPC_list.append(candPC_sublist)
        candP_list.append(candP_sublist)
        candT_list.append(candT_sublist)
        candPC_sublist = []
        candP_sublist = []
        candT_sublist = []
    #график кривизны и порога кривизны
    Graphs.Graph_02 (t_list, curvature_list, t_list, kThres_list, "Параметр u", "Кривизна к", "График кривизны NURBS", "Кривизна", "Порог кривизны", "Кривизна") #график
    #нахождение критических точек и разбиение кривой на подкривые
    tl = 0
    t_out.append(t_list[tl])
    for i in range (len(candPC_list)):
        critP = max(candPC_list[i]) #локальный максимум кривизны
        MAXcurve.append(critP)
        tc = curvature_list.index(critP)
        t_out.append(t_list[tc])
        print("Локальный максимум кривизны " + str(i+1) + " = " + str(critP))
        print("Точка кривой = " + str(candP_list[i][candPC_list[i].index(critP)]))
        print("Параметр t = " + str(candT_list[i][candPC_list[i].index(critP)]))
        subCurve += (curve[tl:tc+1])
        tl = tc
        subCurve_list.append(subCurve)
        #график подкривой
        x = []
        y = []
        for j in range (len(subCurve)):
            x.append(subCurve[j][0])
            y.append(subCurve[j][1])
        subCurve = []
        Graphs.Graph_01 (x, y, "X, мм", "Y, мм", "Подкривая "  + str(len(subCurve_list)), "Траектория", "Подкривая " + str(len(subCurve_list))) #график
    MAXcurve.append(curvature_list[-1])
    tc = t_list.index(t_list[-1])
    t_out.append(t_list[tc])
    subCurve += (curve[tl:tc+1])
    subCurve_list.append(subCurve)
    #график последней подкривой
    x = []
    y = []
    for i in range (len(subCurve)):
        x.append(subCurve[i][0])
        y.append(subCurve[i][1])
    subCurve = []
    Graphs.Graph_01 (x, y, "X, мм", "Y, мм", "Подкривая " + str(len(subCurve_list)), "Траектория", "Подкривая " + str(len(subCurve_list))) #график
    #график кривой (всех подкривых)
    x = []
    y = []
    for i in range (len(subCurve_list)):
        for j in range (len(subCurve_list[i])):
            x.append(subCurve_list[i][j][0])
            y.append(subCurve_list[i][j][1])
    Graphs.Graph_01 (x, y, "X, мм", "Y, мм", "Кривая", "Траектория", "Кривая") #график
    #Цветной график подкривых
    """
    x1 = []
    y1 = []
    x2 = []
    y2 = []
    x3 = []
    y3 = []
    x4 = []
    y4 = []
    for i in range(len(subCurve_list[0])):
        x1.append(subCurve_list[0][i][0])
        y1.append(subCurve_list[0][i][1])
    for i in range(len(subCurve_list[1])):
        x2.append(subCurve_list[1][i][0])
        y2.append(subCurve_list[1][i][1])
    for i in range(len(subCurve_list[2])):
        x3.append(subCurve_list[2][i][0])
        y3.append(subCurve_list[2][i][1])
    for i in range(len(subCurve_list[3])):
        x4.append(subCurve_list[3][i][0])
        y4.append(subCurve_list[3][i][1])
    Graphs.Graph_04(x1, y1, x2, y2, x3, y3, x4, y4,
             "X, мм", "Y, мм", "Кривая после анализа геометрии", "Подкривая 1", "Подкривая 2", "Подкривая 3", "Подкривая 4", "Кривая цветная")
    """
    #график кривой и исходного полигона
    x = []
    y = []
    for i in range(len(points)):
        x.append(points[i][0])
        y.append(points[i][1])
    x_1 = []
    y_1 = []
    for i in range (len(curve)):
        x_1.append(curve[i][0])
        y_1.append(curve[i][1])
    Graphs.Graph_02 (x, y, x_1, y_1, "X, мм", "Y, мм", "Кривая и опорный многоугольник", "Опорный многоугольник", "Кривая", "Кривая и полигон") #график
    #подсчет длины подкривой и формирования блоков подкривых
    out_list = [] #выходной список
    for i in range (len(subCurve_list)):
        subC_len = NURBSLength(subCurve_list[i]) #подсчет длины пути на подкривой
        out_list.append([subC_len, t_out[i], t_out[i+1], MAXcurve[i]]) #формирование выходного списка длин и парамтеров подкривых
    return out_list


#GEOMETRY ANALISYS MODULE
"""Модуль предварительного анализа геометрии.
Осуществляет расчет длины пути участков. Если требуется скругление, то определяются параметры скругления,
скорость на углу, длина скргуления, корректируется длина соседних участков.
Входные данные: 2 экземпляра класса SegmentInterpreter - текущий и следующий участок траектории, системные параметры.
Выходные данные: список экземпляров класса SegmentPreAnalysis - текущий участок траектории и участок скругления, если имеется."""
def GeomAnalisys(segment1, segment2_endP, segment2_GCode, rapid_feed, acc_max, acc_max_axis, j_max, e_con, Tint):
    #задание скорости для ускоренного движения
    if segment1.GCode == 0:
        segment1.Feed = rapid_feed
        #print("Скорость при ускоренном движении Fr = " + str(segment1.Feed) + " мм/с.")
    #если текущий участок - последний.
    if segment1.GCode in [0, 1] and segment2_endP == []:
        print("Участок " + str(segment1.Num+1) + ".")
        length1 = PathLength(segment1.GCode, segment1.Start, segment1.End, segment1.Center, segment1.Radius)
        print("Длина участка L = " + str(length1) + " мм.")
        if segment1.GCode == 0:
            print("Скорость при ускоренном движении Fr = " + str(segment1.Feed) + " мм/с.")
        #линейный участок
        seg1 = InputPlanner.SegmentPreAnalysis(segment1.GCode,    #Тип G-кода как в исходном участке
                                               segment1.Start,    #Начальная точка сегмента
                                               segment1.End,      #Конечная точка сегмента, корректируется скруглением
                                               segment1.Center,   #Координаты центра окружности (при движении по дуге)
                                               segment1.Radius,   #Радиус окружности (при движении по дуге)
                                               segment1.Feed,     #Подача
                                               0,                 #Статус участка (будет корректироваться)
                                               segment1.StopMode, #Наличие скругления в угле после данного участка
                                               segment1.CornPar,  #Параметры скругления (e и n)
                                               0,                 #Номер участка (будет корректироваться)
                                               length1,           #Длина участка
                                               []                 #Параметры кривой(для скруглений или дуг)
                                               )
        #На выход передается обновленный текущий линейный участок (скорректирована длина участка).
        output = [seg1, []]
    #Если участок линейный
    if segment1.GCode in [0, 1] and segment2_GCode in [0, 1] and segment2_endP != []:
        
        print("Участок " + str(segment1.Num+1) + ".")
        
        """Подсчет длины пути на участке"""
        length1 = PathLength(segment1.GCode, segment1.Start, segment1.End, segment1.Center, segment1.Radius)
        length2 = PathLength(segment2_GCode, segment1.End, segment2_endP, [], 0)
        print("Длина участка L = " + str(length1) + " мм.")
        
        """Подсчет угла между участками"""
        angle = Angle(segment1.GCode, segment1.Start, segment1.End, segment2_GCode, segment1.End, segment2_endP, length1, length2)
        print("Угол между текущим и сл. участком Q = " + str(degrees(angle)) + " град.")
    
        """Определение параметров траектории на углу, контрольных точек"""
        if segment1.StopMode == 1:
            if segment1.CornPar[1] == 0:
                segment1.CornPar[1] = nOptimal(angle) #оптимальное n
                print("Оптимальный параметр n = " + str(segment1.CornPar[1]) + ".")
            param = CornerParam (segment1.Start, segment1.End, segment1.End, segment2_endP, segment1.CornPar[0], segment1.CornPar[1])
            corner_cd = param[0:2] #параметры c и d
            points = param[2:8]    #контрольные точки
        
            """Определение скорости на углу"""
            """Определение кривизны"""
            k = CornerFeed (corner_cd[1], segment1.CornPar[1], angle, acc_max_axis)
            #Graphs.Graph_01 (k[1], k[0], "Параметр u", "Кривизна к", "Кривизна кривой Безье", "Кривизна скругления", "Кривизна скругления") #график кривизны
            print("Максимальная кривизна к = " + str(k[2][0]) + " мм.")
            print("Максимальная угловая скорость Fc = " + str(k[3]) + " мм/с.")
            
            """Расчет длины скругления"""
            spline = SplineLength(points)
            lengthSpline = spline[0]
            print("Длина скругления Ls = " + str(lengthSpline) + " мм.")
        
            """Поправка длины пути участка с учетом скруглений"""
            length1 = PathLength(segment1.GCode, segment1.Start, points[0], segment1.Center, segment1.Radius)
            length2 = PathLength(segment2_GCode, points[5], segment2_endP, [], 0)
            print("Уточненная длина текущего участка L = " + str(length1) + " мм.")
            print("Уточненная длина след. участка Lnext = " + str(length2) + " мм.")
            
            """Построение траектории на углу"""
            """
            #график траектории в угле
            x = [points[0][0], segment1.End[0], points[-1][0]]
            y = [points[0][1], segment1.End[1],  points[-1][1]]
            Graphs.Graph_02 (spline[1], spline[2], x, y, "X, мм", "Y, мм ", "Траектория у угле", "Скругление", "Исходная траектория", "Скругление") #график скругления
            #график траектории
            x = [segment1.Start[0], segment1.End[0], segment2_endP[0]]
            y = [segment1.Start[1], segment1.End[1], segment2_endP[1]]
            x_1 = []
            y_1 = []
            x_1.append(segment1.Start[0])
            x_1 += spline[1]
            x_1.append(segment2_endP[0])
            y_1.append(segment1.Start[1])
            y_1 += spline[2]
            y_1.append(segment2_endP[1])
            Graphs.Graph_02 (x_1, y_1, x, y, "X, мм", "Y, мм ", "Траектория", "Скругление", "Исходная траектория", "Скругление плюс траектория") #график скругления
            """
            """проверка угловой ошибки (эпсилон)"""
            p_u05 = spline[4]
            e_real = round(sqrt((pow((segment1.End[0] - p_u05[0]), 2))+(pow((segment1.End[1] - p_u05[1]), 2))), 4)
            print("Полученная угловая ошибка E = " + str(e_real) + " мм.")

        """Организация данных участков"""
        if segment1.StopMode == 1:
            end1 = points[0]
            start2 = points[5]
            #если было рассчитано скругление, то создается дополнительный участок кривой в угле
            segC = InputPlanner.SegmentPreAnalysis(5,                #Тип G-кода 5: угол
                                                   end1,             #Начальная точка сегмента
                                                   start2,           #Конечная точка сегмента
                                                   [],               #Координаты центра окружности (при движении по дуге)
                                                   0,                #Радиус окружности (при движении по дуге)
                                                   k[3],             #Рассчитанная скорость в угле
                                                   0,                #Статус участка (будет корректироваться)
                                                   0,                #для углов = 0
                                                   segment1.CornPar, #Параметры скругления (e и n)
                                                   0,                #Номер участка (будет корректироваться)
                                                   lengthSpline,     #Длина скругления
                                                   points,           #Контрольные точки в угле
                                                   )
        else:
            end1 = segment1.End
            start2 = segment1.End
        #если скругления не было, то создается линейный участок
        seg1 = InputPlanner.SegmentPreAnalysis(segment1.GCode,    #Тип G-кода как в исходном участке
                                               segment1.Start,    #Начальная точка сегмента
                                               end1,              #Конечная точка сегмента, корректируется скруглением
                                               segment1.Center,   #Координаты центра окружности (при движении по дуге)
                                               segment1.Radius,   #Радиус окружности (при движении по дуге)
                                               segment1.Feed,     #Подача
                                               0,                 #Статус участка (будет корректироваться)
                                               segment1.StopMode, #Наличие скругления в угле после данного участка
                                               segment1.CornPar,  #Параметры скругления (e и n)
                                               0,                 #Номер участка (будет корректироваться)
                                               length1,           #Длина участка
                                               []                 #Параметры кривой(для скруглений или дуг)
                                               )
        #На выход передается обновленный текущий линейный участок (скорректирована длина участка), участок скругления (если есть),
        #обновленная начальная точка следующего участка.
        if segment1.StopMode == 1:
            output = [seg1, segC, start2]
        else:
            output = [seg1, start2]

    #если участок - дуга
    elif segment1.GCode in [2, 3]:
        segment1.Radius = sqrt((pow((segment1.Center[0] - segment1.Start[0]), 2)) + (pow((segment1.Center[1] - segment1.Start[1]), 2)))
        print("Радиус участка R = " + str(segment1.Radius) + " мм.")
        length = PathLength(segment1.GCode, segment1.Start, segment1.End, segment1.Center, segment1.Radius)
        print("Длина участка L = " + str(length) + " мм.")
        arc_par = ArcParam (segment1.Start, segment1.End, segment1.Center, segment1.Radius)
        print("Параметр начала и конца дуги u = " + str(arc_par) + " рад.")
        seg1 = InputPlanner.SegmentPreAnalysis(segment1.GCode,    #Тип G-кода как в исходном участке
                                               segment1.Start,    #Начальная точка сегмента
                                               segment1.End,      #Конечная точка сегмента, корректируется скруглением
                                               segment1.Center,   #Координаты центра окружности (при движении по дуге)
                                               segment1.Radius,   #Радиус окружности (при движении по дуге)
                                               segment1.Feed,     #Подача
                                               0,                 #Статус участка (будет корректироваться)
                                               segment1.StopMode, #Наличие скругления в угле после данного участка
                                               segment1.CornPar,  #Параметры скругления (e и n)
                                               0,                 #Номер участка (будет корректироваться)
                                               length,            #Длина участка
                                               arc_par            #Параметры кривой(для скруглений или дуг)
                                               )
        #выходные данные: сформированный блок текущего сегмента и начальная точка следующего блока
        output = [seg1, segment1.End]
        
    #если участок - NURBS кривая
    elif segment1.GCode == 4:
        print("Участок - NURBS кривая.")
        param = NURBS_PrePros(segment1.NURBSpar[0], segment1.NURBSpar[1], segment1.NURBSpar[2], acc_max, j_max, e_con, segment1.Feed, Tint)
        #выходные данные: блоки подкривых, начальная точка след. блока
        seg_list = []
        for i in range (len(param)):
            par = [segment1.NURBSpar[0], segment1.NURBSpar[1], segment1.NURBSpar[2], [param [i][1], param [i][2]], param [i][3]]
            seg = InputPlanner.SegmentPreAnalysis(segment1.GCode,    #Тип G-кода как в исходном участке
                                                  segment1.Start,    #Начальная точка сегмента
                                                  segment1.End,      #Конечная точка сегмента, корректируется скруглением
                                                  segment1.Center,   #Координаты центра окружности (при движении по дуге)
                                                  segment1.Radius,   #Радиус окружности (при движении по дуге)
                                                  segment1.Feed,     #Подача
                                                  0,                 #Статус участка (будет корректироваться)
                                                  segment1.StopMode, #Наличие скругления в угле после данного участка
                                                  segment1.CornPar,  #Параметры скругления (e и n)
                                                  0,                 #Номер участка (будет корректироваться)
                                                  param[i][0],       #Длина участка
                                                  par                #Параметры кривой(для скруглений или дуг)
                                                  )
            seg_list.append(seg)
        output = [seg_list, segment1.End]
        
    return output

