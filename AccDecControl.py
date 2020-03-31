import InputPlanner
from math import pi, cos, sin, fabs, sqrt, ceil, floor

"""Предварительный просмотр кадров. Нахождение конечной скорости участка.
Входные данные:
1. Конечная скорость предыдущего участка - начальная скорость текущего участка Vei-1 = Vsi, мм/с;
2. Список длин следующих N участков, мм;
3. Список подач следующих N участков, мм/с;
4. Число N;
5. Допустимая скорость Vmax, мм/с;
6. Допустимое ускорение Amax, мм/с2;
7. Допустимый рывок Jmax, мм/с3;
8. Время интерполяции T, с;
9. Допустимая линейная ошибка Emax, мм.
Выходные данные:
Конечная скорость участка Vei, мм/с."""
def LinkedFeedrate (Vend_last, l_list, feed_list, Nm, Vemax, N, j_max, acc_max, Tint, maxErr, typeG_l, Econ, curv, Ech, rad):
    if N == 1:
        Vend = 0
    else:
        Ve0 = EndingVelCalc(l_list[0], Vend_last, feed_list[0], j_max, acc_max, Tint, Nm, maxErr)
        K = N - 1
        Ve1 = 0
        while K >= 1:
            Ve1 = EndingVel(l_list[K], Ve1, feed_list[K], feed_list[K-1], j_max, acc_max, Tint, Nm, maxErr)
            if typeG_l[K] == 4:
                Vnurbs = min(2/Tint*sqrt((2*Econ)/curv[K]-Econ*Econ), sqrt(acc_max/curv[K]), sqrt(j_max/(curv[K]*curv[K])))
                Ve1 = min(Ve1, Vemax, Vnurbs)
            elif typeG_l[K] in [2,3]:
                Varc = sqrt(8*rad[K]*Ech)/Tint
                Ve1 = min(Ve1, Vemax, Varc)
            else:
                Ve1 = min(Ve1, Vemax)
            K -= 1
        Vend = min(Ve0, Ve1)
    return Vend

#actual ending velocity
def EndingVel (length, Vstart, Vobj, Vnext, j_max, acc_max, Tint, Nm, maxErr):
    if Vnext >= Vobj:
        if Vstart == Vobj or length >= AccDecDisplacement (Vstart, Vobj, j_max, acc_max, Tint, Nm):
            Ve = Vobj
        else:
            Ve = EndingVelCalc(length, Vstart, Vobj, j_max, acc_max, Tint, Nm, maxErr)
    elif Vnext < Vobj:
        if length >= AccDecDisplacement (Vstart, Vnext, j_max, acc_max, Tint, Nm):
            Ve = Vnext
        else:
            Ve = EndingVelCalc(length, Vstart, Vnext, j_max, acc_max, Tint, Nm, maxErr)
    return Ve

def EndingVelCalc(length, Vstart, Vobj, j_max, acc_max, Tint, Nm, maxErr):
    vl_k = min(Vstart, Vobj)
    vh_k = max(Vstart, Vobj)
    ve_k = (1/2)*(vl_k + vh_k)
    S_k = AccDecDisplacement (Vstart, ve_k, j_max, acc_max, Tint, Nm)
    k = 0
    S_klength_fabs = fabs(S_k - length)
    while S_klength_fabs > maxErr and k < 50:
        if (Vstart - Vobj)*(S_k - length) > 0:
            vl_k = ve_k
        else:
            vh_k = ve_k
        ve_k = (1/2)*(vl_k + vh_k)
        S_k = AccDecDisplacement (Vstart, ve_k, j_max, acc_max, Tint, Nm)
        k += 1
        #if k == 49:
            #print("Достигнуто максимальное число итераций")
            #print(S_klength_fabs)
    Vending = ve_k
    return Vending
    
"""Расчет длины пути при разгоне или торможении от V1 до V2. Вспомогательная функция.
Входные данные:
1. Стартовая скорость V1, мм/с;
2. Конечная скорость V2, мм/с;
3. Допустимый рывок Jmax, мм/с3;
4. Допустимое ускорение Amax, мм/с2;
5. Время интерполяции T, с;
6. Вспомогательная величина Nm.
Выходные данные:
Длина разгона или торможения Laccdecc, мм."""
def AccDecDisplacement (V1, V2, j_max, acc_max, Tint, Nm):
    acclist = []
    vellist = []
    if V1 == V2: #разгон/торможение не требуются
        vellist = []
        LAccDec = 0
    else:
        J = 0
        n1 = 0
        n2 = 0
        V1V2fabs = fabs(V1 - V2)
        M1 = (V1V2fabs)/(Nm*j_max*(Tint*Tint)) - Nm
        M2 = sqrt(V1V2fabs/(j_max*(Tint*Tint)))
        #print(M2)
        if M1 > 0:
            n1 = Nm
            n2 = ceil(M1) #ceiling
        elif M1 <= 0:
            n1 = ceil(M2) #ceiling
            n2 = 0
        A2 = 1/((2*n1+n2-1)*Tint)*(V1V2fabs -(n1 - 1)*(n1 + n2 - 1)*j_max*(Tint*Tint))
        #print(A2)
        if V1 < V2:
            a2 = A2
            J = j_max
        elif V1 > V2:
            a2 = -A2
            J = - j_max
        a = (2*n1+n2)*V1*Tint
        b = 1/2*a2*(4*(n1**2)+4*n1*n2+n2**2-2*n1-n2)*(Tint*Tint)
        c = 1/2*(2*n1**3+3*(n1**2)*n2+n1*(n2**2)-4*n1**2-4*n1*n2-n2**2+2*n1+n2)*J*(Tint*Tint*Tint)
        LAccDec = a + b + c
    return LAccDec

#истинная максимальная скорость
"""Расчет максимальной скорости на участке. Алгоритм дихотомии.
Входные данные:
1. Начальная скорость Vs, мм/с;
2. Конечная скорость Ve, мм/с;
3. Подача, мм/с;
4. Длина участка L, мм;
5. Вспомогательная величина Nm;
6. Допустимая линейная ошибка Emax, мм;
7. Допустимый рывок Jmax, мм/с3;
8. Допустимое ускорение Amax, мм/с2;
9. Время интерполяции T, с;
Выходные данные:
Максимальная скорость на участке Vmax, мм/с."""
def RealMaxFeedrate (Vstart, Vend, feedrate, length, Nm, maxErr, j_max, acc_max, Tint):
    LAcc = AccDecDisplacement(Vstart, feedrate, j_max, acc_max, Tint, Nm)
    LDec = AccDecDisplacement(feedrate, Vend, j_max, acc_max, Tint, Nm)
    if LAcc + LDec < length:
        #print("Сегмент содержит участок с постоянной скоростью.")
        maxfeed = feedrate
        s = LAcc + LDec
        Vlist = []
        Slist = []
    else:
        k = 0
        vl = max(Vstart, Vend)
        vh = feedrate
        #print('vl = ' + str(vl))
        #print('vh = ' + str(vh))
        vm_k = 1/2*(vl + vh)
        #print('vm_k = ' + str(vm_k))
        #print(type(vm_k))
        sm_k = AccDecDisplacement(Vstart, vm_k, j_max, acc_max, Tint, Nm) + AccDecDisplacement(vm_k, Vend, j_max, acc_max, Tint, Nm)
        #print('sm_k = ' + str(sm_k))
        Vlist = []
        Vlist.append(vm_k)
        Slist = []
        Slist.append(sm_k)
        sm_klength_fabs = fabs(sm_k - length)
        while sm_klength_fabs > maxErr and k < 50:
            #print('погрешность = ' + str(sm_k - length))
            #print('____________________________________________________________')
            if sm_k < length:
                vl = vm_k
                #print('vl = ' + str(vl))
                #print('vh = ' + str(vh))
            else:
                vh = vm_k
                #print('vl = ' + str(vl))
                #print('vh = ' + str(vh))
            vm_k = 1/2*(vl + vh)
            #print('vm_k = ' + str(vm_k))
            sm_k = AccDecDisplacement(Vstart, vm_k, j_max, acc_max, Tint, Nm) + AccDecDisplacement(vm_k, Vend, j_max, acc_max, Tint, Nm)
            #print('sm_k = ' + str(sm_k))
            Vlist.append(vm_k)
            Slist.append(sm_k)
            sm_klength_fabs = fabs(sm_k - length)
            k += 1
            #if k == 49:
                #print("Достигнуто максимальное число итераций")
                #print(sm_klength_fabs)
        maxfeed = vm_k
        s = sm_k
    return maxfeed

"""Построение профиля скорости на участке.
"""
def AccDecType (Vstart, Vend, Vmax, length, j_max, acc_max, Tint, Nm):
    Lconst_i = 0
    const = []
    Lconst = 0
    if Vstart < Vmax: #если имеется разгон
        #расчет длины разгона
        Lacc = AccDecDisplacement(Vstart, Vmax, j_max, acc_max, Tint, Nm)
        #расчет профиля разгона
        acc = AccVelProfiles(Vstart, Vmax, j_max, acc_max, Tint, Nm)[0]
    else:
        Lacc = 0
        acc = []
    if Vend < Vmax: #если имеется торможение
        #расчет длины торможения
        Ldec = AccDecDisplacement(Vmax, Vend, j_max, acc_max, Tint, Nm)
        #расчет профиля торможения
        dec = AccVelProfiles(Vmax, Vend, j_max, acc_max, Tint, Nm)[0]
    else:
        Ldec = 0
        dec = []
    l = Lconst + Lacc + Ldec
    while l < length: #если имеется участок с постоянной скоростью
        const.append(Vmax)
        Lconst_i = Tint*Vmax
        Lconst += Lconst_i
        l = l + Lconst_i
    return acc, const, dec, Lacc, Lconst, Ldec

"""Расчет профиля разгона/торможения.
Входные данные:
1. Стартовая скорость V1, мм/с;
2. Конечная скорость V2, мм/с;
3. Допустимый рывок Jmax, мм/с3;
4. Допустимое ускорение Amax, мм/с2;
5. Время интерполяции T, с;
6. Вспомогательная величина Nm.
Выходные данные:
1. Список скоростей для каждого времения интерполяции, мм/с;
2. Список ускорений для каждого времения интерполяции, мм/с2."""
def AccVelProfiles (V1, V2, j_max, acc_max, Tint, Nm):
    acclist = []
    vellist = []
    if V1 == V2: #разгон/торможение не требуются
        vellist = []
        LAccDec = 0
    else:
        J = 0
        n1 = 0
        n2 = 0
        V1V2fabs = fabs(V1 - V2)
        M1 = (V1V2fabs)/(Nm*j_max*(Tint*Tint)) - Nm
        M2 = sqrt(V1V2fabs/(j_max*(Tint*Tint)))
        if M1 > 0:
            n1 = Nm
            n2 = ceil(M1) #ceiling
        elif M1 <= 0:
            n1 = ceil(M2) #ceiling
            n2 = 0
        A2 = 1/((2*n1+n2-1)*Tint)*(V1V2fabs -(n1 - 1)*(n1 + n2 - 1)*j_max*(Tint*Tint))
        if V1 < V2:
            a2 = A2
            J = j_max
        elif V1 > V2:
            a2 = -A2
            J = - j_max
        for i in range (1, (2*n1+n2+1)):
            if i == 1:
                ai = 0
                vi = V1
                acclist.append(ai)
                vellist.append(vi)
            elif i > 1 and i <= n1 + 1:
                ai = a2 + (i - 2)*J*Tint
                vi = vellist[-1] + ai*Tint
                acclist.append(ai)
                vellist.append(vi)
            elif i > n1 + 1 and i <= n1 + n2 + 1:
                ai = a2 + (n1 - 1)*J*Tint
                vi = vellist[-1] + ai*Tint
                acclist.append(ai)
                vellist.append(vi)
            elif i > n1 + n2 + 1 and i <= 2*n1 + n2:
                ai = a2 + (2*n1 + n2 - i)*J*Tint
                vi = vellist[-1] + ai*Tint
                acclist.append(ai)
                vellist.append(vi)
    return vellist, acclist

#ACCELERATION/DECELERATION CONTROL MODULE
"""Модуль контроля скорости.
Осуществляет предварительный просмотр кадров и построение профиля скорости.
Входные данные: стартовая скорость текущего участка, N экземпляров класса SegmentPreAnalysis - N участков траектории,
                которые нужно проанализировать, системные параметры.
Выходные данные: экземпляр класса SegmentAccDecControl, участок после контроля скорости."""
def FeedControl (vst_i, segmentList, N, Tint, v_max, acc_max, j_max, err_max, err_con, err_ch):
    fList = []
    lList = []
    gList = []
    curvList = []
    rList = []
    for i in range (len(segmentList)):
        lList.append(segmentList[i].Length)
        fList.append(segmentList[i].Feed)
        gList.append(segmentList[i].GCode)
        if len(segmentList[i].Curve) == 4:
            curvList.append(segmentList[i].Curve[4])
            print(segmentList[i].Curve[4])
        else:
            curvList.append(1)
        rList.append(segmentList[i].Radius)
    vList = []
    iterList = []
    Nm = floor(acc_max/(j_max*Tint)) #вспомогательная величина, формула № из отчета
    #расчет конечной скорости участка
    vend_i = round(LinkedFeedrate (vst_i, lList, fList, Nm, v_max, N, j_max, acc_max, Tint, err_max, gList, err_con, curvList, err_ch, rList), 4)
    print("Начальная скорость Vs = " + str(round(vst_i, 4)) + " мм/c.")
    print("Подача F = " + str(fList[0]) + " мм/c.")
    print("Конечная скорость Ve = " + str(vend_i) + " мм/c.")
    #построение профиля скорости на текущем участке
    #расчет максимальной скорости на участке
    vMax = RealMaxFeedrate(vst_i, vend_i, fList[0], lList[0], Nm, err_max, j_max, acc_max, Tint)
    print("Максимальная скорость Vmax = " + str(vMax) + " мм/c.")
    #расчет скоростей для каждого периода интерполяции - профиль скорости
    velProfile = AccDecType (vst_i, vend_i, vMax, lList[0], j_max, acc_max, Tint, Nm)
    #print(velProfile[3:6])
    vList += velProfile[0] + velProfile[1] + velProfile[2]
    #формирование данных об участке после контроля скорости
    seg = InputPlanner.SegmentAccDecControl(segmentList[0].GCode,    #Тип G-кода как в исходном участке
                                            segmentList[0].Start,    #Начальная точка сегмента
                                            segmentList[0].End,      #Конечная точка сегмента, корректируется скруглением
                                            segmentList[0].Center,   #Координаты центра окружности (при движении по дуге)
                                            segmentList[0].Radius,   #Радиус окружности (при движении по дуге)
                                            vList,                   #Список скоростей на участке
                                            velProfile[3:6],         #Список периодов интерполяции
                                            segmentList[0].Status,   #Статус участка (будет корректироваться)
                                            segmentList[0].StopMode, #Наличие скругления в угле после данного участка
                                            segmentList[0].CornPar,  #Параметры скругления (e и n)
                                            segmentList[0].Num,      #Номер участка (будет корректироваться)
                                            segmentList[0].Length,   #Длина участка
                                            segmentList[0].Curve     #Контрольные точки (только для скруглений)
                                            )
    return seg
    
