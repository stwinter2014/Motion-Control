#Класс "Участок траектории после интерпретатора"
class SegmentInterpreter(object):
    def __init__(self,
                 nGCode,     #Тип G-кода (0: G00, 1: G01, 2: G02, 3: G03, 4: кривая)
                 pStart,     #Начальная точка сегмента
                 pEnd,       #Конечная точка сегмента
                 pCenter,    #Координаты центра окружности (при движении по дуге)
                 dRadius,    #Радиус окружности (при движении по дуге)
                 lNURBSpar,  #Параметры NURBS кривой [контрольные точки, весы, узловой вектор]
                 dFeed,      #Подача
                 nStatus,    #Статус сегмента (1: начальный, 2: конечный, 0: промежуточный)
                 nStopMode,  #Тип траектории на углу (0: точная остановка, 1: скругление)
                 lCornPar,   #Параметры кривой на углу (при построении скруглении)
                 nNum        #Номер участка
                 ):
        self.GCode = nGCode
        self.Start = pStart
        self.End = pEnd
        self.Center = pCenter
        self.Radius = dRadius
        self.NURBSpar = lNURBSpar
        self.Feed = dFeed
        self.Status = nStatus
        self.StopMode = nStopMode
        self.CornPar = lCornPar
        self.Num = nNum

#Класс "Участок после предварительного анализа"
class SegmentPreAnalysis(object):
    def __init__(self,
                 nGCode,    #Тип G-кода (0: G00, 1: G01, 2: G02, 3: G03, 4: кривая, 5: угол)
                 pStart,    #Начальная точка сегмента
                 pEnd,      #Конечная точка сегмента
                 pCenter,   #Координаты центра окружности (при движении по дуге)
                 dRadius,   #Радиус окружности (при движении по дуге)
                 dFeed,     #Подача
                 nStatus,   #Статус сегмента (1: начальный, 2: конечный, 0: остальное)
                 nStopMode, #Тип траектории на углу (0: точная остановка, 1: скругление)
                 lCornPar,  #Параметры кривой на углу (при построении скруглении)
                 nNum,      #Номер участка
                 dLength,   #Длина участка
                 lCurve     #Параметры кривой (контрольные точки, параметры дуги, параметры подкривых NURBS)
                 ):
        self.GCode = nGCode
        self.Start = pStart
        self.End = pEnd
        self.Center = pCenter
        self.Radius = dRadius
        self.Feed = dFeed
        self.Status = nStatus
        self.StopMode = nStopMode
        self.CornPar = lCornPar
        self.Length = dLength
        self.Num = nNum
        self.Curve = lCurve

#Класс "Участок после контроля скорости"
class SegmentAccDecControl(object):
    def __init__(self,
                 nGCode,    #Тип G-кода (0: G00, 1: G01, 2: G02, 3: G03, 4: кривая, 5: угол)
                 pStart,    #Начальная точка сегмента
                 pEnd,      #Конечная точка сегмента
                 pCenter,   #Координаты центра окружности (при движении по дуге)
                 dRadius,   #Радиус окружности (при движении по дуге)
                 lVel,      #Скорость для каждого периода интерполяции (итерации)
                 lIter,     #Номер итерации
                 nStatus,   #Статус сегмента (1: начальный, 2: конечный, 0: остальное)
                 nStopMode, #Тип траектории на углу (0: точная остановка, 1: скругление)
                 lCornPar,  #Параметры кривой на углу (при построении скруглении)
                 nNum,      #Номер участка
                 dLength,   #Длина участка
                 lCurve     #Параметры кривой (контрольные точки)
                 ):
        self.GCode = nGCode
        self.Start = pStart
        self.End = pEnd
        self.Center = pCenter
        self.Radius = dRadius
        self.Vel = lVel
        self.Iter = lIter
        self.Status = nStatus
        self.StopMode = nStopMode
        self.CornPar = lCornPar
        self.Length = dLength
        self.Num = nNum
        self.Curve = lCurve

#Класс "Участок после интерполяции"
class SegmentInterpolation(object):
    def __init__(self,
                 #nGCode,    #Тип G-кода (0: G00, 1: G01, 2: G02, 3: G03, 4: кривая, 5: угол)
                 #pStart,    #Начальная точка сегмента
                 #pEnd,      #Конечная точка сегмента
                 #pCenter,   #Координаты центра окружности (при движении по дуге)
                 #dRadius,   #Радиус окружности (при движении по дуге)
                 lVel,      #Скорость для каждого периода интерполяции (итерации)
                 lX,
                 lY,
                 lT,        #Время интерполяции
                 nStatus,   #Статус сегмента (1: начальный, 2: конечный, 0: остальное)
                 #nStopMode, #Тип траектории на углу (0: точная остановка, 1: скругление)
                 #lCornPar,  #Параметры кривой на углу (при построении скруглении)
                 nNum,      #Номер участка
                 dLength    #Длина участка
                 #lCurve     #Параметры кривой (контрольные точки)
                 ):
        #self.GCode = nGCode
        #self.Start = pStart
        #self.End = pEnd
        #self.Center = pCenter
        #self.Radius = dRadius
        self.Vel = lVel
        self.X = lX
        self.Y = lY
        self.T = lT
        self.Status = nStatus
        #self.StopMode = nStopMode
        #self.CornPar = lCornPar
        self.Length = dLength
        self.Num = nNum
        #self.Curve = lCurve
