import time
import Arduino
from Move import move
from multiprocessing import Process, Queue
from threading import Thread
import math
# import Queue as queue
import sys
import A_Star
from multiprocessing import Process,Queue
from matplotlib import pyplot
import threading
# import numpy as np
# import pandas as pd
# from matplotlib import pyplot
# from sklearn.model_selection import train_test_split
# from sklearn.preprocessing import LabelEncoder
# from keras.models import Sequential,load_model
# from keras.layers import Dense,Input
# import tensorflow as tf
# from tensorflow import keras
# import csv

astar = A_Star.AStar(500)
start = end = 0
path = 0

n = dirX = dirY = 0
grid = 500
trig = 0

ser = Arduino.Serial('raspi', 'usb')
Od = Arduino.Serial('raspi','gpio')
robot = move()
queue = Queue()

a = c = 0
b = 10000
d = -10000
fcode = 0
data1 = data2 = data3 = data4 = 0
dataUsb1 = dataUsb2 = dataUsb3 = dataUsb4 = 0
x = 0
y = 0
w = 0
dribble = 0
kick = 0
encoX = encoY = angleZ = ball = 0
kp=ki=kd=0
apa = 0
kendali = ''
thread_die = False
xtujuan=ytujuan=0
xStar = yStar = 0

step = 1
stepA = 1
angleDestination = 0
fx = fy = fpath = 0

setpoint = 0
findBall = 1
trigDistance = 0
finishPoint = 0
stepAuto = 0
stepFind = fAngleZ = 1
stepP = angleDestination = angleDestinationFriend = 1
angleBall = 0
ketemu = 0
find = findInDegree = 0
fcond = 0
get = 0
fX = fY = 0
passCond = 'notReady'

def findingObject(object, xF=0, yF=0):
    global ballDegree, ketemu, find, fcond, ballDistance
    global angleZ, fAngleZ, stepFind
    global xBola, yBola, angleBall, condition
    global encoX, encoY, angleZ, stepP, angleDestinationFriend, kendali, path
    global ballDegree

    if object == 'ball':
        if (ballDistance > 100 and ballDistance != 0 and ballDistance < 400) or (ballDegree != -36 and ballDegree > -6 and ballDegree < 6):
            ketemu = 1
        else:
            ketemu = 0
            condition = 0

        if not ketemu :
            find = 0
            # kalo bola keliatan di 360
            if ballDegree != -36:
                # print('nyari di 360')
                time.sleep(1.3) # waktu buat koleksi data kamera
                angleBall = int(ballDegree) + int(angleZ)
                # kejar bola sesuai derajat bola
                serArduino.write(ord('p'), angleBall,0,0,0,0)
                time.sleep(0.05)
                x = y = w = 0
                serArduino.write(0,x,y,w)

            else:
                    #robot muter ke kanan buat nyari
                # print('nyari sambil muter')
                # if ballDistance < 400:
                    mqtt.send(1, 0, 0, 0, 0)
                    if condition == 4 :
                        serArduino.write(0,0,0,-30)
                        angleBall = int(angleZ) - 10
                    elif condition == 5:
                        serArduino.write(0,0,0,30)
                        angleBall = int(angleZ) + 10
                    elif condition == 2:            #Kiri
                        angleBall = int(angleZ)
                        serArduino.write(ord('p'), angleBall)
                        time.sleep(0.05)
                        serArduino.write(0,0,0,0)
                    elif condition == 3:            #Kanan
                        angleBall = int(angleZ)
                        serArduino.write(ord('p'), angleBall)
                        time.sleep(0.05)
                        serArduino.write(0,0,0,0)
                    elif condition == 1:
                        angleBall = int(angleZ)
                        serArduino.write(ord('p'), angleBall)
                        time.sleep(0.05)
                        serArduino.write(0,0,0,0)
                    
                    if condition == 0:
                        if fcond == 4:
                            serArduino.write(0,0,0,-40)
                        elif fcond == 5:
                            serArduino.write(0,0,0,40)
                        else:
                            serArduino.write(0,0,0,40)

        if  ballDistance > 100 and ballDistance != 0 and ballDegree == -36 and ballDistance < 400:
            # print('ketemu di luar 360')
            # print('ketemu2')
            if ballDistance >= 100:
                if condition == 4:
                    # serArduino.write(0,0,0,-30)
                    angleBall = int(angleZ) - 10
                    serArduino.write(ord('p'), angleBall)
                    time.sleep(0.03)
                elif condition == 5:
                    # serArduino.write(0,0,0,30)
                    angleBall = int(angleZ) + 10
                    serArduino.write(ord('p'), angleBall)
                    time.sleep(0.03)
                    # serArduino.write(0,0,0,0,0,0)
                    # time.sleep(1)
                elif condition==1:
                    angleBall = int(angleZ)
                    serArduino.write(ord('p'), angleBall)
                    time.sleep(0.03)
            serArduino.write(0,0,0,0,0,0)

    elif object == "friend":
        kuadran = a = b = 0
        if xF == '-': xF = encoX 
        else: xF = int(xF)
        if yF == '-': yF = encoY 
        else: yF = int(yF) + 1
        
        if stepP == 1:
            print(xF, ' ', yF)
            angleDestinationFriend = round(abs(math.degrees(math.atan((xF-encoX)/(yF-encoY)))))
            print(f'sudut awal = {angleDestinationFriend}')
            stepP = 2

        elif stepP == 2:
            print("Aku di 2")
            # Menentukan sudut tujuan
            if xF > encoX and yF > encoY: 
                kuadran = 1
                a = 0 + angleDestinationFriend
                b = -360 + angleDestinationFriend
            elif xF < encoX and yF > encoY:
                kuadran = 2
                a = 0 - angleDestinationFriend
                b = 360 - angleDestinationFriend
            elif xF < encoX and yF < encoY:
                kuadran = 3
                a = 180 + angleDestinationFriend
                b = -180 + angleDestinationFriend
            elif xF > encoX and yF < encoY:
                kuadran = 4
                a = 180 - angleDestinationFriend
                b = -180 - angleDestinationFriend
            elif xF == encoX and yF > encoY: 
                kuadran = 11   # 0 derajat robot
                a = 0
                b = -1
            elif xF > encoX and yF == encoY: 
                kuadran = 12   # 90 atau -270 derajat robot
                a = 90
                b = -270
            elif xF == encoX and yF < encoY: 
                kuadran = 13   # 180 atau -180 derajat robot
                a = 180
                b = -180
            elif xF < encoX and yF == encoY: 
                kuadran = 14   # 270 atau -90 derajat robot
                a = 270
                b = -90
            if (abs(a - angleZ)) < (abs(b - angleZ)):
                angleDestinationFriend = a
            else:
                angleDestinationFriend = b

            # print(f'angle = {angleDestination}')
            serArduino.write(ord('p'), angleDestinationFriend)
            time.sleep(0.05)
            stepP = 3

        elif stepP == 3:
            # time.sleep(1)
            if angleZ >= (angleDestinationFriend-3) and angleZ <= (angleDestinationFriend+3):
                # if angleZ == angleDestination:
            # print("Aku di 3")
            # time.sleep(0.5)
                # serArduino.write(5)
                stepP = 1
            # kendali = 0
                return 'done'

    elif object == "goal":
        xF = 2900
        yF = 4500
        kuadran = a = b = 0
        if xF == '-': xF = encoX 
        else: xF = int(xF)
        if yF == '-': yF = encoY 
        else: yF = int(yF) + 1
        
        if stepP == 1:
            print(xF, ' ', yF)
            angleDestinationFriend = round(abs(math.degrees(math.atan((xF-encoX)/(yF-encoY)))))
            # print(f'sudut awal = {angleDestination}')
            stepP = 2

        elif stepP == 2:
            # print("Aku di 2")
            # Menentukan sudut tujuan
            if xF > encoX and yF > encoY: 
                kuadran = 1
                a = 0 + angleDestinationFriend
                b = -360 + angleDestinationFriend
            elif xF < encoX and yF > encoY:
                kuadran = 2
                a = 0 - angleDestinationFriend
                b = 360 - angleDestinationFriend
            elif xF < encoX and yF < encoY:
                kuadran = 3
                a = 180 + angleDestinationFriend
                b = -180 + angleDestinationFriend
            elif xF > encoX and yF < encoY:
                kuadran = 4
                a = 180 - angleDestinationFriend
                b = -180 - angleDestinationFriend
            elif xF == encoX and yF > encoY: 
                kuadran = 11   # 0 derajat robot
                a = 0
                b = -1
            elif xF > encoX and yF == encoY: 
                kuadran = 12   # 90 atau -270 derajat robot
                a = 90
                b = -270
            elif xF == encoX and yF < encoY: 
                kuadran = 13   # 180 atau -180 derajat robot
                a = 180
                b = -180
            elif xF < encoX and yF == encoY: 
                kuadran = 14   # 270 atau -90 derajat robot
                a = 270
                b = -90
            if (abs(a - angleZ)) < (abs(b - angleZ)):
                angleDestinationFriend = a
            else:
                angleDestinationFriend = b

            # print(f'angle = {angleDestination}')
            ser.write(ord('p'), angleDestinationFriend)
            time.sleep(0.05)
            stepP = 3

        elif stepP == 3:
            # time.sleep(1)
            if angleZ >= (angleDestinationFriend-3) and angleZ <= (angleDestinationFriend+3):
                # if angleZ == angleDestination:
            # print("Aku di 3")
            # time.sleep(0.5)
                # serArduino.write(5)
                stepP = 1
            # kendali = 0
                return 'done'

def aStar(x, y):
    global path, xtujuan, ytujuan, kendali, stepA
    global fx, fy, fpath

    if stepA == 1:
        xMap = int(x/500) # dibagi grid map nya
        yMap = int(y/500)
        xStart = round(encoX/500)
        yStart = round(encoY/500)
        
        start = (xStart, yStart)
        end = (xMap, yMap) # Grid 11x8, perhitungan dimulai dari 0
        # start = (0,0)
        # end = (2,2)

        path = astar.solve(start, end)

        for i in range(len(path)):
            xi = (path[i][0]) * 500
            yi = (path[i][1] )* 500
            path.pop(i)
            path.insert(i,(xi, yi))
        path.pop()
        path.append((x, y))
        path.pop(0)
        path.insert(0,(encoX, encoY))
        if path != fpath:
            # print(path,' ', encoX,' ', encoY)
            fpath = path
        stepA = 2

    elif stepA == 2:
        point = 0
        print(path)
        if len(path) > 1:
            x1 = path[1][0]
            y1 = path[1][1]
            
            if len(path) > 2:
                x2 = path[2][0]
                y2 = path[2][1]
                print(x1,' ', x2,' ', y1,' ', y2)
                sudutNext = round(abs(math.degrees(math.atan((x2-x1)/(y2-y1)))))
                print(sudutNext)
                if sudutNext < 4:
                    point = pointToPoint(x2, y2)
                else:
                    point = pointToPoint(x1, y1)
            else:
            # time.sleep(0.05)
                point = pointToPoint(x1, y1)
            # print(path, ' ',encoX, ' ',encoY)
        else:
            print('Kelar Bos')
            stepA = 1
            kendali = 0
        if point == 'done':
            # kendali = 0
            print('sudah')
            path.pop(0)
            # print(encoX,' ', encoY)
            if len(path) == 1:
                kendali = 0
            stepA = 1

        
    # if point == 'done':
    #     print(f'len path = {len(path)}')
    #     time.sleep(0.1)
    #     path.pop(0)

def pointToPoint(xF, yF):
    global encoX, encoY, angleZ, step, angleDestination, kendali, path
    kuadran = a = b = 0
    if xF == '-': xF = encoX 
    else: xF = int(xF)
    if yF == '-': yF = encoY 
    else: yF = int(yF) + 1
    
    if step == 1:
        print(xF, ' ', yF)
        angleDestination = round(abs(math.degrees(math.atan((xF-encoX)/(yF-encoY)))))
        print(f'sudut awal = {angleDestination}')
        step = 2

    elif step == 2:
        print("Aku di 2")
        # Menentukan sudut tujuan
        if xF > encoX and yF > encoY: 
            kuadran = 1
            a = 0 + angleDestination
            b = -360 + angleDestination
        elif xF < encoX and yF > encoY:
            kuadran = 2
            a = 0 - angleDestination
            b = 360 - angleDestination
        elif xF < encoX and yF < encoY:
            kuadran = 3
            a = 180 + angleDestination
            b = -180 + angleDestination
        elif xF > encoX and yF < encoY:
            kuadran = 4
            a = 180 - angleDestination
            b = -180 - angleDestination
        elif xF == encoX and yF > encoY: 
            kuadran = 11   # 0 derajat robot
            a = 0
            b = -1
        elif xF > encoX and yF == encoY: 
            kuadran = 12   # 90 atau -270 derajat robot
            a = 90
            b = -270
        elif xF == encoX and yF < encoY: 
            kuadran = 13   # 180 atau -180 derajat robot
            a = 180
            b = -180
        elif xF < encoX and yF == encoY: 
            kuadran = 14   # 270 atau -90 derajat robot
            a = 270
            b = -90
        if (abs(a - angleZ)) < (abs(b - angleZ)):
            angleDestination = a
        else:
            angleDestination = b

        print(f'angle = {angleDestination}')
        ser.write(ord('p'), angleDestination)
        time.sleep(0.05)
        step = 3

    elif step == 3:
        # time.sleep(1)
        if angleZ >= (angleDestination-2) and angleZ <= (angleDestination+2):
            # if angleZ == angleDestination:
            print("Aku di 3")
            # time.sleep(0.5)
            ser.write(5)
            step = 4
            # step = 1
            # kendali = 0
    
    elif step == 4:
        # if (math.hypot(encoX, encoY)) <= (math.hypot(xF, yF) - 100) or (math.hypot(encoX, encoY)) >= (math.hypot(xF, yF) + 10):
    
        # if (encoY <= (yF - 30) or encoY >= (yF + 30)) and (encoX <= (xF - 30) or encoX >= (xF + 30)) or (math.hypot(encoX, encoY)) <= (math.hypot(xF, yF) - 10) or (math.hypot(encoX, encoY)) >= (math.hypot(xF, yF) + 10):
        if (encoY <= (yF - 30) or encoY >= (yF + 30)) or (encoX <= (xF - 30) or encoX >= (xF + 30)):
            # or (math.hypot(encoX, encoY)) <= (math.hypot(xF, yF) - 10) or (math.hypot(encoX, encoY)) >= (math.hypot(xF, yF) + 10):
            x = -5
            y = 130
            w = 0
            dribble = 1
            ser.write(0, x, y, w, dribble, 0)
            # print("Aku di 2")
        else:
            ser.write(0, 0, 0, 0, 0, 0)
            time.sleep(0.1)
            ser.write(4)
            step = 1
            print('done di 4')
            # time.sleep(2)
            kendali = 0
            return 'done'
       

def terimaArd():
    fData = ''
    while True:
        # try:
        #data1 = enconder x, #data2 = encoder y, data3 = head, data = ball
        data = ser.readArduino()
        if fData != data:
            recv = list(data)
            recv.insert(0, 'A')
            queue.put(recv)
            # print(recv)
            fData = data
            # print(f'data = {data}')

def dataArduino():
    global encoX, encoY, angleZ, ball
    global kendali, xtujuan, ytujuan, xStar, yStar, path
    global cond,distB,degB,KrdX,KrdY
    (encoX,encoY)=(5500, 1740)
    while True:
        if queue.empty() is False:
            packet = queue.get()
            # print(packet)
            if packet[0] == "A":
                encoX = (int(packet[1]))/10
                encoY = (int(packet[2]))/10
                angleZ = int(packet[3])
                ball = int(packet[4])
                print('packet = ',packet)
                # print("data = ", encoX,' ', encoY,' ',angleZ,' ',ball,' ',xtujuan,' ',ytujuan,' ',round(math.hypot(encoX, encoY)),' ' ,round(math.hypot(int(xtujuan), int(ytujuan))))
                # time.sleep(0.01)
            elif packet[0]=="O":
                cond = int(packet[1])
                distB= int(packet[2])
                degB= int(packet[3])
                KrdX= int(packet[4])
                KrdY= int(packet[5])

        if kendali is 'point':
            pointToPoint(xtujuan,ytujuan);

        elif kendali is 'astar':
            aStar(xStar, yStar)

        if thread_die is True:
            print("Thread Die")
            break

def dataOdroid():
    fData=''
    while True:
        DataIn=Od.readOdroid()
        if fData!=DataIn:
            rec = list(DataIn)
            rec.insert(0,'O')
            # print(rec)
            queue.put(rec)
            fData=DataIn
class JST:
    def __init__(self):
        #--Parse Input--#
        #protocol [jrkX,jrkY,head,XBola,jrkBola]
        self.d1=self.d2=self.d3=self.d4=self.d5=self.d6=self.d7=0
    def ParseParam (self,masukan):
        if masukan[0]=="A":
            [self.d1,self.d2,self.d3] = [int(masukan[1]),int(masukan[2]),int(masukan[3])]
        elif masukan[0]=="O":
            [self.d4,self.d5,self.d6,self.d7]=[int(masukan[1]),int(masukan[2]),int(masukan[3]),int(masukan[4])]
        return self.d1,self.d2,self.d3,self.d4,self.d5,self.d6,self.d7
    def lakukan (self,xPix,yPix):
        xPix = round(xPix/16,1)
        yPix = round(yPix/16,1)
        xkorJrk = np.asarray([[xPix,yPix]]).astype(np.float32)
        # print(xkorJrk)
        ypred=model.predict(xkorJrk)
        bit2=float(ypred[0,0])
        bit1=float(ypred[0,1])
        bit0=float(ypred[0,2])
        # print("bit0: "+str(bit0)+" bit1: "+str(bit1)+" bit2:"+str(bit2))
        if bit2>bit1 and bit2>bit0:
            self.out="x"#DIAM
            self.prob=bit2
        elif bit1>bit0 and bit1>bit2:
            self.out="d"#KANAN
            self.prob=bit1
        elif bit0>bit1 and bit0>bit2:
            self.out="a"#KIRI
            self.prob=bit0
        # if 0.5<=bit2:
        #     self.out="DIAM"
        # elif 0.5<=bit1:
        #     self.out="KANAN"
        # elif 0.5<=bit0:
        #     self.out="KIRI"
        # print(self.out)
        return self.out,self.prob
# jst = JST()
# model = keras.models.load_model("Plan2_20x20_HuangV5.h5")

p1 = Process(target = terimaArd)
p2 = Thread(target = dataArduino)
p3 = Process(target=dataOdroid)
fGrk=""
cepat=0
if __name__== '__main__':
    p1.start()
    p2.start()
    p3.start()
    time.sleep(3)
    # ser.write(1, 5500, 1740)
    robot.resetEnco(5500, 1740)
    print("Reset Encoder telah dilakukan")
    fDiam=False
    while True:
        try: #Satuan kordinat X&Y Lapangan milimeter        
            # print('\t\t\t\t\t||', encoY)
            kendali = (input("Masukan kendali : "))
            # if KrdY>143 and ball!=1:
            #     fGrk=jst.lakukan(KrdY,KrdX)
            #     kendali=fGrk[0]
            #     cepat=float(fGrk[1])*130
            # fGrk=jst.lakukan(120,160)
            # print(fGrk,KrdY)
            # print(KrdY,KrdX)
            # if KrdY >=120 and ball!=1:
            #     if cond==1 or cond==0:
            #         kendali='x'
            #     elif cond==2:
            #         kendali='a'
            #     elif cond==3:
            #         kendali='d'
            # print(fEncoL,fEncoR,cond,encoX,kendali)
            # if ball==1:
            #     fDiam=True
            
            # if fDiam==True:
            #     robot.berhenti()
            #     if KrdY<120:
            #         fDiam=False
            #         print("sudah selesai")
            if kendali is 'p':
                # Setpoint
                set = input("            Masukan Setpoint : ")
                apa = 'p'
                print('setpoint = ', set)
                robot.sudut(set)

            elif kendali is 'l':
                xtujuan = (input('        Masukan X Tujuan: '))
                ytujuan = (input('        Masukan Y Tujuan: '))
                kendali = 'point'

            elif kendali is 'u':
                xStar = int((input('        Masukan X Map: ')))
                yStar = int((input('        Masukan Y Map: ')))
                kendali = 'astar'
            #---titk 0 kotak start
            elif kendali is '1': 
                robot.resetEnco(5500, 1740)

            elif kendali is '2':
                robot.resetKompas()    
            # Kick
            elif kendali is 'n':
                friend = findingObject('friend', 200, 300)

            # Kick
            elif kendali is 'k':
                print("Kick")
                x = 0
                y = 0
                w = 0
                dribble = 0
                kick = 1
                # apa = 'w'
                # ser.write(5, 0, 0, 0, 0, 0)
                # time.sleep(0.05)
                ser.write(0, x, y, w, dribble, kick)

            # Charge
            elif kendali is 'j':
                print("Charge")
                x = 0
                y = 0
                w = 0
                dribble = 0
                kick = 0
                # apa = 'w'
                # ser.write(5, 0, 0, 0, 0, 0)
                # time.sleep(0.05)
                ser.write(0, x, y, w, dribble, kick)

            # Maju
            elif kendali is 'w':
                # print("aku disini bro")
                robot.maju()

            # Mundur
            elif kendali is 's':
                robot.mundur()
            
            # Kiri
            elif kendali is 'a':
                robot.kiri()
                # robot.kiri_speed(cepat)

            # Kanan
            elif kendali is 'd':
               robot.kanan()
            #    robot.kanan_speed(cepat)

            # Diagonal Kanan Atas
            elif kendali is 'e':
                robot.diagonalKananAtas()

            # Diagonal Kiri Atas
            elif kendali is 'q':
                robot.diagonalKiriAtas()
            
            # Diagonal Kiri Bawah
            elif kendali is 'z':
                robot.diagonalKiriAtas
                
            # Diagonal Kanan Bawah
            elif kendali is 'c':
                robot.diagonalKananBawah()

            elif kendali is 'r':
               robot.rotasiKiri()

            elif kendali is 't':
                robot.rotasiKanan()

            elif kendali is 'x':
                robot.berhenti()
                
            # Stop
            else:
                x = 0
                y = 0
                w = 0
                dribble = 0

            # x += 1
            # y += 1
            # w -= 1
            # dribble += 1
            # time.sleep(0.02)
            # ser.write(0, x, 50, w, dribble, kick)
            # print(x, ' ', y, ' ', w)
            
        except KeyboardInterrupt:
            thread_die = True
            p2.join()
            # p2.setDaemon(True)
            print("Exit nih gua")
            # p1.join()
            p1.terminate()
            p3.terminate()
            # p2.terminate()
            sys.exit(0)
            
            