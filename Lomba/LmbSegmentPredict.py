import time
import Arduino
from multiprocessing import Process, Queue
from threading import Thread
import math
# import Queue as queue
import sys
from matplotlib import pyplot
import threading
import numpy as np
import pandas as pd
from matplotlib import pyplot
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import LabelEncoder
from keras.models import Sequential,load_model
from keras.layers import Dense,Input
import tensorflow as tf
from tensorflow import keras

# astar = A_Star.AStar(500)
start = end = 0
path = 0

n = dirX = dirY = 0
grid = 500
trig = 0

ser = Arduino.Serial('raspi', 'usb')
serOdroid = Arduino.Serial('raspi', 'gpio')
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
ballDistance = enemyDistance = enemyDegree = condition = 0
KordX=KordY=ballDistance=ballDegree=0
step = 1
stepA = 1
angleDestination = 0
fx = fy = fpath = 0
class JST:
    def __init__(self):
        #--Parse Input--#
        #protocol [jrkX,jrkY,head,XBola,jrkBola]
        self.d1=self.d2=self.d3=self.d4=self.d5=self.d6=self.d7=0
    def lakukan (self,xPix,yPix):
        # model.summary()#cari mengenai summary
        # tes = np.asarray([xPix])
        # print(tes.shape)
        xkorJrk = np.asarray([[xPix,yPix]]).astype(np.float32)
        # print(xkorJrk)
        ypred=model.predict(xkorJrk)
        bit2=float(ypred[0,0])
        bit1=float(ypred[0,1])
        bit0=float(ypred[0,2])
        # print("bit0: "+str(bit0)+" bit1: "+str(bit1)+" bit2:"+str(bit2))
        if bit2>bit1 and bit2>bit0:
            self.out="DIAM"
        elif bit1>bit0 and bit1>bit2:
            self.out="KANAN"
        elif bit0>bit1 and bit0>bit2:
            self.out="KIRI"
        # if 0.5<=bit2:
        #     self.out="DIAM"
        # elif 0.5<=bit1:
        #     self.out="KANAN"
        # elif 0.5<=bit0:
        #     self.out="KIRI"
        # print(self.out)
        return self.out
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
        time.sleep(1)
        # if angleZ >= (angleDestination-2) and angleZ <= (angleDestination+2):
            # if angleZ == angleDestination:
        print("Aku di 3")
        # time.sleep(0.5)
        ser.write(5)
        step = 4
            # kendali = 0
    
    elif step == 4:
        # if (math.hypot(encoX, encoY)) <= (math.hypot(xF, yF) - 100) or (math.hypot(encoX, encoY)) >= (math.hypot(xF, yF) + 10):
    
        if (encoY <= (yF - 10) or encoY >= (yF + 10) and encoX <= (xF - 10) or encoX >= (xF + 10)) or (math.hypot(encoX, encoY)) <= (math.hypot(xF, yF) - 10) or (math.hypot(encoX, encoY)) >= (math.hypot(xF, yF) + 10):
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
       

def terima():
    fData = ''
    while True:
        # try:
        #data1 = enconder x, #data2 = encoder y, data3 = head, data = ball
        data = ser.readArduino()
        if fData != data:
            recv = list(data)
            recv.insert(0, 'A')
            queue.put(recv)
            fData = data
            # print(f'data = {data}')

def receiveOdroid():
    data1=data2=data3=data4=data5=0
    fData = ''
    # time.sleep(1)
    while True:
        recv = serOdroid.readOdroid()
        if fData != recv:
            # print(recv)
            data = list(recv)
            data.insert(0, 'O')
            queue.put(data)
            fData = recv

def dataArduino():
    global encoX, encoY, angleZ, ball
    global KordX,KordY
    global kendali, xtujuan, ytujuan, xStar, yStar, path
    global ballDistance, ballDegree, enemyDistance, enemyDegree, condition, fcond
    while True:
        if queue.empty() is False:
            packet = queue.get()
            if packet[0] == "A":
                encoX = (int(packet[1]))/10
                encoY = (int(packet[2]))/10
                angleZ = int(packet[3])
                ball = int(packet[4])
                # print('packet = ',packet)
                # print("data = ", encoX,' ', encoY,' ',angleZ,' ',ball,' ',xtujuan,' ',ytujuan,' ',round(math.hypot(encoX, encoY)),' ' ,round(math.hypot(int(xtujuan), int(ytujuan))))
                # time.sleep(0.01)
                
            elif packet[0] == 'O':          # terima data dari Odroid
                # ballDistance = int(packet[1]) + 12
                # ballDegree = int(packet[2])
                # enemyDistance = int(packet[3])
                # enemyDegree = int(packet[4])
                # condition = int(packet[5])
                # KordX = int(packet[6])
                # KordY = int(packet[7])
                ballDegree = int(packet[1])
                KordX = int(packet[2])
                KordY = int(packet[3])
                # print(packet[6],packet[7])

        if kendali is 'point':
            pointToPoint(xtujuan,ytujuan);

        elif kendali is 'astar':
            aStar(xStar, yStar)

        if thread_die is True:
            print("Thread Die")
            break
#-----Initial Segment-----#
sgmn = 20
(x_pixel,y_pixel)=(320,240)
Result=0
combine=0
Jrow=[]
[sgX,sgY]=[0,0]
bagiX=x_pixel/sgmn
bagiY=y_pixel/sgmn
[col,row]=[0,0]
DTawal=[]
DXInt=[]
DYInt=[]
grk=[]
Rtulis=False
Bt = 0
fTdat=False
vGrk=0
Cuplik=0
for D in range(0,2):
    DTawal.append(0)
DTawal.append(1)
DTawal=[DTawal]
#--create dimension for move obj on pixel 10x10--#
for SgIX in range(0,sgmn):
    DXInt.append(0)
for SgIY in range(0,sgmn):
    DYInt.append(DXInt)
DSegment = np.asarray(DYInt).astype(float)

def GridObj(x,y,Sudut):
    global Result,combine,Jrow,sgX,sgY,DTawal,DXInt,DYInt,grk
    global Rtulis,Bt,DSegment,fTdat
    if y> 159:
        for SgmnY in range(0,sgmn):
            if (bagiY*SgmnY)<y and y <= bagiY*(SgmnY+1):
                for SgmnX in range(0,sgmn):
                    if (bagiX*SgmnX)<x and x <= bagiX*(SgmnX+1):
                        DSegment[SgmnY,SgmnX]=1
                        Rtulis=True
    #--Save kordinat [y,x]--#
    dtSmntr = np.argwhere(DSegment==1)
    dtSmntr+=1
    #gabungin kordinat dan gerakan
    (rowK,colK)=dtSmntr.shape
    #--Grid kamera 360--#
    if y>=225:
        if Sudut<3 or Sudut>-3:
            vGrk=1
            print("DIAM")
            while Bt<rowK:
                grk.append([vGrk])
                Bt+=1
            Rtulis=True
            fTdat=True
        # if 190<=x_360 and y_360<=207 and y_360 >=165:
        #     vGrk=2
        #     print("KANAN")
        #     while Bt<rowK:
        #         grk.append([vGrk])
        #         Bt+=1
        #     Rtulis=True
        #     fTdat=True
        # if 173<=x_360 and y_360<=83 and y_360>=53:
        #     vGrk=3
        #     print("KIRI")
        #     while Bt<rowK:
        #         grk.append([vGrk])
        #         Bt+=1
        #     Rtulis=True
        #     fTdat=True
    #--reset data kordinat sementara ||Save dataset || gerakan sementara--
    if fTdat == True and Rtulis==True:
        Dataset = np.concatenate((dtSmntr,grk),axis=1)
        Dataset = np.append(DTawal,Dataset,axis=0)
        DTawal = Dataset
        # print(DSegment)
        # print("-Gerakan-")
        # print(grk)
        # print(len(grk))
        # print("-Kordinat-")
        # print(dtSmntr)
        # print(rowK,colK)
        # print(dtSmntr.shape) 
        print("--DATA SET--")
        print(Dataset)
        print(DSegment)
        print(Dataset.shape)
        Cuplik+=1
        print(Cuplik)
        # print(DTawal)
        grk.clear()
        Bt = 0
        # for Rowrst in range(0,rowK):   
        dtSmntr = np.delete(dtSmntr,0,0)
        dtSmntr = np.delete(dtSmntr,1,1)
        dtSmntr = np.delete(dtSmntr,0,1)
        DSegment *= 0
        # print("-DATA GERAKAN-")
        # print(grk)
        # print("-DATA SEMENTARA-")
        # print(dtSmntr)
        Rtulis=False
    if fTdat == True and y<159 and y>60 and x_360==0 and y_360==0:
        print("reset")
        if Rtulis ==True:
            grk.clear()
            Bt = 0
            # for Rowrst in range(0,rowK):   
            dtSmntr = np.delete(dtSmntr,0,0)
            dtSmntr = np.delete(dtSmntr,1,1)
            dtSmntr = np.delete(dtSmntr,0,1)
            DSegment *= 0
            Rtulis=False
        fTdat=False
p1 = Process(target = terima)
p3 = Process(target = receiveOdroid)
p2 = Thread(target = dataArduino)

#--declare param ANN--#
# jst = JST()
# model = keras.models.load_model("Plan2_20x20_HuangV5.h5")
rCnd=100
pJark=0
if __name__== '__main__':
    p1.start()
    p3.start()
    p2.start()
    while True:
        try:            
            # SX = round(x/16,1)
            # SY = round(y/12,1)
            # hslJST=jst.lakukan(SY,SX)
            # if hslJST == "DIAM":
            #     kendali = "b"
            #     # print("Diam")
            # if hslJST =="KANAN":
            #     kendali="d"
            #     # print("kanan")
            # elif hslJST == "KIRI":
            #     kendali="a"
            # print(kendali)
            # print('\t\t\t\t\t||', encoY)
            kendali = (input("Masukan kendali : "))
            # print(KordX,KordY)
            if kendali is 'p':
                # Setpoint
                set = input("            Masukan Setpoint : ")
                apa = 'p'
                print('setpoint = ', set)
                # ser.write(3, 0, 0, 0, 0, 0)
                # time.sleep(0.05)
                ser.write(ord(apa), set, 0, 0, 0, 0)
                # time.sleep(0.03)
                # ser.write(0, x, y, w, dribble, kick)
                kendali = '0'

            elif kendali is 'l':
                xtujuan = (input('        Masukan X Tujuan: '))
                ytujuan = (input('        Masukan Y Tujuan: '))
                kendali = 'point'

            elif kendali is 'u':
                xStar = int((input('        Masukan X Map: ')))
                yStar = int((input('        Masukan Y Map: ')))
                kendali = 'astar'

            elif kendali is '1':
                # Coba Reset Encoder
                ser.write(1, 5505, 1740)
                # time.sleep(0.05)
                # ser.write(1, 0, 0,0,0,0)
                # time.sleep(0.1)
                # ser.write(4)
                kendali = '0'

            elif kendali is '2':
                # Coba reset Compass
                ser.write(2) 
                # time.sleep(0.05)   
            
            # Maju
            elif kendali is 'w':
                # print("aku disini bro")
                x = -4
                y = 130
                w = 0
                dribble = 1
                apa = 'w'
                # ser.write(5, 0, 0, 0, 0, 0)
                # time.sleep(0.01)
                # ser.write(3, 0, 0, 0, 0, 0)
                # time.sleep(0.01)
                ser.write(0, x, y, w, dribble, kick)

            # Mundur
            elif kendali is 's':
                x = 4
                y = -100
                w = 0
                dribble = 2
                apa = 's'
                # ser.write(5, 0, 0, 0, 0, 0)
                # time.sleep(0.01)
                ser.write(0, x, y, w, dribble, kick)
            
            # Kiri
            elif kendali is 'a':
                x = -80
                y = -6
                w = 0
                dribble = 4
                apa = 'a'
                # ser.write(6, 0, 0, 0, 0, 0)
                # time.sleep(0.01)
                ser.write(0, x, y, w, dribble, kick)
                # ser.write('a', x, y, w, dribble, kick)

            # Kanan
            elif kendali is 'd':
                x = 80
                y = 6
                w = 0
                dribble = 3
                apa = 'd'
                ser.write(6, 0, 0, 0, 0, 0)
                time.sleep(0.01)
                ser.write(0, x, y, w, dribble, kick)

            # Diagonal Kanan Atas
            elif kendali is 'e':
                x = 50
                y = 50
                w = 0
                dribble = 5
                apa = 0
                ser.write(4, 0, 0, 0, 0, 0)
                # time.sleep(0.1)
                ser.write(0, x, y, 0, dribble, kick)

            # Diagonal Kiri Atas
            elif kendali is 'q':
                x = -50
                y = 50
                w = 0
                dribble = 6
                apa = '11'
                ser.write(4, 0, 0, 0, 0, 0)
                # time.sleep(0.1)
                ser.write(0, x, y, w, dribble, kick)
                # ser.write('q', x, y, w, dribble, kick)
            
            # Diagonal Kiri Bawah
            elif kendali is 'z':
                x = -50
                y = -50
                w = 0
                dribble = 6
                apa = '12'
                ser.write(4, 0, 0, 0, 0, 0)
                # time.sleep(0.1)
                ser.write(0, x, y, w, dribble, kick)
                # ser.write(z, x, y, w, dribble, kick)
                
            # Diagonal Kanan Bawah
            elif kendali is 'c':
                x = 50
                y = -50
                w = 0
                dribble = 6
                apa = '14'
                ser.write(4, 0, 0, 0, 0, 0)
                # time.sleep(0.1)
                ser.write(0, x, y, w, dribble, kick)

            elif kendali is 'r':
                x = 0
                y = 0
                w = -15
                dribble = 0
                apa = 0
                ser.write(3, 0, 0, 0, 0, 0)
                # time.sleep(0.1)
                ser.write(0, x, y, w, dribble, kick)

            elif kendali is 't':
                x = 0
                y = 0
                w = 15
                dribble = 0
                apa = 0
                ser.write(3, 0, 0, 0, 0, 0)
                # time.sleep(0.1)
                ser.write(0, x, y, w, dribble, kick)

            elif kendali is 'x':
                ser.write(0, 0, 0, 0, 0, 0)
                # time.sleep(0.01)
                # ser.write(4, 0, 0, 0, dribble, kick)
                # time.sleep(0.05)
                # ser.write(3, 0, 0, 0, dribble, kick)
                
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
            # p2.terminate()
            sys.exit(0)
            
            
            