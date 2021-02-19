import MQTT as mqtt
import Arduino
from Move import move
# import A_Star
from multiprocessing import Process, Queue
from threading import Thread
import sys
import time
import math
import re

# astar = A_Star.AStar(500)
# start = (0, 0)
# end = (2, 3)
# path = astar.solve(start, end)
n = dirX = dirY = 0
grid = 500
trig = 0

serArduino = Arduino.Serial('raspi', 'usb')
serOdroid = Arduino.Serial('raspi', 'gpio')
robot = move()

mqtt.set("192.168.0.112", "Jasmine")
queue = Queue()

# Receive Data from Base Station
R0 = robotCode = 0
data1 = data2 = data3 = data4 = data5 = 0

# Variable Process
find1=find2=point1=point2=point3=0

# Receive Data from Arduino
encoX = encoY = angleZ = ball = 0

# Receive Data from Odroid
cond = distB = degB = KrdX = KrdY = 0

# Point to Point
stepP = angleDestination = 1

# Find Object
angleObject = 1

# Multithreading
thread_die = False

#filter_ball
count = 0
b_1 = b_2 = b_3 = b_4 =  b_5=0

#ball_follow
diam = False
diam2 = False
muter_b = False
s_degree1 = 0
kena_360 = False
lock = False

def ball_follow():
    global diam,diam2,muter_b,s_degree1,kena_360,lock
    global ball,angleZ,encoX,encoY
    global KrdX, KrdY,degB, distB, cond
    time.sleep(0.05)
    hasil = fuzzy_weight(KrdX)
    # print(hasil)
    # if KrdY >220:
    #     robot.berhenti()
    #     print('cek')
    if ball == 1 and degB == -36 :
        robot.berhenti()
        # diam =True
        print('bola')
    elif diam == True:
        robot.berhenti()
        print('diam')
        diam = False
        lock = True
        diam2 = True
        # if KrdY <120:
        #     diam =False
    elif (degB >45 or degB <-45) and kena_360 is False:
        robot.berhenti()
        print('kena 360')
        kena_360 = True
        diam = True
    elif hasil < 0 and lock is False:
        robot.kiri_speed(hasil)
        # print('kiri')
    elif hasil > 0 and lock is False:
        robot.kanan_speed(hasil)
        # print('kanan')
    elif hasil == 0 and lock is False:
        robot.berhenti()
        # print('henti')

    elif diam2 is True and ball is 0:
        if degB != -36:
            # apa = 'p'
            s_degree1 = (degB + angleZ)
            robot.sudut(s_degree1)
            # serArd.write(ord(apa), s_degree, 0, 0, 0, 0)
            # time.sleep(3)
            muter_b = True
            diam2 = False
            print('muter bro')
        else:
            robot.mundur()
            time.sleep(1)
            robot.berhenti()
            time.sleep(2)
            print('ga nemu mundur dulu')
    elif muter_b is True:
        #print('s_degree', s_degree)
        if angleZ >= (s_degree1-5) and angleZ <= (s_degree1+5):
            # time.sleep(1)
            robot.maju()
            time.sleep(2)
            print('maju abis muter')
            robot.berhenti()
            time.sleep(1)
            # muter = False
            # s_2 = False
            # print('finish')
            if ball is 1:
                robot.berhenti()
                time.sleep(1)
                muter_b = False
                diam2 = False
                # print('finish find Ball at ', encoX, ', ', encoY)
                # return 'done'
            elif ball is 0:
                robot.berhenti()
                time.sleep(2)
                diam2 = True
                print('ulang lagi muter')


#puzzy_for_ball_follow
def fuzzy_weight(INPUT):
    fuzz = [0]*5
    A1 = 10  #[0,10,40,100,160]
    A2 = 30
    A3 = 80
    A4 = 160
    O1 = 30  #[0,30,70,100,130]
    O2 = 70
    O3 = 100
    O4 = 130
    if(INPUT > 0.0):

        rgx = [0, A1, A2, A3, A4] #close
        # rgx = [0, 30, 70, 120, 160] #[0, 30, 70, 120, 160]slow but sure
        if(INPUT <= rgx[0]):
            fuzz[0] = 1
        elif(INPUT > rgx[0] and INPUT  <= rgx[1]):
            fuzz[0] = (rgx[1] - INPUT)/(rgx[1] - rgx[0])
        else:
            fuzz[0] = 0

        if (INPUT <= rgx[0]):
            fuzz[1] = 0
        elif (INPUT > rgx[0] and INPUT <= rgx[1]):
            fuzz[1] = (INPUT-rgx[0])/(rgx[1]-rgx[0])
        elif (INPUT > rgx[1] and INPUT <= rgx[2]):
            fuzz[1] = (rgx[2]-INPUT)/(rgx[2] - rgx[1])
        else:
            fuzz[1] = 0

        if (INPUT <= rgx[1]):
            fuzz[2] = 0
        elif (INPUT > rgx[1] and INPUT <= rgx[2]):
            fuzz[2] = (INPUT-rgx[1])/(rgx[2]-rgx[1])
        elif (INPUT > rgx[2] and INPUT <= rgx[3]):
            fuzz[2] = (rgx[3]-INPUT)/(rgx[3] - rgx[2])
        else:
            fuzz[2] = 0

        if (INPUT <= rgx[2]):
            fuzz[3] = 0
        elif (INPUT > rgx[2] and INPUT <= rgx[3]):
            fuzz[3] = (INPUT - rgx[2])/(rgx[3]-rgx[2])
        elif (INPUT > rgx[3] and INPUT <= rgx[4]):
            fuzz[3] = (rgx[4]-INPUT)/(rgx[4] - rgx[3])
        else:
            fuzz[3] = 0
        
        #sangat jauh
        if (INPUT <= rgx[3]):
            fuzz[4] = 0
        elif (INPUT > rgx[3] and INPUT <= rgx[4]):
            fuzz[4] = (INPUT-rgx[3])/(rgx[4]-rgx[3])
        else:
            fuzz[4] = 1

        output_rule = [0,O1,O2,O3,O4] 
        #output_rule = [0,20,40,80,120]

        sigma = 0
        den = 0
        for i in range(0, 5):
            sigma = sigma + fuzz[i]*output_rule[i]
            den = den + fuzz[i]
            
        return sigma/den

    if(INPUT <= 0.0):
        rgx = [0, -A1, -A2, -A3, -A4] #[0, 10, 40, 100, 160]

        if(INPUT >= rgx[0]):
            fuzz[0] = 1
        elif(INPUT < rgx[0] and INPUT  >= rgx[1]):
            fuzz[0] = (rgx[1] - INPUT)/(rgx[1] - rgx[0])
        else:
            fuzz[0] = 0

        if (INPUT >= rgx[0]):
            fuzz[1] = 0
        elif (INPUT < 0 and INPUT >= rgx[1]):
            fuzz[1] = (INPUT-rgx[0])/(rgx[1]-rgx[0])
        elif (INPUT < rgx[1] and INPUT >= rgx[2]):
            fuzz[1] = (rgx[2]-INPUT)/(rgx[2] - rgx[1])
        else:
            fuzz[1] = 0

        if (INPUT >= rgx[1]):
            fuzz[2] = 0
        elif (INPUT < rgx[1] and INPUT >= rgx[2]):
            fuzz[2] = (INPUT-rgx[1])/(rgx[2]-rgx[1])
        elif (INPUT < rgx[2] and INPUT >= rgx[3]):
            fuzz[2] = (rgx[3]-INPUT)/(rgx[3] - rgx[2])
        else:
            fuzz[2] = 0

        if (INPUT >= rgx[2]):
            fuzz[3] = 0
        elif (INPUT < rgx[2] and INPUT >= rgx[3]):
            fuzz[3] = (INPUT - rgx[2])/(rgx[3]-rgx[2])
        elif (INPUT < rgx[3] and INPUT >= rgx[4]):
            fuzz[3] = (rgx[4]-INPUT)/(rgx[4] - rgx[3])
        else:
            fuzz[3] = 0
        
        #sangat jauh
        if (INPUT >= rgx[3]):
            fuzz[4] = 0
        elif (INPUT < rgx[3] and INPUT >= rgx[4]):
            fuzz[4] = (INPUT-rgx[3])/(rgx[4]-rgx[3])
        else:
            fuzz[4] = 1

        output_rule = [0,-O1,-O2,-O3,-O4] #[0,30,70,100,130]
        #output_rule = [0,-20,-40,-80,-120]

        sigma = 0
        den = 0
        for i in range(0, 5):
            sigma = sigma + fuzz[i]*output_rule[i]
            den = den + fuzz[i]
            
        return sigma/den

# ===================== Receive All of Data to Main Thread Multithrading =======================
def receiveData():  
    global R0, robotCode, data1, data2, data3, data4, data5
    global thread_die
    global encoX, encoY, angleZ, ball
    global cond, distB, degB, KrdX, KrdY
    global count, b_1, b_2, b_3, b_4,b_5
    FPrint1=0
    FPrint2=0
    while True:
        # print("ditrima")
        if not queue.empty():
            packet = queue.get()
            # time.sleep(0.01)
            # print(packet)

            if packet[0] == 'A':            # terima data dari Arduino
                encoX = round(int(packet[1])/10)
                encoY = round(int(packet[2])/10)
                angleZ = int(packet[3])
                c_ball = int(packet[4])
                if count > 5:
                    count = 1
                else:
                    count += 1
                if count == 1:
                    b_1 = c_ball
                elif count == 2:
                    b_2 = c_ball
                elif count == 3:
                    b_3 = c_ball
                elif count == 4:
                    b_4 = c_ball
                elif count == 5:
                    b_5 = c_ball
                
                ball_filter = ((b_1 + b_2 + b_3 + b_4 + b_5)/5)
                # print(ball_filter)
                if ball_filter >0:
                    ball = 1
                else:
                    ball = 0
                # print('packet = ',packet)
                # mqtt.send(0,encoX, encoY, angleZ, ball)

            elif packet[0] == 'O':          # terima data dari Odroid
                cond = int(packet[1])
                distB= int(packet[2])
                degB= int(packet[3])
                c_KrdX= int(packet[4])
                KrdY= int(packet[5])
                KrdX = (c_KrdX-160)
                # print(degB)
                # if KrdX == -160:
                #     KrdX = 0
                # print(KrdX)
                # mqtt.send(1,distB, degB, enemyDistance, enemyDegree)

        R0 = mqtt.read("Base")          # terima data dari Base Station
        if R0:
            # print(R0)
            robotCode = R0[0]
            data1 = (R0[1])
            data2 = (R0[2])
            data3 = (R0[3])
            data4 = (R0[4])
            data5 = (R0[5])
            # print(robotCode,'\t', xRobot,'\t', yRobot,'\t', zRobot,'\t', speed)
        
        if thread_die is True:
            print('thread diee..')
            break
        # Jrk = (encoX,encoY)
        if FPrint1!=encoX and FPrint2!=encoY:
        # print(robotCode,' ', data1, ' ', data2,' ', data3,' ', data4)
            print(encoX, ' ', encoY,' ', angleZ,' ', ball)
            (FPrint1,FPrint2)=(encoX,encoY)
            # print(robotCode,' ', data1, ' ', data2,' ', data3,' ', data4)
            # (FPrint1,FPrint2)=(robotCode,data2)
        # print(encoX,' ', encoY,' ', angleZ,' ', ball, ' ', distB, ' ',degB)
        # print(encoX,' ', encoY,' ', angleZ,' ', ball,' ',distB, ' ',degB,' ', KrdX,' ',KrdY)
        # print(cond,' ', distB, ' ',degB,' ', KrdX,' ',KrdY)

# ================== Receive Serial Multiprocessing =====================
def receiveArduino():
    fData = ''
    while True:
        recv = serArduino.readArduino()
        if fData != recv:
            data = list(recv)
            data.insert(0, 'A')
            queue.put(data)
            # print(data)
            fData = recv

def receiveOdroid():
    fData = ''
    while True:
        recv = serOdroid.readOdroid()
        if fData != recv:
            data = list(recv)
            data.insert(0, 'O')
            queue.put(data)
            fData = recv

# ========================== Function =====================
def pointToPoint(xF, yF):
    global encoX, encoY, angleZ, degB
    global stepP, angleDestination

    kuadran = a = b = 0
    if xF == '-': xF = encoX 
    else: xF = int(xF)
    if yF == '-': yF = encoY 
    else: yF = int(yF) + 1
    
    if stepP == 1:
        if encoX >= (xF - 10) and encoX <= (xF + 10) and encoY >= (yF - 10) and encoY <= (yF + 10):
            pass
        else:
            print('to ', xF, ' ', yF)
            if encoX == 0: encox = 1
            if encoY == 0: encoY = 1
            angleDestination = round(abs(math.degrees(math.atan((xF-encoX)/(yF-encoY)))))
            print(f'sudut awal = {angleDestination}')
            stepP = 2

    elif stepP == 2:
        # print("Aku di 2")
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

        if angleDestination >= 350:
            angleDestination = angleDestination - 360
        elif angleDestination <= -350:
            angleDestination = angleDestination + 360

        print(f'angle = {angleDestination}')
        robot.sudut(angleDestination)
        stepP = 3

    elif stepP == 3:
        # time.sleep(1)
        if angleZ >= (angleDestination-10) and angleZ <= (angleDestination+10):
            # if angleZ == angleDestination:
        # print("Aku di 3")
        # time.sleep(0.5)
            # serArduino.write(5)
            time.sleep(1)
            stepP = 4
        # kendali = 0
        # return 'done'
    
    elif stepP == 4:
        # if (math.hypot(encoX, encoY)) <= (math.hypot(xF, yF) - 100) or (math.hypot(encoX, encoY)) >= (math.hypot(xF, yF) + 10):
        if (encoY >= (yF - 30) and encoY <= (yF + 30)) and (encoX >= (xF - 30) and encoX <= (xF + 30)):
            robot.berhenti()
            stepP = 1
            print('Point to Point done at | ', encoX, ' ', encoY)
            return 'done'
        else:
            robot.maju() 
            # print('majuuu')


def findingObject(object, xF=0, yF=0):
    global degB, distB
    global encoX, encoY, angleZ
    global stepP, angleObject

    if object == "friend":
        kuadran = a = b = 0
        if xF == '-': xF = encoX 
        else: xF = int(xF)
        if yF == '-': yF = encoY 
        else: yF = int(yF) + 1
        
        if stepP == 1:
            print(xF, ' ', yF, ' | ', encoX, ' ', encoY)
            if encoX == 0: encox = 1
            if encoY == 0: encoY = 1
            angleObject = round(abs(math.degrees(math.atan((xF-encoX)/(yF-encoY)))))
            print(f'sudut awal = {angleObject}')
            stepP = 2

        elif stepP == 2:
            # print("Aku di 2")
            # Menentukan sudut tujuan
            if xF > encoX and yF > encoY: 
                kuadran = 1
                print('kuadran 1')
                a = 0 + angleObject
                b = -360 + angleObject
            elif xF < encoX and yF > encoY:
                kuadran = 2
                print('kuadran 2')
                a = 0 - angleObject
                b = 360 - angleObject
            elif xF < encoX and yF < encoY:
                kuadran = 3
                print('kuadran 3')
                a = 180 + angleObject
                b = -180 + angleObject
            elif xF > encoX and yF < encoY:
                kuadran = 4
                print('kuadran 4')
                a = 180 - angleObject
                b = -180 - angleObject
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
                angleObject = a
                # print('angleObject = ', angleObject)
            else:
                angleObject = b
                # print('angleObject = ', angleObject)

            # if angleObject >= 350:
            #     angleObject = angleObject - 360
            # elif angleObject <= -350:
            #     angleObject = angleObject + 360

            time.sleep(1)
            print(f'angle = {angleObject},  {encoX},  {encoY}')
            robot.sudut(angleObject)
            stepP = 3

        elif stepP == 3:
            # time.sleep(1)
            if angleZ >= (angleObject-3) and angleZ <= (angleObject+3):
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
            angleObject = round(abs(math.degrees(math.atan((xF-encoX)/(yF-encoY)))))
            # print(f'sudut awal = {angleDestination}')
            stepP = 2

        elif stepP == 2:
            # print("Aku di 2")
            # Menentukan sudut tujuan
            if xF > encoX and yF > encoY: 
                kuadran = 1
                a = 0 + angleObject
                b = -360 + angleObject
            elif xF < encoX and yF > encoY:
                kuadran = 2
                a = 0 - angleObject
                b = 360 - angleObject
            elif xF < encoX and yF < encoY:
                kuadran = 3
                a = 180 + angleObject
                b = -180 + angleObject
            elif xF > encoX and yF < encoY:
                kuadran = 4
                a = 180 - angleObject
                b = -180 - angleObject
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
                angleObject = a
            else:
                angleObject = b
            
            if angleObject >= 350:
                angleObject = angleObject - 360
            elif angleObject <= -350:
                angleObject = angleObject + 360

            # print(f'angle = {angleDestination}')
            time.sleep(1)
            robot.sudut(angleObject)
            time.sleep(0.05)
            stepP = 3

        elif stepP == 3:
            # time.sleep(1)
            if angleZ >= (angleObject-3) and angleZ <= (angleObject+3):
                # if angleZ == angleDestination:
            # print("Aku di 3")
            # time.sleep(0.5)
                # serArduino.write(5)
                stepP = 1
            # kendali = 0
                return 'done'


# ========================= Main Program ====================
def main():
    global robotCode, data1, data2, data3, data4, data5
    global thread_die, stepP
    global find1, find2, point1, point2, point3
    global encoX, encoY, angleZ, ball
    global cond, distB, degB, KrdX, KrdY
    # robotCode = 'A'
    
    if degB == 0:
        degB = -36
    
    fprint=0
    while True:
        try:
            if robotCode == 'A':
                # print(encoX, ' ', encoY)
                if point1 != 'done':
                    point1 = pointToPoint(520, 30)
                if point1 == 'done' and find1 != 'done':
                    # print('sipp sipp sipp')
                    find1 = findingObject('friend', 300, 20)
                    time.sleep(0.05)
                    robot.chargeKicker()
                if find1 == 'done' and point2 != 'done' and ball == 1:
                    # print('hehe')
                    point2 = pointToPoint(520, 370)
                if point2 == 'done' and point3 != 'done':
                    point3 = pointToPoint(365, 350)
                if point3 == 'done' and ball == 1:
                    find2 = findingObject('friend', 220, 350)
                    print('nyari temen')
                if find2 == 'done' and ball == 1:
                    robot.tendang()
                

    # while True:
    #     try:
    #         if robotCode == '3':
    #             if data1 == 'step':
    #                 if data2 == '1':
    #                     if point1 != 'done':
    #                         point1 = pointToPoint(520, 20)
    #                     if point1 == 'done':
    #                         mqtt.send('recvPos', 1)
    #                 elif data2 == '2':
    #                     if find1 != 'done':
    #                         print('terima bola dari teman di ', data3, ' ', data4)
    #                         find1 = findObject('friend', data3, data4)
    #                     elif find1 == 'done':
    #                         mqtt.send('friend', 1)
    #                 elif data2 == '3':
    #                      # ball_follow()
    #                     if ball == 1:
    #                         mqtt.send('recvBall', 1)
    #                 elif data2 == '4':
    #                     if point2 != 'done':
    #                         point2 = pointToPoint(520, 370)
    #                     elif point2 == 'done' and point3 != 'done':
    #                         point3 = pointToPoint(365, 350)
    #                     if point3 == 'done':
    #                         mqtt.send('passPos', 1)
    #                 elif data2 == '5':
    #                     print('oper ke teman di ', data3, ' ', data4)
    #                     if find2 != 'done':
    #                         find2 = findObject('friend', data2, data3)
    #                     if find2 == 'done':
    #                         mqtt.send('friend', 1)
    #                 elif data2 == '6':
    #                     robot.tendang()
    #                     mqtt.send('passBall', 1)
    #                 elif data2 == '7':
    #                     robot.berhenti()
                
                    
    #         if robotCode == '3M':
    #             # (0, xSpeed, ySpeed, zSpeed, dribble, kick)
    #             dribble = 0
    #             if data1 > 0 and data2==0 : dribble = 1
    #             elif data1 < 0 and data2==0: dribble = 2
    #             elif data2 > 0 and data1==0: dribble = 3
    #             elif data2 < 0 and data1==0: dribble = 4
    #             else: dribble = 0
    #             serArduino.write(0, data1, data2, data3, dribble, data4)
           
        except KeyboardInterrupt:
            time.sleep(1)
            mqtt.send(2, 0)
            thread_die = True
            p1.join()
            # p2.setDaemon(True)
            print("Exit nih gua")
            p2.terminate()
            p3.terminate()
            # p2.terminate()
            sys.exit(0)

# Multithreading for MQTT
p1 = Thread(target = receiveData)
p1.setDaemon(True)
p1.start()

# Multiprocessing for Serial
p2 = Process(target = receiveArduino)
p3 = Process(target = receiveOdroid)

# time.sleep(3)

if __name__== '__main__':
    p2.start()
    p3.start()
    time.sleep(2)
    robot.resetKompas()
    time.sleep(1)
    robot.resetEnco(5430, 1740)
    time.sleep(2)
    print('reset Encoder and Compass = ', encoX, ' ', encoY, ' ', angleZ)
    robot.sudut(-90)
    time.sleep(3)
    main()
    # try:
    #     while True:
    #         ball_follow()
            
            

    # except KeyboardInterrupt:
    #     # mqtt.send(2, 0)
    #     thread_die = True
    #     p1.join()
    #     # p2.setDaemon(True)
    #     print("Exit nih gua")
    #     p2.terminate()
    #     p3.terminate()
    #     # p2.terminate()
    #     sys.exit(0)
    

