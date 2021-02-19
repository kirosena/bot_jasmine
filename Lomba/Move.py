import Arduino
import time

class move:
    def __init__(self):
        self.x = self.y = self.w = self.dribble = self.kick = 0
        self.ser = Arduino.Serial('raspi', 'usb')

    def maju(self):
        self.x = -4
        self.y = 110
        self.w = 0
        self.dribble = 1
        self.kick = 0
        self.ser.write(0, self.x, self.y, self.w, self.dribble, self.kick)

    def mundur(self):
        self.x = 4
        self.y = -80
        self.w = 0
        self.dribble = 2
        self.kick = 0
        self.ser.write(0, self.x, self.y, self.w, self.dribble, self.kick)

    def kanan(self):
        self.x = 130
        self.y = 6
        self.w = 0
        self.dribble = 3
        self.kick = 0
        self.ser.write(0, self.x, self.y, self.w, self.dribble, self.kick)

    def kanan_speed(self,speed):
        self.x = speed#130
        self.y = 6
        self.w = 0
        self.dribble = 3
        self.kick = 0
        # print(speed)
        self.ser.write(0, self.x, self.y, self.w, self.dribble, self.kick)

    def kiri_speed(self,speed):
        self.x = speed#-130
        self.y = -6
        self.w = 0
        self.dribble = 4
        self.kick = 0
        # print(speed)
        self.ser.write(0, self.x, self.y, self.w, self.dribble, self.kick)
    
    def kiri(self):
        self.x = -130
        self.y = -6
        self.w = 0
        self.dribble = 4
        self.kick = 0
        self.ser.write(0, self.x, self.y, self.w, self.dribble, self.kick)

    def berhenti(self):
        self.ser.write(0, 0, 0, 0, 0, 0)
        # time.sleep(0.1)
        self.ser.write(4)

    def diagonalKananAtas(self):
        self.x = 50
        self.y = 50
        self.w = 0
        self.dribble = 1
        self.kick = 0
        self.ser.write(0, self.x, self.y, self.w, self.dribble, self.kick)
    
    def diagonalKiriAtas(self):
        self.x = -50
        self.y = 50
        self.w = 0
        self.dribble = 1
        self.kick = 0
        self.ser.write(0, self.x, self.y, self.w, self.dribble, self.kick)

    def diagonalKananBawah(self):
        self.x = 50
        self.y = -50
        self.w = 0
        self.dribble = 2
        self.kick = 0
        self.ser.write(0, self.x, self.y, self.w, self.dribble, self.kick)

    def diagonalKiriBawah(self):
        self.x = -50
        self.y = -50
        self.w = 0
        self.dribble = 2
        self.kick = 0
        self.ser.write(0, self.x, self.y, self.w, self.dribble, self.kick)

    def rotasiKanan(self):
        self.x = 0
        self.y = 0
        self.w = 20
        self.dribble = 0
        self.kick = 0
        self.ser.write(0, self.x, self.y, self.w, self.dribble, self.kick)

    def rotasiKiri(self):
        self.x = 0
        self.y = 0
        self.w = -20
        self.dribble = 0
        self.kick = 0
        self.ser.write(0, self.x, self.y, self.w, self.dribble, self.kick)

    def tendang(self):
        self.x = 0
        self.y = 0
        self.w = 0
        self.dribble = 0
        self.kick = 1
        self.ser.write(0, self.x, self.y, self.w, self.dribble, self.kick)
    
    def chargeKicker(self):
        self.x = 0
        self.y = 0
        self.w = 0
        self.dribble = 0
        self.kick = 0
        self.ser.write(0, self.x, self.y, self.w, self.dribble, self.kick)

    def sudut(self, angle):
        self.ser.write(ord('p'), angle)

    def resetEnco(self, encoX, encoY):
        self.ser.write(1, encoX, encoY)
    
    def resetKompas(self):
        self.ser.write(2)
        time.sleep(0.05)
        self.berhenti()
        