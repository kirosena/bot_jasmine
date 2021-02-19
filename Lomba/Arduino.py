import serial
import time
import re
import sys


class Serial:
    def __init__(self,pc, media='gpio'):
        self.pc = pc
        self.media = media
        self.port = ''
        self.usb = 0
        self.con = self.Connection()
        self.q = 0
        self.g1=self.g2=self.g3=self.g4=self.g5=0
        self.g6=self.g7=0
        self.a1=self.a2=self.a3=self.a4=self.a5=0
        self.fkode = 0
        self.fx = self.fy = self.fz = 1
        self.fdribble = self.fkick = self.fsetpoint = 0
        self.fxEnco = self.fyEnco = 1
        self.p1 = self.p2 = self.p3 = self.p4 = self.p5 = self.p6 = 0

    def Connection(self):
        if self.pc is 'pc':
            self.port = 'COM'
        elif self.pc is 'raspi':
            if self.media is "usb":
                self.port = "/dev/ttyUSB"
            elif self.media is "gpio":
                self.port = "/dev/serial"
            else:
                print ("NO Connection")
                sys.exit(0)
        else :
            print ("NO Connection")
            sys.exit(0)

        print("Connecting...")
        print("port is ",self.port)
        for i in range(40):
            try:
                conName = self.port + str(self.usb)
                ser = serial.Serial(conName, 57600, timeout=1)
                ser.flush()
                if ser:
                    print(conName)
                return ser
                break
            except:
                self.usb += 1 
        if self.usb == 40:
            print("No Connection")
            sys.exit(0)

    def readAll(self):
        try:
            recv = self.con.readline().decode('utf-8').rstrip()
            # print(recv)
            if len(recv) > 0:
                return (recv)
        except KeyboardInterrupt:
            print("System Exit")
            sys.exit(0)
        except:
            pass
    
    def readOdroid(self):
        try:
            data = self.con.readline().decode('utf-8').rstrip()
            if data:
                # print(data)
                prs = re.compile(r'!(\S*)@(\S*)#(\S*)%(\S*)&(\S*)')
                dataParsing = re.search(prs, str(data))
                if dataParsing:
                    self.g1 = dataParsing.group(1)
                    self.g2 = dataParsing.group(2)
                    self.g3 = dataParsing.group(3)            
                    self.g4 = dataParsing.group(4)
                    self.g5 = dataParsing.group(5)

            return self.g1,self.g2,self.g3,self.g4, self.g5
        except KeyboardInterrupt:
            print("System Exit")
            sys.exit(0)
        except:
            pass
            return self.g1,self.g2,self.g3,self.g4, self.g5
    def readArduino(self):
        try:
            data = self.con.readline().decode('utf-8').rstrip()
            if data:
                # print(data)
                prs = re.compile(r'!(\S*)@(\S*)#(\S*)%(\S*)')
                dataParsing = re.search(prs, str(data))
                if dataParsing:
                    self.g1 = dataParsing.group(1)
                    self.g2 = dataParsing.group(2)
                    self.g3 = dataParsing.group(3)            
                    self.g4 = dataParsing.group(4)

            return self.g1,self.g2,self.g3,self.g4
        except KeyboardInterrupt:
            print("System Exit")
            sys.exit(0)
        except:
            pass
            return self.g1,self.g2,self.g3,self.g4
    
    def parsingArduino(self, data):
        if data:
            # print(data)
            prs = re.compile(r'!(\S*)@(\S*)#(\S*)%(\S*)')
            dataParsing = re.search(prs, str(data))
            if dataParsing:
                self.a1 = dataParsing.group(1)
                self.a2 = dataParsing.group(2)
                self.a3 = dataParsing.group(3)            
                self.a4 = dataParsing.group(4)
                
        return self.a1,self.a2,self.a3,self.a4

    def parsingOdroid(self, data):
        if data:
        # print(data)
            prs = re.compile(r'!(\S*)@(\S*)#(\S*)%(\S*)&(\S*)')
            dataParsing = re.search(prs, str(data))
            if dataParsing:
                self.a1 = dataParsing.group(1)
                self.a2 = dataParsing.group(2)
                self.a3 = dataParsing.group(3)            
                self.a4 = dataParsing.group(4)
                self.a5 = dataParsing.group(5)
            
        return self.a1,self.a2,self.a3,self.a4,self.a5

    def write(self, kode, p1=0, p2=0, p3=0, p4=0, p5=0 ): #arduino
        try:
            if kode != self.fkode or p1 != self.fx or p2 != self.fy or p3 != self.fz or p4 != self.fdribble or p5 != self.fkick:
                packet = str(kode)+'!'+str(p1)+'@'+str(p2)+'#'+str(p3)+'$'+str(p4)+'%'+str(p5)+'^'+'\n'
                # print(packet)
                self.con.write(packet.encode())
                self.fkode = kode
                self.fx = p1
                self.fy = p2
                self.fz = p3
                self.fdribble = p4
                self.fkick = p5
                    # self.p6 = p6

            # if kode == 1 or kode == 2 or kode == 3 or kode == 4 or kode == 5 or kode == 6:
            #     if kode != self.fkode:
            #         packet = str(kode)+'!'+str(p1)+'@'+str(p2)+'#'+str(0)+'$'+str(0)+'%'+str(0)+'^'+'\n'
            #         # print(packet)
            #         self.con.write(packet.encode())
            #         self.fkode = kode
            # elif kode == 0 or kode==9:
            #     x = p1
            #     y = p2
            #     z = p3
            #     dribble = p4
            #     kick = p5
            #     if x != self.fx or y != self.fy or z != self.fz or dribble != self.fdribble or kick != self.fkick:
            #         packet = str(kode)+'!'+str(x)+'@'+str(y)+'#'+str(z)+'$'+str(dribble)+'%'+str(kick)+'^'+'\n'
            #         # print(packet)
            #         self.con.write(packet.encode())
            #         self.fx = x
            #         self.fy = y
            #         self.fz = z
            #         self.fdribble = dribble
            #         self.fkick = kick

            # elif kode == 112:
            #     setpoint = p1
            #     if setpoint != self.fsetpoint:
            #         packet = str(kode)+'!'+str(setpoint)+'@'+str(0)+'#'+str(0)+'$'+str(0)+'%'+str(0)+'^'+'\n'
            #         # print(packet)
            #         self.con.write(packet.encode())
            #         self.fsetpoint = setpoint

        except KeyboardInterrupt:
            print("System Exit")
            sys.exit(0)
        except:
            print('error sending message')
            pass

    def writeOdroid(self, p1=0, p2=0, p3=0, p4=0):
        try:
            packet ='!'+str(p1)+'@'+str(p2)+'#'+str(p3)+'%'+str(p4)
            self.con.write(packet.encode())
        except KeyboardInterrupt:
            print("System Exit")
            sys.exit(0)
        except:
            print('error sending message')
            pass
    
