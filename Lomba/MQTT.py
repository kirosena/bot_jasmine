import paho.mqtt.client as mqtt
import time
import re

text = ''
client = None
name = ''
flag = 0

def connect(client, userdata, flags, rc):
    if rc == 0:
        client.connected_flag = True
        print("Sambungan OK Bang")
    else:
        print("Koneksi Jelek Bang, ini Kodenya = ", rc)
        client.loop_stop()

def message(client, userdata, message):
    global text
    text = str(message.payload.decode("utf-8")) 

def set(broker, nama):
    global client, name
    name = nama
    mqtt.Client.connected_flag = False
    client = mqtt.Client(name)
    client.on_connect = connect
    client.on_message = message
    client.connect(broker, 1883)
    # client.loop_start()
    # while not client.connected_flag: #wait in loop
    #     print("In wait loop")
    #     time.sleep(0.5)

def read(topic):
    global text, client,flag
    g1=g2=g3=g4=g5=g6=0
    # if flag == 0:
    try:
        client.loop_start()
        client.subscribe(topic, 2)
        # print(text)
        time.sleep(0.01)
        # print(text)
        if text: 
            prs = re.compile(r'(\S*)@(\S*)#(\S*)%(\S*)&(\S*),(\S*)')
            dataParsing = re.search(prs, str(text))
            if dataParsing:
                g1 = dataParsing.group(1)   
                g2 = dataParsing.group(2)   
                g3 = dataParsing.group(3)   
                g4 = dataParsing.group(4)  
                g5 = dataParsing.group(5)                
                g6 = dataParsing.group(6)
            else:
                g1 = 0
                g2 = 0
                g3 = 0
                g4 = 0
                g5 = 0
                g6 = 0
            time.sleep(0.02)
            return g1,g2,g3,g4,g5,g6
        
    except KeyboardInterrupt:
        client.loop_stop()


def send(kode, data1=0, data2=0, data3=0, data4=0, data5=0 ):
    global client, name, flag
    flag = 1
    # kode = 0  =>  data1 = xRobot,     data2 = yRobot,  data3 = zRobot,  data4 = kick
    # kode = 1  =>  data1 = xBola,      data2 = yBola
    # kode = 2  =>  data1 = kondisi Bola
    # kode = 3  =>  data1 = xLawan,     data2 = yLawan
    
    # if flag == 1:
    client.loop_start()
    packet = str(kode)+'@'+str(data1)+'#'+str(data2)+'%'+str(data3)+'&'+str(data4)+','+str(data5)
    client.publish(name, packet, 2, True)
    # print(name)
    time.sleep(0.01)
    flag = 0
    # else:
    #     client.loop_stop()
    



