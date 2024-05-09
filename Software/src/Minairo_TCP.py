'''
    __  __ _             _        _    __   ___
   |  \/  (_)           (_)      //   /_ | / _ \
   | \  / |_ _ __   __ _ _ _ __ ___    | || | | |
   | |\/| | | '_ \ / _` | | '__/ _ \   | || | | |
   | |  | | | | | | (_| | | | | (_) |  | || |_| |
   |_|  |_|_|_| |_|\__,_|_|_|  \___/   |_(_)___/

   Descripción: Esta libreria permite controlar un Robot MINAIRÓ 1.0 mediante distintos métodos
    implementados en la Clase "MinairoSocket" para Python3 .
    En la versión 3 se transmiten todos los paràmetros en una sala instrucción en RAW.

   Versión: 3.0
   Autor: Daniel Flores Elias
   mail: daniel.flores@gmail.com

   Metodos:
        MinairoSocket(IP,port)
        run():
        transmit():
        stop():
        close():
        setPullingTime(x):
        getPullingTime():
        setVel(x,y,w):
        setX(x):
        setY(y):
        setW(w):
        setSensorLine_Threshold(x):
        getSensorLine_Threshold():
        getSensorLine_Analog():
        getSensorLine_Digital():
        getSensorSharp():
        getAnalogs():
        confGPIO(pin,mode):
        setGPIO(pin,value):
        getGPIO():
        setSERVO(pin,value):
        getSONAR():
_______________________________________________________________________________________


   Copyright (c) 2023 Institut Jaume Huguet-Valls, Daniel Flores
   <daniel.flores@gmail.com>  All rights reserved.
   See the bottom of this file for the license terms.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are
   met:

   1. Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.

   2. Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.

   3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
   HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
'''

from socket import *
from threading import Thread, Timer
from struct import pack,unpack
from ctypes import *
import time
from tkinter import *
from math import tan,pi,cos,sin,pow

class MinairoSocket():
    """MinairoSocket() - Requiere IP del MINAIRÓ i numero de puerto"""
    def __init__(self,host,port):
        self.host = host
        self.port = port
        self.obj = socket()
        self.command = ""
        self.InByte = ''
        self.InString = ""
        self.OutBuffer = bytearray(32)
        self.InBuffer = bytearray(44)

        self.pullingTime = 0.025  #Tiempo de ciclo para adquisición de datos con el Robot
        self.thread_runs = True
        self.X = 0.0    # Velocidad en m/s
        self.Y = 0.0    # Velocidad en m/s
        self.W = 0.0    # Velocidad en rad/s
        self.SensorLine_Analog = [0,0,0,0,0,0,0,0]
        self.SensorLine_Digital = [False,False,False,False,False,False,False,False]
        self.SensorLine_Threshold = 2400
        self.SensorSharp = [0,0,0,0]
        self.CtrlWord1 = 0
        self.CtrlWord2 = 0
        self.StatusWord1 = 0
        self.StatusWord2 = 0
        self.EncoderValue = 0
        self.SetPoint = 0
        self.AuxMotor = {"SP": 0, "PV": 0, "Home": 0, "H_SW": 0, "L_SW": 0}
        self.Motor_set_Transmit = False
        self.GPIO_conf = [0,0,0,0,0,0,0]
        self.GPIO_value = [0,0,0,0,0,0,0]
        self.GPIO_status = [0,0,0,0,0,0,0]
        self.GPIO_conf_Transmit = False
        self.GPIO_set_Transmit = False
        self.Analogicas = [0,0,0,0]
        self.Sonar = 0
        self.Servos = [1500,1500,1500,1500,1500]
        self.Servo_set_Transmit = False
        self.setOutBuffer()
        self.TransmitCounter = 0
        self.T1 = round(time.time()*1000)
        self.T2 = round(time.time()*1000)
        print (f"Minairo 1.0 en host:{self.host} creat")

    def setOutBuffer(self):
        Vx = pack('f', self.X)
        Vy = pack('f', self.Y)
        Vw = pack('f', self.W)
        CtrlW1 = pack('H', self.CtrlWord1)
        CtrlW2 = pack('H', self.CtrlWord2)
        SP = pack('I', self.SetPoint)
        GPIO_C = 0
        for k in range(0,7):
            if self.GPIO_conf[k]==1:
                GPIO_C = GPIO_C | 1<<k
            else:
                GPIO_C = GPIO_C & ~(1<<k)
        GPIO_C = pack('b',GPIO_C)
        GPIO_V = 0
        for k in range(0,7):
            if self.GPIO_value[k]==1:
                GPIO_V = GPIO_V | 1<<k
            else:
                GPIO_V = GPIO_V & ~(1<<k)
        GPIO_V = pack('b',GPIO_V)
        Srv0 = pack('H', self.Servos[0])
        Srv1 = pack('H', self.Servos[1])
        Srv2 = pack('H', self.Servos[2])
        Srv3 = pack('H', self.Servos[3])
        Srv4 = pack('H', self.Servos[4])
        self.OutBuffer = Vx + Vy + Vw + CtrlW1 + CtrlW2 + SP + GPIO_C + GPIO_V + Srv0 + Srv1 + Srv2 + Srv3 + Srv4

    def getInBuffer(self):
        [self.StatusWord1] = unpack('H', self.InBuffer[0:2])
        [self.StatusWord2] = unpack('H', self.InBuffer[2:4])
        [self.EncoderValue] = unpack('i', self.InBuffer[4:8])
        [LS_0] = unpack('H', self.InBuffer[8:10])
        [LS_1] = unpack('H', self.InBuffer[10:12])
        [LS_2] = unpack('H', self.InBuffer[12:14])
        [LS_3] = unpack('H', self.InBuffer[14:16])
        [LS_4] = unpack('H', self.InBuffer[16:18])
        [LS_5] = unpack('H', self.InBuffer[18:20])
        [LS_6] = unpack('H', self.InBuffer[20:22])
        [LS_7] = unpack('H', self.InBuffer[22:24])
        [SS_0] = unpack('H', self.InBuffer[24:26])
        [SS_1] = unpack('H', self.InBuffer[26:28])
        [SS_2] = unpack('H', self.InBuffer[28:30])
        [SS_3] = unpack('H', self.InBuffer[30:32])
        [GPIO_Status] = unpack('B', self.InBuffer[32:33])
        [AN_0] = unpack('H', self.InBuffer[34:36])
        [AN_1] = unpack('H', self.InBuffer[36:38])
        [AN_2] = unpack('H', self.InBuffer[38:40])
        [AN_3] = unpack('H', self.InBuffer[40:42])
        [self.Sonar] = unpack('H', self.InBuffer[42:44])
        self.SensorLine_Analog  = [LS_0,LS_1,LS_2,LS_3,LS_4,LS_5,LS_6,LS_7]
        self.SensorSharp = [SS_0,SS_1,SS_2,SS_3]
        self.Analogicas = [AN_0,AN_1,AN_2,AN_3]
        for k in range(0,7):
            if GPIO_Status & 1<<k:
                self.GPIO_status[k]= 1
            else:
                self.GPIO_status[k]= 0

    def connect(self):
        self.obj.connect((self.host,self.port))

 
    def buidarSocket(self):
        while 1:
            s = self.obj.recv(1).decode()
            if s == '\n':
                break;

    def sendCommand(self, data):
        self.command = data + '\n'
        self.obj.send( self.command.encode())

    def read(self):
        self.InByte = ''
        self.InString = ""
        while self.InByte != '\n':
            self.InString += self.InByte
            self.InByte = self.obj.recv(1).decode()
        return self.InString

    def close(self):
        self.obj.close()
        print (f"Minairo 1.0 en host:{self.host} desconectado")

    def stop(self):
        self.X = 0.0
        self.Y = 0.0
        self.W = 0.0
        self.setOutBuffer()
        self.thread_runs = False

    def run(self):
        self.thread_runs = True
        self.connect()
        self.transmitCyclic()
    
    def transmitCyclic(self):
        if self.thread_runs:
            self.transmit()
            Timer(self.pullingTime, self.transmitCyclic).start()
        else:
            self.transmit()
            self.obj.close()

    def transmit(self):
        ################# c m d _ R A W #################
        self.obj.send(self.OutBuffer)
        self.InBuffer = self.obj.recv(44)
        self.getInBuffer()

    def setVel(self,x,y,w):
        self.X = x
        self.Y = y
        self.W = w
        self.setOutBuffer()

    def setX(self,x):
        self.X = x
        self.setOutBuffer()
    
    def getX(self):
        return self.X
    
 
    def setY(self,y):
        self.Y = y
        self.setOutBuffer()

    def getY(self):
        return self.Y

    def setW(self,w):
        self.W = w
        self.setOutBuffer()

    def getW(self):
        return self.W

    def setSensorLine_Threshold(self,x):
        self.SensorLine_Threshold = x

    def getSensorLine_Threshold(self):
        return self.SensorLine_Threshold

    def getSensorLine_Analog(self):
        return self.SensorLine_Analog

    def getSensorLine_Digital(self):
        for x in range(0,8):
            if self.SensorLine_Analog[x]>=self.SensorLine_Threshold:
                self.SensorLine_Digital[x] = True
            else:
                self.SensorLine_Digital[x] = False
        return self.SensorLine_Digital

    def getSensorSharp(self):
        Sensor = [0,0,0,0]
        for x in range(0,4):
            Sensor[x] = int(32120*((1+self.SensorSharp[x])**(-1.238)))
            if Sensor[x] > 400:
                Sensor[x] = 400
        return Sensor
    
    def getAnalogs(self):
        return self.Analogicas

    def setPullingTime(self,x):
        self.pullingTime = x

    def getPullingTime(self):
        return self.pullingTime
    
    def confGPIO(self,pin,mode):
        '''
        FORMATO: confGPIO( pin , mode )

        PARAMETROS:
            pin -> Numero de pin [0..6]
            mode -> [INPUT,OUTPUT]
        '''
        if mode == "INPUT":
            self.GPIO_conf[pin] = 0
        if mode == "OUTPUT":
            self.GPIO_conf[pin] = 1
        self.setOutBuffer()

    def setGPIO(self,pin,value):
        '''
        FORMATO: confGPIO( pin , value )

        PARAMETROS:
            pin -> Numero de pin [0..6]
            value -> [0,1]
        '''
        if value :
            self.GPIO_value[pin] = 1
        else:
            self.GPIO_value[pin] = 0
        self.setOutBuffer()

    def getGPIO(self,pin):
        '''
        FORMATO: getGPIO( pin )

        PARAMETROS:
            pin -> Numero de pin [0..6]

        RETORNA:
            Estado de la GPIO [0..1]
        '''
        return self.GPIO_status[pin]

    def setSERVO(self,pin,value):
        '''
        FORMATO: setSERVO( pin , value )

        PARAMETROS:
            pin -> Numero de Servo [0..4]
            value -> [1000..2000] Ancho de Pulsos en microsegundos
        '''
        self.Servos[pin] = value
        self.setOutBuffer()

    def getSONAR(self):
        '''
        FORMATO: getSONAR()

        RETORNA:
            [X]  Vector con distáncia hasta obstaculo  [0 .. 4000] mm
        '''
        return ((self.Sonar * 0.1759)+5.0039)
    
    def setMotorPosition(self,consigna):
        '''
        FORMATO: setMotorPosition(consigna)

        PARAMETROS:
            
        '''
        self.AuxMotor['SP'] = consigna
        self.setOutBuffer()

    def setMotorHome(self):
        '''
        FORMATO: setMotorHome()

        PARAMETROS:
            
        '''
        self.AuxMotor['Home'] = 1
        self.setOutBuffer()

class Clock():
    def __init__(self,pollTime):
        self.TempsOld = round(time.time()*1000)
        self.TempsActual = round(time.time()*1000)
        self.TempsCicle = pollTime

    def timeout(self):
        self.TempsActual = round(time.time()*1000)
        if (self.TempsActual-self.TempsOld>= self.TempsCicle):
            self.TempsOld = self.TempsActual
            return True
        else:
            return False
    def restart(self):
        self.TempsOld = round(time.time()*1000)


class MinairoSensorPerimetre(Canvas):
    def dimensions(self, W, H):
        self._Width = W
        self._Height = H
        self.FondoEscala = 400
        self.openAngle = 10
        MinairoSensorPerimetre.config(self,width=self._Width,height=H+10, bg='black')
        self.SensorSharp = [0,0,0,0]
        self.Nsensors = 4
        self.i=0
        self.Gap=30
        self.SenRad = self._Width//30
        self.Xa=self.Gap
        self.Ya=self.Gap
        self.Xb=self._Width-self.Gap
        self.Yb=self.Gap
        self.Xc=self.Gap
        self.Yc=self._Height
        self.Xd=self._Width-self.Gap
        self.Yd=self._Height
        self.Xo=self._Width//2
        self.Yo=self._Height
        self.DeltaH=(self.Yc-self.Ya)//5
        self.DeltaW=(self.Xo-self.Xc)//5
        self.DeltaXp=(self.Yc-self.Ya)/tan(3.1415/3)
        self.DeltaYp=tan(3.1415/3)*(self.Xd-self.Xo)
        self.DeltaRx=self.DeltaW*cos(3.1415/3)
        self.DeltaRy=self.DeltaW*sin(3.1415/3)
        if (self.DeltaYp<(self.Yd-self.Yb)):
            self.x2=self.Xb
            self.x1=self.Xa
            self.y2=self.Yd-self.DeltaYp
            self.y1=self.y2
            self.EscalableX=(self.Xo-self.Xc-self.DeltaRx)
            self.EscalableY=(self.DeltaYp-self.DeltaRy)
        else:
            self.x2=self.Xo+self.DeltaXp
            self.x1=self.Xo-self.DeltaXp
            self.y2=self.Yb
            self.y1=self.y2
            self.EscalableX=(self.DeltaXp-self.DeltaRx)
            self.EscalableY=(self.Yo-self.Ya-self.DeltaRy)



    def update(self,valor):
        color=['green','green','green','green']
        self.delete(ALL)
        self.create_line(self.Xa,self.Ya,self.Xc,self.Yc, fill='darkgreen' )
        self.create_line(self.Xb,self.Yb,self.Xd,self.Yd, fill='darkgreen' )
        self.create_line(self.Xa,self.Ya,self.Xb,self.Yb, fill='darkgreen' )
        self.create_line(self.Xc,self.Yc,self.Xd,self.Yd, fill='darkgreen' )

        self.create_line(self.Xo,self.Yo,self.x2,self.y2,fill='darkgreen')
        self.create_line(self.Xo,self.Yo,self.x1,self.y1,fill='darkgreen')
        for i in range(5):
            self.create_arc(self.Xo-self.DeltaW*(i+1),self.Yo-self.DeltaH*(i+1),self.Xo+self.DeltaW*(i+1),self.Yo+self.DeltaH*(i+1), start=0,extent=180,outline='darkgreen')
        self.create_arc(self.Xo-self.DeltaW,self.Yo-self.DeltaH,self.Xo+self.DeltaW,self.Yo+self.DeltaH, start=0,extent=180,outline='darkgreen',fill='darkgreen')
        modulX0 = (valor[0]*(self.Xo-self.DeltaW-self.Xc))//400
        modulX1 = (valor[1]*self.EscalableX)//400
        modulY1 = (valor[1]*self.EscalableY)//400
        modulX2 = (valor[2]*self.EscalableX)//400
        modulY2 = (valor[2]*self.EscalableY)//400
        modulX3 = (valor[3]*(self.Xo-self.DeltaW-self.Xc))//400
        x0=self.Xo+self.DeltaW+modulX0
        x1=self.Xo+self.DeltaRx+modulX1
        y1=self.Yo-self.DeltaRy-modulY1
        x2=self.Xo-self.DeltaRx-modulX2
        y2=self.Yo-self.DeltaRy-modulY2
        x3=self.Xo-self.DeltaW-modulX3
        for i in range(4):
            if valor[i]<100:
                color[i]='red'
            elif valor[i]<200:
                color[i]='orange'
            elif valor[i]<300:
                color[i]='yellow'
            else:
                color[i]='green1'
        self.create_oval(x0-self.SenRad,self.Yc-self.SenRad,x0+self.SenRad,self.Yc+self.SenRad,outline='darkgreen',fill=color[0])
        self.create_oval(x1-self.SenRad,y1-self.SenRad,x1+self.SenRad,y1+self.SenRad,outline='darkgreen',fill=color[1])
        self.create_oval(x2-self.SenRad,y2-self.SenRad,x2+self.SenRad,y2+self.SenRad,outline='darkgreen',fill=color[2])
        self.create_oval(x3-self.SenRad,self.Yc-self.SenRad,x3+self.SenRad,self.Yc+self.SenRad,outline='darkgreen',fill=color[3])
        self.create_text(self.Xd+15,self.Yd-8,text=valor[0],fill='darkgreen',font=('Helvetica 8'))
        self.create_text(self.x2+15,self.y2-10,text=valor[1],fill='darkgreen',font=('Helvetica 8'))
        self.create_text(self.x1-15,self.y1-10,text=valor[2],fill='darkgreen',font=('Helvetica 8'))
        self.create_text(self.Xc-15,self.Yc-8,text=valor[3],fill='darkgreen',font=('Helvetica 8'))

class MinairoSensorLine(Canvas):
    def dimensions(self, W, H):
        self.LEDheight = 20
        self.columnWidth = W//8
        self.columnHeight = H-self.LEDheight
        self._Width = 8*self.columnWidth
        self._Height = H
        self.convFactor = self.columnHeight//2500
        MinairoSensorLine.config(self,width=self._Width,height=self._Height, bg='black')
        self.threshold = 2400
        self.scaledThreshold = (self.threshold*self.columnHeight)//2500
        self.SensorLine_Analog = [self._Height/2,self._Height/3,self._Height/4,self._Height/5,self._Height/4,self._Height/3,self._Height/2,self._Height]
        self.SensorsAnalog=list()
        self.Nsensors = 8
        self.i=0
        
        for i in range(self.Nsensors):
            w=self.create_rectangle(self.columnWidth*i,self.columnHeight-self.SensorLine_Analog[i],self.columnWidth*(i+1),self.columnHeight, fill='green')
            self.SensorsAnalog.append(w)
        self.create_line(0,self.columnHeight-(self.scaledThreshold), self._Width, self.columnHeight-(self.scaledThreshold), width=2, fill='darkgreen')
        for i in range(self.Nsensors):
            if self.SensorLine_Analog[i] > self.scaledThreshold:
                w=self.create_rectangle(self.columnWidth*i,self.columnHeight,self.columnWidth*(i+1),self.columnHeight+self.LEDheight, fill='green1')
            else:
                w=self.create_rectangle(self.columnWidth*i,self.columnHeight,self.columnWidth*(i+1),self.columnHeight+self.LEDheight, fill='darkgreen')
            self.SensorsAnalog.append(w)

    
    def update(self,valors):
        for i in range(self.Nsensors):
            self.SensorLine_Analog[i] = (valors[i]*self.columnHeight)//2500
        self.delete(ALL)
        self.SensorsAnalog.clear    
        for i in range(self.Nsensors):
            w=self.create_rectangle(self.columnWidth*i,self.columnHeight-self.SensorLine_Analog[i],self.columnWidth*(i+1),self.columnHeight, fill='green')
            self.SensorsAnalog.append(w)
        self.create_line(0,self.columnHeight-(self.scaledThreshold), self._Width, self.columnHeight-(self.scaledThreshold), width=2, fill='darkgreen')
        for i in range(self.Nsensors):
            if self.SensorLine_Analog[i] > self.scaledThreshold:
                w=self.create_rectangle(self.columnWidth*i,self.columnHeight,self.columnWidth*(i+1),self.columnHeight+self.LEDheight, fill='green1')
            else:
                w=self.create_rectangle(self.columnWidth*i,self.columnHeight,self.columnWidth*(i+1),self.columnHeight+self.LEDheight, fill='darkgreen')
            self.SensorsAnalog.append(w)

class TON():
    """TON() - Temporizador con retardo a l'activación"""
    def __init__(self):
        self._IN = False
        self._PT = 0
        self._Q = False
        self._ET = 0
        self.TempsOld = round(time.time()*1000)
        self.TempsActual = round(time.time()*1000)

    def update(self):
        if self._IN:
            self.TempsActual = round(time.time()*1000)
            self._ET = self.TempsActual-self.TempsOld
            if (self._ET >= self._PT):
                self._Q=True
            else:
                self._Q=False
        else:
            self.TempsActual = round(time.time()*1000)
            self.TempsOld = self.TempsActual
            self._Q=False
        
    def IN(self,valor):
        self._IN = valor

    def PT(self,valor):
        self._PT = valor
    
    def Q(self):
        return self._Q
    
    def ET(self):
        return self._ET

class MinairoSonar(Canvas):
    def dimensions(self, W):
        self._Width = W
        self._Height = W
        MinairoSonar.config(self,width=self._Width,height=self._Height, bg='black')
        self.FondoEscala = 400
        self.SonarPoints = 360
        self.Sonar=[0 for _ in range( self.SonarPoints)]
        for i in range(360):
            self.Sonar[i]=2
        self.i=0
        self.Gap=20
        #self.SenRad = self._Width//100
        self.SenRad = 1
        self.Xa=self.Gap
        self.Ya=self.Gap
        self.Xb=self._Width-self.Gap
        self.Yb=self.Gap
        self.Xc=self.Gap
        self.Yc=self._Height-self.Gap
        self.Xd=self._Width-self.Gap
        self.Yd=self._Height-self.Gap
        self.Xo=self._Width//2
        self.Yo=self._Height//2
        self.Divisions = 8
        self.DeltaH=(self.Yo-self.Ya)//self.Divisions
        self.DeltaW=(self.Xo-self.Xc)//self.Divisions
        self.Escalable=(self.Xo-self.DeltaW-self.Xc)//2
        column, row = 2, 360
        self.Trigonom = [[0 for _ in range(column)] for _ in range(row)]
        for i in range(360):
            self.Trigonom[i][0]=sin((3.14156/180)*i)*self.Escalable
            self.Trigonom[i][1]=cos((3.14156/180)*i)*self.Escalable
        print(self.Trigonom)
 
    def update(self):
        color=['green','green','green','green']
        self.delete(ALL)
        self.create_line(self.Xa,self.Ya,self.Xc,self.Yc, fill='darkgreen' )
        self.create_line(self.Xb,self.Yb,self.Xd,self.Yd, fill='darkgreen' )
        self.create_line(self.Xa,self.Ya,self.Xb,self.Yb, fill='darkgreen' )
        self.create_line(self.Xc,self.Yc,self.Xd,self.Yd, fill='darkgreen' )

        self.create_line(self.Xa,self.Ya,self.Xd,self.Yd,fill='darkgreen')
        self.create_line(self.Xb,self.Yb,self.Xc,self.Yc,fill='darkgreen')
        self.create_line(self.Gap,self.Yo,self._Width-self.Gap,self.Yo,fill='darkgreen')
        self.create_line(self.Xo,self.Gap,self.Xo,self._Height-self.Gap,fill='darkgreen')


        for i in range(self.Divisions):
            self.create_oval(self.Xo-self.DeltaW*(i+1),self.Yo-self.DeltaH*(i+1),self.Xo+self.DeltaW*(i+1),self.Yo+self.DeltaH*(i+1),outline='darkgreen')
        #self.create_oval(self.Xo-self.DeltaW,self.Yo-self.DeltaH,self.Xo+self.DeltaW,self.Yo+self.DeltaH,outline='darkgreen',fill='darkgreen')

        self.create_text(self.Xo,self.Gap//2,justify='center',text="0",fill='darkgreen',font=('Helvetica 8'))
        self.create_text(self.Xo,self._Height-self.Gap//2,justify='center',text="180",fill='darkgreen',font=('Helvetica 8'))
        self.create_text(self._Width-self.Gap//2,self.Yo,justify='center',text="90",fill='darkgreen',font=('Helvetica 8'))
        self.create_text(self.Gap//2,self.Yo,justify='center',text="270",fill='darkgreen',font=('Helvetica 8'))
        
    def clear(self):
        for i in range(360):
            self.Sonar[i]=-1
        self.update()
        

    def plot(self,valor,angle):
        self.Sonar[angle]=valor
        xa=self.Sonar[angle]*round( self.Trigonom[angle][0])
        ya=self.Sonar[angle]*round(self.Trigonom[angle][1])
        if self.Sonar[angle]<2 and self.Sonar[angle]>0:
            self.create_oval(self.Xo+xa-self.SenRad,self.Yo-ya-self.SenRad,self.Xo+xa+self.SenRad,self.Yo-ya+self.SenRad,outline='green1',fill='green1')

    def plotAll(self):
        for i in range(360):
            xa=self.Sonar[i]*round( self.Trigonom[i][0])
            ya=self.Sonar[i]*round(self.Trigonom[i][1])
            if self.Sonar[i]<2 and self.Sonar[i]>0:
                self.create_oval(self.Xo+xa-self.SenRad,self.Yo-ya-self.SenRad,self.Xo+xa+self.SenRad,self.Yo-ya+self.SenRad,outline='green1',fill='green1')
        