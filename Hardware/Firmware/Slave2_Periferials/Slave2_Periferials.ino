/*
    __  __ _             _        _    __   ___
   |  \/  (_)           (_)      //   /_ | / _ \
   | \  / |_ _ __   __ _ _ _ __ ___    | || | | |
   | |\/| | | '_ \ / _` | | '__/ _ \   | || | | |
   | |  | | | | | | (_| | | | | (_) |  | || |_| |
   |_|  |_|_|_| |_|\__,_|_|_|  \___/   |_(_)___/

   Descripción: Este firmware, realiza el control de los sistemas perifericos
   del Robot.

   Funciones del Slave2:
      - Control de 5 servos RC.
      - Control de 7 GPIO
      - Control de 4 AnalogInputs
      - Control del sonar por ultrasonidos.

   Versión: 1.0
   Autor: Daniel Flores Elias
   mail: daniel.flores@gmail.com

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
*/

#include <EEPROM.h>
#include <Wire.h>
#include <Servo.h>
#include "COMParameters.h"

Servo SERVO_0, SERVO_1, SERVO_2, SERVO_3, SERVO_4;

int eeAddress = 0;
int SALVE_2_ADDR = 10;

_SLAVE2_IN  Slave_2_IN;
_SLAVE2_OUT Slave_2_OUT;
_SLAVE2_IN Slave_2_IN_copy;

unsigned long TempsOld = 0;
uint8_t tempsPulling = 20;

unsigned long TempsSonarOld = 0;
uint8_t tempsSonarPulling = 50;

int GPIO_0_pin = 7;
int GPIO_1_pin = 8;
int GPIO_2_pin = 11;
int GPIO_3_pin = 12;
int GPIO_4_pin = 13;
int GPIO_5_pin = 20;
int GPIO_6_pin = 21;
const int SONAR_TRIG_pin = 2;
const int SONAR_ECHO_pin = 4;
volatile uint32_t t, tH, tL;
volatile unsigned int pulseWidth = 0;
long duration;

int distance;
uint8_t TempByte = 0;
bool DataReceived = false;

void setup() {
  Wire.setClock( 400000L);
  Wire.begin(SALVE_2_ADDR);
  Wire.onRequest(RequestEven); // registrar evento de solicitud de datos
  Wire.onReceive(receiveEvent); // registrar evento de recepcion de datos
  Serial.begin(115200);
  SERVO_0.attach(3);
  SERVO_1.attach(5);
  SERVO_2.attach(6);
  SERVO_3.attach(9);
  SERVO_4.attach(10);
  Slave_2_IN.INPUT_DATA.Servo_0 = 1500;
  Slave_2_IN.INPUT_DATA.Servo_1 = 1500;
  Slave_2_IN.INPUT_DATA.Servo_2 = 1500;
  Slave_2_IN.INPUT_DATA.Servo_3 = 1500;
  Slave_2_IN.INPUT_DATA.Servo_4 = 1500;
  Slave_2_IN.INPUT_DATA.GPIO_Config = 0;
  Slave_2_IN.INPUT_DATA.GPIO_Value = 0;


  SERVO_0.writeMicroseconds(Slave_2_IN.INPUT_DATA.Servo_0);
  SERVO_1.writeMicroseconds(Slave_2_IN.INPUT_DATA.Servo_1);
  SERVO_2.writeMicroseconds(Slave_2_IN.INPUT_DATA.Servo_2);
  SERVO_3.writeMicroseconds(Slave_2_IN.INPUT_DATA.Servo_3);
  SERVO_4.writeMicroseconds(Slave_2_IN.INPUT_DATA.Servo_4);

  pinMode(SONAR_TRIG_pin, OUTPUT);
  pinMode(SONAR_ECHO_pin, INPUT);
  attachInterrupt(digitalPinToInterrupt(SONAR_ECHO_pin), ISR_Sonar, CHANGE);
}
//******************* Bucle principal *******************
void loop() {
  //////////// P U L L I N G //////////////
  if ( millis() - TempsSonarOld > tempsSonarPulling ) {
    TempsSonarOld = millis();

    //GENERAR PULSO DE SONAR
    Sonar_Pulse();
    //ACTUALIZAR DISTANCIA INDICADA POR EL SONAR
    //duration = pulseIn(SONAR_ECHO_pin, HIGH);
    // Calculating the distance
    //distance = int(duration * 0.034 / 2);
    Slave_2_OUT.OUTPUT_DATA.Sonar = pulseWidth;
    
    /*
    distance = pulseWidth*0.034/2;
    Slave_2_OUT.OUTPUT_DATA.Sonar = distance;
    pulseWidth = 0;
    */
  }

  
  if ( millis() - TempsOld > tempsPulling ) {
    TempsOld = millis();

    //REGISTRAR ESTADO DE LAS GPIOs:
    TempByte = digitalRead(GPIO_0_pin) ? TempByte | GPIO_0_Mask : TempByte & ~GPIO_0_Mask;
    TempByte = digitalRead(GPIO_1_pin) ? TempByte | GPIO_1_Mask : TempByte & ~GPIO_1_Mask;
    TempByte = digitalRead(GPIO_2_pin) ? TempByte | GPIO_2_Mask : TempByte & ~GPIO_2_Mask;
    TempByte = digitalRead(GPIO_3_pin) ? TempByte | GPIO_3_Mask : TempByte & ~GPIO_3_Mask;
    TempByte = digitalRead(GPIO_4_pin) ? TempByte | GPIO_4_Mask : TempByte & ~GPIO_4_Mask;
    TempByte = digitalRead(GPIO_5_pin) ? TempByte | GPIO_5_Mask : TempByte & ~GPIO_5_Mask;
    TempByte = digitalRead(GPIO_6_pin) ? TempByte | GPIO_6_Mask : TempByte & ~GPIO_6_Mask;

    Slave_2_OUT.OUTPUT_DATA.GPIO_Status = TempByte;


    //REGISTRAR ESTADO DE LAS ENTRADAS ANALÒGICAS:
    Slave_2_OUT.OUTPUT_DATA.AN_0 = analogRead(A0);
    Slave_2_OUT.OUTPUT_DATA.AN_1 = analogRead(A1);
    Slave_2_OUT.OUTPUT_DATA.AN_2 = analogRead(A2);
    Slave_2_OUT.OUTPUT_DATA.AN_3 = analogRead(A3);

  }///______________ FINAL DEL PULLING ______________

  ////////////   C O M U N I C A C I O N E S  ////////////
  if (DataReceived) {
    DataReceived = false; // Reinicia el valor de DataReceived
    bool GPIO_Config_Change = Slave_2_IN_copy.INPUT_DATA.GPIO_Config ^ Slave_2_IN.INPUT_DATA.GPIO_Config ? true : false ;
    bool GPIO_Value_Change = Slave_2_IN_copy.INPUT_DATA.GPIO_Value ^ Slave_2_IN.INPUT_DATA.GPIO_Value ? true : false ;
    bool Servo_0_Change = Slave_2_IN_copy.INPUT_DATA.Servo_0 ^ Slave_2_IN.INPUT_DATA.Servo_0 ? true : false ;
    bool Servo_1_Change = Slave_2_IN_copy.INPUT_DATA.Servo_1 ^ Slave_2_IN.INPUT_DATA.Servo_1 ? true : false ;
    bool Servo_2_Change = Slave_2_IN_copy.INPUT_DATA.Servo_2 ^ Slave_2_IN.INPUT_DATA.Servo_2 ? true : false ;
    bool Servo_3_Change = Slave_2_IN_copy.INPUT_DATA.Servo_3 ^ Slave_2_IN.INPUT_DATA.Servo_3 ? true : false ;
    bool Servo_4_Change = Slave_2_IN_copy.INPUT_DATA.Servo_4 ^ Slave_2_IN.INPUT_DATA.Servo_4 ? true : false ;

    // CONFIGURACIÓN DE GPIOs
    if (GPIO_Config_Change){
      if (Slave_2_IN.INPUT_DATA.GPIO_Config & GPIO_0_Mask) pinMode(GPIO_0_pin, OUTPUT); else pinMode(GPIO_0_pin,INPUT);
      if (Slave_2_IN.INPUT_DATA.GPIO_Config & GPIO_1_Mask) pinMode(GPIO_1_pin, OUTPUT); else pinMode(GPIO_1_pin,INPUT);
      if (Slave_2_IN.INPUT_DATA.GPIO_Config & GPIO_2_Mask) pinMode(GPIO_2_pin, OUTPUT); else pinMode(GPIO_2_pin,INPUT);
      if (Slave_2_IN.INPUT_DATA.GPIO_Config & GPIO_3_Mask) pinMode(GPIO_3_pin, OUTPUT); else pinMode(GPIO_3_pin,INPUT);
      if (Slave_2_IN.INPUT_DATA.GPIO_Config & GPIO_4_Mask) pinMode(GPIO_4_pin, OUTPUT); else pinMode(GPIO_4_pin,INPUT);
      if (Slave_2_IN.INPUT_DATA.GPIO_Config & GPIO_5_Mask) pinMode(GPIO_5_pin, OUTPUT); else pinMode(GPIO_5_pin,INPUT);
      if (Slave_2_IN.INPUT_DATA.GPIO_Config & GPIO_6_Mask) pinMode(GPIO_6_pin, OUTPUT); else pinMode(GPIO_6_pin,INPUT);
    }

    //ON/OFF DE SALIDAS DIGITALES: (aquellas configuradas como
    // entradas, no se veran afectadas.
    if (GPIO_Value_Change){
      digitalWrite(GPIO_0_pin, Slave_2_IN.INPUT_DATA.GPIO_Value & GPIO_0_Mask ? HIGH : LOW);
      digitalWrite(GPIO_1_pin, Slave_2_IN.INPUT_DATA.GPIO_Value & GPIO_1_Mask ? HIGH : LOW);
      digitalWrite(GPIO_2_pin, Slave_2_IN.INPUT_DATA.GPIO_Value & GPIO_2_Mask ? HIGH : LOW);
      digitalWrite(GPIO_3_pin, Slave_2_IN.INPUT_DATA.GPIO_Value & GPIO_3_Mask ? HIGH : LOW);
      digitalWrite(GPIO_4_pin, Slave_2_IN.INPUT_DATA.GPIO_Value & GPIO_4_Mask ? HIGH : LOW);
      digitalWrite(GPIO_5_pin, Slave_2_IN.INPUT_DATA.GPIO_Value & GPIO_5_Mask ? HIGH : LOW);
      digitalWrite(GPIO_6_pin, Slave_2_IN.INPUT_DATA.GPIO_Value & GPIO_6_Mask ? HIGH : LOW);
    }
    
    // ACTUALIZAR SERVOS
    if(Servo_0_Change) SERVO_0.writeMicroseconds(Slave_2_IN.INPUT_DATA.Servo_0);
    if(Servo_1_Change) SERVO_1.writeMicroseconds(Slave_2_IN.INPUT_DATA.Servo_1);
    if(Servo_2_Change) SERVO_2.writeMicroseconds(Slave_2_IN.INPUT_DATA.Servo_2);
    if(Servo_3_Change) SERVO_3.writeMicroseconds(Slave_2_IN.INPUT_DATA.Servo_3);
    if(Servo_4_Change) SERVO_4.writeMicroseconds(Slave_2_IN.INPUT_DATA.Servo_4);

    Slave_2_IN_copy = Slave_2_IN;
  }///______________ FINAL DE COMUNICACIONES ______________
}


//***************** F U N C T I O N S *****************

// Función requerida al recibir datos por I2C
void receiveEvent(int howMany)
{
  int n = 0;
  while (Wire.available()) { // slave may send less than requested
    Slave_2_IN.B[n] = Wire.read();    // receive a byte as character
    n++;
  }
  DataReceived = true;     // nova línea.
}

// Función a ejecutar cuando Maestro solicita datos
void RequestEven() {
    for (int i = 0; i < sizeof(Slave_2_OUT); i++) {
      Wire.write(Slave_2_OUT.B[i]);
    }
}

// Generar pulso para sonar
void Sonar_Pulse() {
  digitalWrite(SONAR_TRIG_pin, LOW);
  delayMicroseconds(2);
  digitalWrite(SONAR_TRIG_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(SONAR_TRIG_pin, LOW);
}

// Función para leer el ancho de pulso del echo del Sonar.
void ISR_Sonar() {
  t = micros();
  if ( digitalRead(SONAR_ECHO_pin) == HIGH){
    tH = t;  
  } 
  else {
    tL = t;
    if ((tL-tH)<11664) pulseWidth = (tL-tH);  // Limite del impulso a 2 metros
    else pulseWidth = 11664;
  }
}
