/*
    __  __ _             _        _    __   ___
   |  \/  (_)           (_)      //   /_ | / _ \
   | \  / |_ _ __   __ _ _ _ __ ___    | || | | |
   | |\/| | | '_ \ / _` | | '__/ _ \   | || | | |
   | |  | | | | | | (_| | | | | (_) |  | || |_| |
   |_|  |_|_|_| |_|\__,_|_|_|  \___/   |_(_)___/

   Descripción: Este firmware, gestiona las comunicaciones entre el Robot Minairó i la UART.
   El Master, gestiona los comandos entrantes. Atiende los comandos locales, i redirecciona los
   comandos a los Slaves correspondientes.

   Funciones del M0aster:
      - Control del motor auxiliar (MOTOR_3) Con control de posición.
      - 2x Finales de carrera MOTOR_3
      - 8x Sensores del seguidor de Línea
      - 4x Sensores Analógicos perimetrales
      - Comunicación UART con periferia
      - Comunicación I2C con Slaves.

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

#include "Encoder.h"
#include "AccelMotor.h"
#include <EEPROM.h>
#include <QTRSensors.h>
#include <Wire.h>
#include "COMParameters.h"


int eeAddress = 0;
int SALVE_0_ADDR = 8;

_SLAVE0_IN  Slave_0_IN;
_SLAVE0_OUT Slave_0_OUT;

Encoder myEnc(11, 12);
//   avoid using pins with LEDs attached
AccelMotor motor(DRIVER3WIRE, 13, 20, 10, HIGH);

QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

const float Rad2Ticks = 4480 / (2 * PI);


float Teta = 0;
float Dist = 0;
float DistTotal = 0;
float Radi = 1; // Radi de la Roda en metres
float Speed = 0;
unsigned long TempsOld = 0;
uint8_t tempsPulling = 20;
const float tempsCalc = float(tempsPulling) / 1000.0;

String InString = "";
String OutString = "";
String Comando = "";
bool NewLine = false;
String Temporal = "";

bool  Transmit = false;
bool  TimeOut = false;
int TempsTimeOut = 200;
unsigned long TempsOld_TimeOut = 0;

int TransmitTime = 100;
unsigned long TempsOld_TransmitTime = 0;

long oldPosition  = 0;

int H_LimitSwitch = 0;
bool H_LimitSwitchOld = 0;
int L_LimitSwitch = 0;
bool L_LimitSwitchOld = 0;

bool MotorHomeFlag = false;
bool DataReceived = false;

const int Sensors = 4;
const int Mostres = 8;
int SensorArray[ Sensors ][ Mostres ];
uint16_t TempSharpArray[Sensors];




void setup() {
  Slave_0_IN.INPUT_DATA.CtrlWord1 = 0;
  Slave_0_IN.INPUT_DATA.CtrlWord2 = 0;
  Slave_0_IN.INPUT_DATA.SetPoint = 0;
  Wire.setClock( 400000L);
  Wire.begin(SALVE_0_ADDR);
  Serial.begin(115200);
  Wire.onRequest(RequestEven); // registrar evento de solicitud de datos
  Wire.onReceive(receiveEvent); // registrar evento de recepcion de datos

  // Yo uso el motor
  // codificador 64 tics por revolución
  motor.setRatio(70 * 64);
  // período de integración (por defecto 20)
  motor.setDt(20);  // milisegundos
  // configuración de la velocidad máxima para el modo ACCEL_POS
  motor.setMaxSpeedDeg(300);  // en grados / seg
  // configuración de la aceleración para el modo ACCEL_POS
  motor.setAccelerationDeg(5);  // en grados / seg / seg
  // señal PWM mínima (módulo) (en la que el motor arranca)
  motor.setMinDuty(10);
  // Coeficientes del regulador PID
  motor.kp = 0.5;   // responsable de la nitidez de la regulación.
  // A valores bajos, no habrá señal en absoluto, a valores demasiado altos, temblará
  motor.ki = 0.0; // responsable de corregir el error a lo largo del tiempo
  motor.kd = 0.0; // responsable de compensar los cambios repentinos
  // establecer la zona de parada del motor para el modo de estabilización de posición en ticks (por defecto 8)
  motor.setStopZone(100);
  motor.setRunMode(PID_POS);

  // configure the sensors
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]) {
    2, 3, 4, 5, 6, 7, 8, 9
  }, SensorCount);
  qtr.calibrate();

  for (uint8_t i = 0; i < SensorCount; i++)
  {
    qtr.calibrationOn.minimum[i] = 500;
    qtr.calibrationOn.maximum[i] = 2500;
  }

}

void loop() {
  //////////// P U L L I N G //////////////
  if (millis() > (TempsOld + tempsPulling)) {
    TempsOld += tempsPulling;
    Slave_0_OUT.OUTPUT_DATA.EncoderValue = myEnc.read();

    if (!(Slave_0_IN.INPUT_DATA.CtrlWord1 & MotorHomeMask)) {
      MotorHomeFlag = false;
      motor.setTarget(Slave_0_IN.INPUT_DATA.SetPoint);
    }

    if ((Slave_0_IN.INPUT_DATA.CtrlWord1 & MotorHomeMask) && !MotorHomeFlag) {
      MotorHomeFlag = true;
      Slave_0_IN.INPUT_DATA.SetPoint = 0;
      myEnc.write(0);
      motor.setTarget(0);
    }

    ////////////// Control de Finales de carrera del motor auxiliar //////////////
    Slave_0_OUT.OUTPUT_DATA.StatusWord1 = (analogRead(A7) > 768) ? Slave_0_OUT.OUTPUT_DATA.StatusWord1 | MotorHswMask : Slave_0_OUT.OUTPUT_DATA.StatusWord1 & ~MotorHswMask ;
    Slave_0_OUT.OUTPUT_DATA.StatusWord1 = (analogRead(A7) < 256) ? Slave_0_OUT.OUTPUT_DATA.StatusWord1 | MotorLswMask : Slave_0_OUT.OUTPUT_DATA.StatusWord1 & ~MotorLswMask ;

    ////////////// Forzado del Final de carrera HIGH //////////////
    if ((Slave_0_OUT.OUTPUT_DATA.StatusWord1 & MotorHswMask) && !H_LimitSwitchOld)
    {
      Slave_0_IN.INPUT_DATA.SetPoint = Slave_0_OUT.OUTPUT_DATA.EncoderValue;
    }
    ////////////// Forzado del Final de carrera LOW //////////////
    if ((Slave_0_OUT.OUTPUT_DATA.StatusWord1 & MotorLswMask) && !L_LimitSwitchOld)
    {
      Slave_0_IN.INPUT_DATA.SetPoint = Slave_0_OUT.OUTPUT_DATA.EncoderValue;
    }
    H_LimitSwitchOld = (Slave_0_OUT.OUTPUT_DATA.StatusWord1 & MotorHswMask) ? true : false;
    L_LimitSwitchOld = (Slave_0_OUT.OUTPUT_DATA.StatusWord1 & MotorLswMask) ? true : false;

    //motor.setMaxSpeedDeg(xxxx);  // en grados / seg


    ///////////// Actualización sensores de seguidor de línea /////////////
    qtr.read(sensorValues);
    Slave_0_OUT.OUTPUT_DATA.LS_0 = sensorValues[0];
    Slave_0_OUT.OUTPUT_DATA.LS_1 = sensorValues[1];
    Slave_0_OUT.OUTPUT_DATA.LS_2 = sensorValues[2];
    Slave_0_OUT.OUTPUT_DATA.LS_3 = sensorValues[3];
    Slave_0_OUT.OUTPUT_DATA.LS_4 = sensorValues[4];
    Slave_0_OUT.OUTPUT_DATA.LS_5 = sensorValues[5];
    Slave_0_OUT.OUTPUT_DATA.LS_6 = sensorValues[6];
    Slave_0_OUT.OUTPUT_DATA.LS_7 = sensorValues[7];

    ///////////// Actualización sensores perimetrales /////////////
    for (int i = (Mostres - 1); i > 0; i--) {
      for (int j = 0; j < Sensors; j++) {
        SensorArray[ j ][ i ] = SensorArray[ j ][i - 1];
      }
    }
    SensorArray[ 0 ][ 0 ] = analogRead(A0);
    SensorArray[ 1 ][ 0 ] = analogRead(A1);
    SensorArray[ 2 ][ 0 ] = analogRead(A2);
    SensorArray[ 3 ][ 0 ] = analogRead(A3);
    
    for(int j = 0; j<Sensors ;j++){
      TempSharpArray[j] = 0;
      for(int i = 0; i < Mostres; i++){
        TempSharpArray[j] = TempSharpArray[j] + SensorArray[j][i];
      }
      TempSharpArray[j] = TempSharpArray[j] >> 3;
    }

    Slave_0_OUT.OUTPUT_DATA.SS_0 = TempSharpArray[0];
    Slave_0_OUT.OUTPUT_DATA.SS_1 = TempSharpArray[1];
    Slave_0_OUT.OUTPUT_DATA.SS_2 = TempSharpArray[2];
    Slave_0_OUT.OUTPUT_DATA.SS_3 = TempSharpArray[3];;

  } ///______________ FINAL DEL PULLING ______________

  ////////////   C O M U N I C A C I O N E S  ////////////
  if (DataReceived) {
    TempsOld_TimeOut = millis();
    DataReceived = false; // Reinicia el valor de DataReceived

  }///______________ FINAL DE COMUNICACIONES ______________


  // función requerida. Hace todos los cálculos
  // toma el valor actual de un codificador o potenciómetro
  motor.tick(myEnc.read());
}


//***************** F U N C T I O N S *****************

// Función requerida al recibir datos por I2C
void receiveEvent(int howMany)
{
  int n = 0;
  while (Wire.available()) { // slave may send less than requested
    Slave_0_IN.B[n] = Wire.read();    // receive a byte as character
    n++;
  }
  DataReceived = true;     // nova línea.
}

// Función a ejecutar cuando Maestro solicita datos
void RequestEven() {
  for (int i = 0; i < sizeof(Slave_0_OUT); i++) {
    Wire.write(Slave_0_OUT.B[i]);
  }
}
