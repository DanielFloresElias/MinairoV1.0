/*  
 *  __  __ _             _        _    __   ___   
 * |  \/  (_)           (_)      //   /_ | / _ \ 
 * | \  / |_ _ __   __ _ _ _ __ ___    | || | | |
 * | |\/| | | '_ \ / _` | | '__/ _ \   | || | | |
 * | |  | | | | | | (_| | | | | (_) |  | || |_| |
 * |_|  |_|_|_| |_|\__,_|_|_|  \___/   |_(_)___/
 * 
 * Descripción: Este firmware, realiza la cinemàtica del robot omnidireccional
 * en configuración de tres ejes a 120º.
 * 
 * Funciones del Master:
 *    - Control de las tres ruedas motrices.
 *    - Control de los lazos de control de velocidad para cada rueda.
 *    - Cálculo de la cinemàtica necesaria para topologia de robot omni de tres ruedas
 * 
 * Versión: 1.0
 * Autor: Daniel Flores Elias
 * mail: daniel.flores@gmail.com
 *                                                                                                              
 * Copyright (c) 2023 Institut Jaume Huguet-Valls, Daniel Flores
 * <daniel.flores@gmail.com>  All rights reserved.
 * See the bottom of this file for the license terms.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 * 
 * 1. Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "Encoder.h"
#include "AccelMotor.h"
#include "invKinematic.h"
#include <EEPROM.h>
#include <Wire.h>
#include "COMParameters.h"

int eeAddress = 0;
int SALVE_1_ADDR = 9;
_SLAVE1_IN  Slave_1_IN;

// Change these two numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
Encoder myEnc1(2, 3);
Encoder myEnc2(8, 11);
Encoder myEnc3(14, 15);
//   avoid using pins with LEDs attached

AccelMotor motor1(DRIVER3WIRE, 4, 7, 5, HIGH);
AccelMotor motor2(DRIVER3WIRE, 12, 13, 6, HIGH);
AccelMotor motor3(DRIVER3WIRE, 16, 17, 9, HIGH);

float Robot_Radius=0.15;  // Radio en metros. Longitud desde el centro del robot hasta el punto central de la rueda.
float Wheel_Radius=0.04;  // Radio en metros. Radio de las ruedas omnidireccionales.
const float Rad2Ticks=4480/(2*PI);

invKinematic invKin(Robot_Radius, Wheel_Radius);      // invKin(Robot_Radius,Wheel_Radius)

float Teta = 0;
float Dist = 0;
float DistTotal = 0;
//float Radi = 1; // Radi de la Roda en metres
float Speed = 0;
unsigned long TempsOld=0;
uint8_t tempsPulling=20;
const float tempsCalc = float(tempsPulling)/1000.0;
comando_speed ComandoSpeed;
motor_speed MotorSpeed;
String InString = "";
String Comando = "";
String InString2 = "";
String Comando2 = "";
bool DataReceived = false;
String Temporal = "";

bool  TimeOut = false;
int TempsTimeOut=1100;
unsigned long TempsOld_TimeOut=0;

void setup() {
  Wire.setClock( 400000L);
  Wire.begin(SALVE_1_ADDR);
  Wire.onReceive(receiveEvent); 
  Serial.begin(115200);
  // Yo uso Motor con reductora 70:1 con encoder de Pololu 4754 (https://www.pololu.com/product/4754)
  // codificador 64 tics por revolución
  motor1.setRatio(70 * 64);
  motor2.setRatio(70 * 64);
  motor3.setRatio(70 * 64);

  // período de integración (por defecto 20)
  motor1.setDt(20);  // milisegundos
  motor2.setDt(20);
  motor3.setDt(20);

  // configuración de la velocidad máxima para el modo ACCEL_POS
  motor1.setMaxSpeedDeg(600);  // en grados / seg
  motor2.setMaxSpeedDeg(600);
  motor3.setMaxSpeedDeg(600);

  // configuración de la aceleración para el modo ACCEL_POS
  motor1.setAccelerationDeg(300);  // en grados / seg / seg
  motor2.setAccelerationDeg(300);
  motor3.setAccelerationDeg(300);

  // señal PWM mínima (módulo) (en la que el motor arranca)
  motor1.setMinDuty(10);
  motor2.setMinDuty(10);
  motor3.setMinDuty(10);

  // Coeficientes del regulador PID
  motor1.kp = 0.02;   // responsable de la nitidez de la regulación.
  motor2.kp = 0.02; 
  motor3.kp = 0.02;
  // A valores bajos, no habrá señal en absoluto, a valores demasiado altos, temblará

  motor1.ki = 0.05; // responsable de corregir el error a lo largo del tiempo
  motor2.ki = 0.05;
  motor3.ki = 0.05;

  motor1.kd = 0; // responsable de compensar los cambios repentinos
  motor2.kd = 0;
  motor3.kd = 0;

  // establecer la zona de parada del motor para el modo de estabilización de posición en ticks
  motor1.setStopZone(30);
  motor2.setStopZone(30);
  motor3.setStopZone(30);

  motor1.setRunMode(PID_SPEED);
  motor2.setRunMode(PID_SPEED);
  motor3.setRunMode(PID_SPEED);

}

void receiveEvent(int howMany)
{
  int n = 0;
  while (Wire.available()) { // slave may send less than requested
    Slave_1_IN.B[n] = Wire.read();    // receive a byte as character
    n++;
  }
  DataReceived = true;     // nova línea.
}


void loop() {

  ////////////   C O M U N I C A C I O N E S  ////////////
   if (DataReceived) {
        TempsOld_TimeOut = millis();
        DataReceived = false; // Reinicia el valor de DataReceived
        ComandoSpeed.linear[0] = Slave_1_IN.INPUT_DATA.x;
        ComandoSpeed.linear[1] = Slave_1_IN.INPUT_DATA.y;
        ComandoSpeed.angular[2] = Slave_1_IN.INPUT_DATA.w;

        MotorSpeed = invKin.calc(ComandoSpeed);
        motor1.setTargetSpeed(MotorSpeed.angular[0]*Rad2Ticks);
        motor2.setTargetSpeed(MotorSpeed.angular[1]*Rad2Ticks);
        motor3.setTargetSpeed(MotorSpeed.angular[2]*Rad2Ticks);
    }///______________ FINAL DE COMUNICACIONES ______________

  // CONTROL DEL TIMEOUT DE LES TRANSMISSIONS PER LA UART //
  //////////////////////////////////////////////////////////
  if ( millis() - TempsOld_TimeOut > TempsTimeOut ){
        motor1.setTargetSpeed(0);
        motor2.setTargetSpeed(0);
        motor3.setTargetSpeed(0);
    }
   
  // función requerida. Hace todos los cálculos
  // toma el valor actual de un codificador o potenciómetro
  motor1.tick(myEnc1.read());
  motor2.tick(myEnc2.read());
  motor3.tick(myEnc3.read());
}
