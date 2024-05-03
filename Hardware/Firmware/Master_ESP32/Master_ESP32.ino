/*
    __  __ _             _        _    __   ___
   |  \/  (_)           (_)      //   /_ | / _ \
   | \  / |_ _ __   __ _ _ _ __ ___    | || | | |
   | |\/| | | '_ \ / _` | | '__/ _ \   | || | | |
   | |  | | | | | | (_| | | | | (_) |  | || |_| |
   |_|  |_|_|_| |_|\__,_|_|_|  \___/   |_(_)___/

*/

//#include <esp_wifi.h>
#include <WiFi.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include "COMParameters.h"

/*
  typedef union{
  byte B[12];
  struct{
      float x;                // m/s
      float y;                // m/s
      float w;                // rad/s
  } INPUT_DATA;
  }_BUFFER_IN;

  _BUFFER_IN Buffer_IN;
  byte bufferArray[12];
*/
int kk = 0;
bool RawFlag = false;

HardwareSerial SerialPort(2); // use UART2

_RAW_IN    Raw_IN;
_RAW_OUT   Raw_OUT;

_SLAVE0_IN  Slave_0_IN;
_SLAVE0_OUT Slave_0_OUT;

_SLAVE1_IN  Slave_1_IN;

_SLAVE2_IN  Slave_2_IN;
_SLAVE2_OUT Slave_2_OUT;

#define MAX_SRV_CLIENTS 1
#include "config.h"
#define I2C_SDA 23
#define I2C_SCL 22

WiFiServer server(22);
//WiFiClient serverClients[MAX_SRV_CLIENTS];
WiFiClient serverClient;

int led = 13;
int SALVE_0_ADDR = 8;
int SALVE_1_ADDR = 9;
int SALVE_2_ADDR = 10;
int n = 0;

bool  TimeOut = false;
int TempsTimeOut = 500;
int TransmitTime = 50;
unsigned long TempsOld_TimeOut = 0;
unsigned long TempsOld_TransmitTime = 0;
unsigned long TempsCicle = 0;

String InString = "";
String Comando = "";
bool NewLine = false;
byte OutBuffer[512];
float Sonar;
int ReciveCounter = 0;
int byteCounter = 0;

void setup() {

  Serial.begin(115200);
  /* You can remove the password parameter if you want the AP to be open. */
  WiFi.softAP(ssid, password);
  WiFi.softAPConfig(local_ip, gateway, subnet);

  Slave_0_IN.INPUT_DATA.CtrlWord1 = 0B0000000000000000;
  Slave_0_IN.INPUT_DATA.CtrlWord2 = 0B0000000000000000;
  Slave_0_IN.INPUT_DATA.SetPoint = 0;

  Slave_1_IN.INPUT_DATA.x = 0.0;
  Slave_1_IN.INPUT_DATA.y = 0.0;
  Slave_1_IN.INPUT_DATA.w = 0.0;

  Slave_2_IN.INPUT_DATA.GPIO_Config = B00000000;
  Slave_2_IN.INPUT_DATA.GPIO_Value = B00000000;
  Slave_2_IN.INPUT_DATA.Servo_0 = 1500;
  Slave_2_IN.INPUT_DATA.Servo_1 = 1500;
  Slave_2_IN.INPUT_DATA.Servo_2 = 1500;
  Slave_2_IN.INPUT_DATA.Servo_3 = 1500;
  Slave_2_IN.INPUT_DATA.Servo_4 = 1500;


  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);
  // start telnet server
  server.begin();
  server.setNoDelay(true);
  SerialPort.begin(115200, SERIAL_8N1, 16, 17);
  pinMode(led, OUTPUT);
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock( 400000L);
}

void loop() {
  //Serial.println(millis()-TempsCicle);
  //  TempsCicle = millis();
  uint8_t i;
  if (server.hasClient()) {
    digitalWrite(led, HIGH);
  } else {
    digitalWrite(led, LOW);
  }
  //check if there are any new clients
  if (server.hasClient()) {
    /*
        for (i = 0; i < MAX_SRV_CLIENTS; i++) {
          //find free/disconnected spot
          if (!serverClients[i] || !serverClients[i].connected()) {
            if (serverClients[i]) serverClients[i].stop();
            serverClients[i] = server.available();
            continue;
          }
        }
    */
    //find free/disconnected spot
    if (!serverClient || !serverClient.connected()) {
      if (serverClient) serverClient.stop();
      serverClient = server.available();
      //continue;
    }

    //no free/disconnected spot so reject
    WiFiClient serverClient = server.available();
    serverClient.stop();
  }


  if (serverClient && serverClient.connected()) {
    if (serverClient.available()) {
      ////////////   C O M U N I C A C I O N S  ////////////
      Raw_IN.B[byteCounter] = serverClient.read();     // Llegeix el caràcter.
      byteCounter++;
      if (byteCounter == 32) {
        byteCounter = 0;
        NewLine = true;
      }
    }
    else {
      byteCounter = 0;
    }
  }

  // TRATAMIENTO DE LA TRAMA RECIBIDA //
  //////////////////////////////////////////////////////////
  if (NewLine) {  // Nova linea?
    TempsOld_TimeOut = millis();
    NewLine = false; // Reinicia el valor de NewLine
    Slave_1_IN.INPUT_DATA.x = -Raw_IN.INPUT_DATA.x;  // x(m/s);
    Slave_1_IN.INPUT_DATA.y = -Raw_IN.INPUT_DATA.y;  // y(m/s);
    Slave_1_IN.INPUT_DATA.w = -Raw_IN.INPUT_DATA.w;  // w(m/s);
    Slave_2_IN.INPUT_DATA.GPIO_Config = Raw_IN.INPUT_DATA.GPIO_Config;
    Slave_2_IN.INPUT_DATA.GPIO_Value = Raw_IN.INPUT_DATA.GPIO_Value;
    Slave_2_IN.INPUT_DATA.Servo_0 = Raw_IN.INPUT_DATA.Servo_0;
    Slave_2_IN.INPUT_DATA.Servo_1 = Raw_IN.INPUT_DATA.Servo_1;
    Slave_2_IN.INPUT_DATA.Servo_2 = Raw_IN.INPUT_DATA.Servo_2;
    Slave_2_IN.INPUT_DATA.Servo_3 = Raw_IN.INPUT_DATA.Servo_3;
    Slave_2_IN.INPUT_DATA.Servo_4 = Raw_IN.INPUT_DATA.Servo_4;

    // TRAMA DE RESPUESTA
    if (serverClient && serverClient.connected()) {
      serverClient.write(Raw_OUT.B, 44);
    }
  }

  
    // CONTROL DEL TIMEOUT DE LES TRANSMISSIONS PER LA UART //
    //////////////////////////////////////////////////////////
    if ( millis() - TempsOld_TimeOut > TempsTimeOut ) {
      Slave_1_IN.INPUT_DATA.x = 0.0;
      Slave_1_IN.INPUT_DATA.y = 0.0;
      Slave_1_IN.INPUT_DATA.w = 0.0;
    }

    // CONTROL DE LES TRANSMISSIONS PER LA I2C //
    //////////////////////////////////////////////////////////
    if ( millis() - TempsOld_TransmitTime > TransmitTime )
    {
      TempsOld_TransmitTime = millis();

      ///////////////// Comunicacion con SLAVE 0 /////////////////
      Wire.beginTransmission(SALVE_0_ADDR); // transmit to device SLAVE_0
      for (int i = 0; i < sizeof(Slave_0_IN); i++) {
        Wire.write(Slave_0_IN.B[i]);
      }
      Wire.endTransmission();    // stop transmitting

      Wire.requestFrom(SALVE_0_ADDR, sizeof(Slave_0_OUT));    // Solicitar x bytes del esclavo 0
      n = 0;
      while (Wire.available()) { // slave may send less than requested
        Slave_0_OUT.B[n] = Wire.read();    // receive a byte as character
        n++;
      }
      for (int i = 0; i < sizeof(Slave_0_OUT); i++){
        Raw_OUT.B[i]=Slave_0_OUT.B[i];
      }

      ///////////////// Comunicacion con SLAVE 1 ///////////////// - Control de Motores Motrízes
      Wire.beginTransmission(SALVE_1_ADDR); // transmit to device SLAVE_1
      for (int i = 0; i < sizeof(Slave_1_IN); i++) {
        Wire.write(Slave_1_IN.B[i]);
      }
      Wire.endTransmission();    // stop transmitting


      ///////////////// Comunicacion con SLAVE 2 /////////////////
      Wire.beginTransmission(SALVE_2_ADDR); // transmit to device SLAVE_2
      for (int i = 0; i < sizeof(Slave_2_IN); i++) {
        Wire.write(Slave_2_IN.B[i]);
      }
      Wire.endTransmission();    // stop transmitting

      Wire.requestFrom(SALVE_2_ADDR, sizeof(Slave_2_OUT));    // Solicitar x bytes del esclavo 2
      n = 0;
      while (Wire.available()) { // slave may send less than requested
        Slave_2_OUT.B[n] = Wire.read();    // receive a byte as character
        n++;
      }
      for (int i = 0; i < sizeof(Slave_2_OUT); i++){
        Raw_OUT.B[i+32]=Slave_2_OUT.B[i];
      }
    }
  

}
