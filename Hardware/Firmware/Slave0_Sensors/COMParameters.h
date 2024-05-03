typedef union{
  byte B[12];
  struct{
      float x;                // m/s
      float y;                // m/s
      float w;                // rad/s
  } INPUT_DATA;
}_SLAVE1_IN;

typedef union{
  byte B[8];
  struct{
      uint16_t CtrlWord1;     // [b0]Enable, [b1]Home,
      uint16_t CtrlWord2; 
      uint32_t SetPoint;          // ticks
  } INPUT_DATA;
}_SLAVE0_IN;

#define MotorEnableMask 0x0001
#define MotorHomeMask   0x0002

typedef union{
  byte B[32];
  struct{
      uint16_t StatusWord1;     // [b0]L_SW, [b1]H_SW,
      uint16_t StatusWord2; 
      int32_t EncoderValue;          // ticks
      uint16_t LS_0;
      uint16_t LS_1;
      uint16_t LS_2;
      uint16_t LS_3;
      uint16_t LS_4;
      uint16_t LS_5;
      uint16_t LS_6;
      uint16_t LS_7;
      uint16_t SS_0;
      uint16_t SS_1;
      uint16_t SS_2;
      uint16_t SS_3;
  } OUTPUT_DATA;
}_SLAVE0_OUT;

#define MotorHswMask  0x0001
#define MotorLswMask  0x0002

typedef union{
  byte B[12];
  struct{
      uint8_t GPIO_Config;       
      uint8_t GPIO_Value;       
      uint16_t Servo_0;       // pulse width in micro seconds
      uint16_t Servo_1;       // pulse width in micro seconds
      uint16_t Servo_2;       // pulse width in micro seconds
      uint16_t Servo_3;       // pulse width in micro seconds
      uint16_t Servo_4;       // pulse width in micro seconds
  } INPUT_DATA;
}_SLAVE2_IN;

typedef union{
  byte B[14];
  struct{
      uint8_t GPIO_Status;  
      uint16_t AN_0;       
      uint16_t AN_1;       
      uint16_t AN_2;       
      uint16_t AN_3;
      uint32_t Sonar;     
  } OUTPUT_DATA;
}_SLAVE2_OUT;

#define GPIO_0_Mask B00000001
#define GPIO_1_Mask B00000010
#define GPIO_2_Mask B00000100
#define GPIO_3_Mask B00001000
#define GPIO_4_Mask B00010000
#define GPIO_5_Mask B00100000
#define GPIO_6_Mask B01000000
