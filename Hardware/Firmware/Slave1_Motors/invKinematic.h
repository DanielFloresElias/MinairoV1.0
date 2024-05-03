#ifndef invKinematic_h
#define invKinematic_h

#include <Arduino.h>

typedef struct{
    float linear[3];
    float angular[3];
}comando_speed;

typedef struct{
    float angular[3];
}motor_speed;

class invKinematic
{
    public:
        invKinematic(float RobotRadius, float WheelRadius);
        motor_speed calc(comando_speed Comand);
    private:
        float _RobRad;
        float _WheelRad;
        motor_speed _MSpeed;
        const float _cos30 = 0.86602540378443864676372317075294;
};

#endif