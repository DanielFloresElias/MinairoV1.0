#include <Arduino.h>
#include "invKinematic.h"

invKinematic::invKinematic(float RobotRadius, float WheelRadius)
{
    _RobRad = RobotRadius;
    _WheelRad = WheelRadius;
}

motor_speed invKinematic::calc(comando_speed Comand)
{
    _MSpeed.angular[0] = (-_cos30*Comand.linear[0] + 0.5*Comand.linear[1] + _RobRad*Comand.angular[2]) / _WheelRad;
    _MSpeed.angular[1] = (_cos30*Comand.linear[0] + 0.5*Comand.linear[1] + _RobRad*Comand.angular[2]) / _WheelRad;
    _MSpeed.angular[2] = (-Comand.linear[1] + _RobRad*Comand.angular[2]) / _WheelRad;
    return (_MSpeed);
}
