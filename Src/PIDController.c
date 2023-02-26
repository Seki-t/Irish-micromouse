#include "PIDController.h"


void PIDInitialize(PIDParams* pid, float p,float i,float d,float Ts,float Td){
    pid->P = p;
    pid->I = i;
    pid->D = d;
    pid->Ts = Ts;
    pid->Td  = Td;
    pid->e = 0;
    pid->err_sum = 0;
}

float PIDOutput(PIDParams* pid, float r, float y){
    float e = r - y;

    pid->err_sum += (e + pid->e) * 0.5f;

    float out =  pid->P * e + pid->I * pid->err_sum + pid->D * (pid->e - e);

    pid->e = e;

    return out;
}

void PIDReset(PIDParams* pid){
    pid->err_sum = 0.0f;
}