#pragma once
#include"stm32f4xx_hal.h"
#include<stdint.h>

typedef struct{
    float P;
    float I;
    float D;
    float Ts;
    float Td;
    float e;
    float err_sum;
}PIDParams;

float PIDOutput(PIDParams* pid,float r,float y);

void PIDInitialize(PIDParams* pid, float p,float i,float d,float Ts,float Td);

void PIDReset(PIDParams* pid);