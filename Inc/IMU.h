#pragma once
#ifdef __cplusplus
 extern "C" {
#endif
#include"stm32f4xx_hal.h"
#include<stdint.h>

typedef enum{
    IMU_STATE_KEYON,
    IMU_STATE_CALIBRATION,
    IMU_STATE_NORMAL,
    IMU_STATE_BAD
}IMUState;

typedef struct{
    float x, y, z;
}Attitude;

typedef struct{
    Attitude theta;
    Attitude omega;
    Attitude acc;
   
    //calibration value
    float ox_offset;
    float oy_offset;
    float oz_offset;

}GyroState;

void IMUUpdate();
void IMUInitialize();
void IMUStartCalibration();
int8_t IMUIsReadEnable();

Attitude IMUGetAllAttitude();
Attitude IMUGetAllOmega();

float IMUGetAttitude();
float IMUGetAngularVelocity();
#ifdef __cplusplus
}
#endif