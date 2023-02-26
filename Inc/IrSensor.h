#pragma once
#include"stm32f4xx_hal.h"
#include<stdint.h>

#define PULSE_NUM 4
#define SUMPLE_FOR_PULSE 20

typedef struct{
    int32_t left_side;
    int32_t left_front;
    int32_t right_side;
    int32_t right_front;
    uint8_t is_wall_ls;
    uint8_t is_wall_lf;
    uint8_t is_wall_rs;
    uint8_t is_wall_rf;
}IRSensorInfo;

void adcStart();
void adcInit();
void adcStop();

int32_t isIRSensorUpdate();

IRSensorInfo getIRSensorInfo();

//irSensor値にローパスをかけるためのバッファを更新する(一定周期で呼ぶ。getIRSensorInfoの前に呼ぶ)
void IRSensorBufferUpdate();