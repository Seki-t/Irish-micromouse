#include"Encoder.h"
#include"GlobalParameter.h"
#include"stm32f4xx_hal.h"


void EncoderInitialize(MortorEncoder* enc,EncoderKind kind, float r, int32_t dir){
    enc->kind = kind;  
    enc->dir = dir;
    enc->speed = 0;
    enc->r = r;
    enc->value = 0;
    enc->movement = 0;
}

void EncoderUpdate(MortorEncoder* enc){

    //critical section ?
    int32_t now_rotation = 0;
    switch(enc->kind){
        case TIM3_ENCODER:
        now_rotation = TIM3->CNT;
        TIM3->CNT = 0;
        break;
        case TIM4_ENCODER:
        now_rotation = TIM4->CNT;
        TIM4->CNT = 0;
        break;
        default:
        now_rotation = 0;
        break;
    }

    if(now_rotation > 32768){
        now_rotation = now_rotation - 65536;
    }
    enc->value = now_rotation * enc->dir;
    enc->movement = GEAR_RATIO * enc->r * (float)(enc->value) * 2.0f * MY_PI / (float)(ENCODER_ROTATION_PULSE_AMOUNT);
    //enc->speed = enc->r * ENCODER_FS * (float)(enc->value) * 2.0f * MY_PI / ENCODER_ROTATION_PULSE_AMOUNT;
    enc->speed = ENCODER_FS * enc->movement;
}


float EncoderGetSpeed(MortorEncoder* enc){
    return enc->speed;
}

float EncoderGetRotation(MortorEncoder* enc){
    return enc->movement;
}
int32_t EncoderGetValue(MortorEncoder* enc){
    return enc->value;
}
