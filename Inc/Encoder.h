#pragma once


#ifdef __cplusplus
 extern "C" {
#endif

#include<stdint.h>

typedef enum{
    TIM3_ENCODER,
    TIM4_ENCODER
}EncoderKind;

typedef struct{
    EncoderKind kind;
    int32_t dir;
    int32_t value;
    float speed;
    float r;
    float movement;
}MortorEncoder;

void EncoderInitialize(MortorEncoder* enc,EncoderKind kind, float r, int32_t dir);

void EncoderUpdate(MortorEncoder* enc);

float EncoderGetSpeed(MortorEncoder* enc);

float EncoderGetRotation(MortorEncoder* enc);

int32_t EncoderGetValue(MortorEncoder* enc);

#ifdef __cplusplus
}
#endif
