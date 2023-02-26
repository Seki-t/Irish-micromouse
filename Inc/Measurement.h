#pragma once

#include"Encoder.h"
#include"IMU.h"

/*
Encoderは制御にも計測にも使うので、計測側で所持はできない。関連関係になる。
*/

//前方方向 がY正方向,右方向がX正方向,Y軸から右回りが回転角の正方向

typedef struct{
    float x;
    float y;
    float theta;
}MortionParams;

typedef struct{
    MortionParams position;
    MortionParams velocity;

    MortorEncoder* encL;
    MortorEncoder* encR;

    float L;

}Measurement;


void MortionParamsSet(MortionParams* met, float x,float y, float t);

void MeasurementInitialize(Measurement* met, MortorEncoder* el, MortorEncoder* er,float L, MortionParams start_pos);

void MeasurementUpdate(Measurement* met);

MortionParams MeasurementGetPosition(Measurement* met);
void MeasurementSetPosition(Measurement* met, MortionParams pos);

