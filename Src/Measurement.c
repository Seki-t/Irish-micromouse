#include"Motor.h"
#include"Measurement.h"
#include"IMU.h"
#include"stm32f4xx_hal.h"
#include"GlobalParameter.h"
#include"IMU.h"
#include<math.h>


void MeasurementInitialize(Measurement* met, MortorEncoder* el, MortorEncoder* er,float _L, MortionParams start_pos){
    met->encL = el;
    met->encR = er;
    met->L = _L;
    MortionParamsSet(&met->position, start_pos.x, start_pos.y, start_pos.theta);
    MortionParamsSet(&met->velocity, 0, 0, 0);
}

void MortionParamsSet(MortionParams* param, float x,float y, float t){
    param->x = x;
    param->y = y;
    param->theta = t;
}

void MeasurementUpdate(Measurement* met){

    EncoderUpdate(met->encL);
    EncoderUpdate(met->encR);

    float velocity_left = EncoderGetSpeed(met->encL);
    float velocity_right = EncoderGetSpeed(met->encR);

    float V = (velocity_left + velocity_right) / 2.0f;
    float W = (velocity_left - velocity_right) / met->L;    // ((v1 - v2) / 2.0f) / (L / 2.0f)

    //直線近似・横滑り考慮なしの場合の空間座標系への速度変換 
    float v_x = V * sinf(met->position.theta);
    float v_y = V * cosf(met->position.theta);

    met->position.x += 0.5f * (met->velocity.x + v_x) * MEASUREMENT_TS;
    met->position.y += 0.5f * (met->velocity.y + v_y) * MEASUREMENT_TS;
    met->position.theta += 0.5f * (met->velocity.theta + W) * MEASUREMENT_TS;
    
    //直線近似・横滑り考慮なしの場合の空間座標系への速度変換 
    met->velocity.x = v_x;
    met->velocity.y = v_y;
    met->velocity.theta = W;

    if(IMUIsReadEnable()){
        //met->position.theta = IMUGetAttitude();
    }
/*
    const float pi2 = 2.0f * MY_PI;
    if(met->position.theta > pi2){
        met->position.theta -= pi2;
    }

    if(met->position.theta < -pi2){
        met->position.theta += pi2;
    }
*/
}

MortionParams MeasurementGetPosition(Measurement* met){
	return met->position;
}

void MeasurementSetPosition(Measurement* met, MortionParams pos){
	met->position = pos;
}

