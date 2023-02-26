#include"AttitudeControl.h"
#include "PIDController.h"

float e_pre = 0.0f;

float AttitudeControl(float r_theta, float y_theta){
    float e_theta = r_theta - y_theta;
    float e_diff = e_theta - e_pre ;
    e_pre = e_theta;

   return ( 0.5f * e_theta + 0.0f * e_diff);
}