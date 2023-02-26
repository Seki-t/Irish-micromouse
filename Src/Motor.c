#include"Motor.h"
#include"GlobalParameter.h"
#include"GlobalObjects.h"
#include"stm32f4xx_hal.h"

extern TIM_HandleTypeDef htim1;

void MortorInitialize(PWMMortor* mortor,MortorId id, int32_t dir){
  mortor->id = id;
  mortor->dir = dir;
}


void MortorOutput(PWMMortor* mortor, int32_t duty){

  if(mortor->id == MORTOR_L){
    g_pwm_outL = duty;
    OutputL((duty * mortor->dir));
  }
  else{
    g_pwm_outR = duty;
    OutputR(duty * mortor->dir);
  }
}

void MortorOutputVoltage(PWMMortor* mortor, float voltage){

}

// R/L direction is right.
//duty to enc no +/- direction is right
void OutputL(int32_t duty) {

  if (duty < 0)
  {
    int32_t adjusted_duty = duty - MORTOR_OFFSET_VOLTAGE;
    
    if (adjusted_duty < -999) adjusted_duty = -999;

    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, -adjusted_duty);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  }
  else if (duty > 0)
  {
    int32_t adjusted_duty = duty + MORTOR_OFFSET_VOLTAGE;

    if (adjusted_duty > 999) adjusted_duty = 999;

    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, adjusted_duty);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  }
  else
  {
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    // HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    // HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
  }
}

// R/L direction is right.
//duty to enc no +/- direction is right
void OutputR(int32_t duty) {

  if (duty < 0) {
    if(duty < -999) duty = -999;
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, -duty);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  } else if (duty > 0) {
    if(duty > 999) duty = 999;
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, duty);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  } else {
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
    //HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
    //HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);
  }
}

