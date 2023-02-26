#pragma once

#include"PIDController.h"
#include"EncoderedMortor.h"
#include"Measurement.h"
#include"MovementManager.h"

extern Measurement measurement;
extern Movement movement;

extern MortorEncoder right_encoder;
extern MortorEncoder left_encoder;

extern int32_t g_debug_outL;
extern int32_t g_debug_outR;

extern int32_t g_debug_x;
extern int32_t g_debug_y;
extern int32_t g_debug_theta;

extern int8_t g_action_direction;

extern float g_mr_v;

extern int32_t g_pwm_outL;
extern int32_t g_pwm_outR;

extern int32_t g_global_tick;
