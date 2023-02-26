#include"GlobalObjects.h"


Measurement measurement;
Movement movement;

MortorEncoder right_encoder;
MortorEncoder left_encoder;


int32_t g_debug_outL = 0, g_debug_outR = 0; 
int32_t g_debug_x = 0;
int32_t g_debug_y = 0;
int32_t g_debug_theta = 0;

int8_t g_action_direction = 'L';

float g_mr_v = 0;

int32_t g_pwm_outL = 0;
int32_t g_pwm_outR = 0;

int32_t g_global_tick = 0;