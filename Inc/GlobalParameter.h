#pragma once

#define TIER_RADIUS 12.0f //[mm]

#define ENCODER_TS 0.01f
#define ENCODER_FS (1.0f / (ENCODER_TS)) // [Hz]
#define ENCODER_ROTATION_PULSE_AMOUNT 4096  //1024 * 4 [pls / rotation] 
#define GEAR_RATIO 0.325f  //   13 / 40 = 0.325
#define MY_PI 3.141593f    // [rad]

#define DISTANCE_BETWEEN_WHEELS 81.0f // L [mm]   // W = ((v1 - v2) / 2) / (L / 2)
#define DISTANCE_BACK_TO_SHAFT 22.0f    // [mm]
#define MEASUREMENT_TS 0.01f

#define GYRO_UPDATE_PERIOD_MS 10   //[ms]
#define GYRO_UPDATE_PERIOD (GYRO_UPDATE_PERIOD_MS / 1000.0f)
#define GYRO_UPDATE_FREQ (1000 / GYRO_UPDATE_PERIOD_MS)
#define GYRO_GAIN (8.75f / 1000.0f)

#define MORTOR_CONTROL_FS 10
#define MORTOR_CONTROL_TS (1.0f / (MORTOR_CONTROL_FS))

#define MORTOR_OFFSET_VOLTAGE 0
#define ADJUST_LEFT_MORTOR 1.2f


#define MAP_X_SIZE 16
#define MAP_Y_SIZE 16

#define FIELD_X_SIZE (180 * MAP_X_SIZE)     //[mm]
#define FIELD_Y_SIZE (180 * MAP_Y_SIZE)     //[mm]

#define MAP_CHIP_SIZE 180


#define DEBUG_PRINT_ENABLE 1

#if DEBUG_PRINT_ENABLE == 1
#define DEBUG_PRINT(macro_str,...) xprintf(macro_str,__VA_ARGS__)
#else
#define DEBUG_PRINT(macro_str)
#endif