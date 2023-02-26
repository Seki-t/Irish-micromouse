#pragma once

#include<math.h>
#include<stdint.h>
#include"xprintf.h"
#include"IrSensor.h"
#pragma once
#include"IrSensor.h"
#include"xprintf.h"

extern const float cos_arr[];
extern const float sin_arr[];

extern float fourie_results[];

float* caluculateFourie(uint16_t* dma_buffer);
