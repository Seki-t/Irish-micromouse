#pragma once

#ifdef __cplusplus
 extern "C" {
#endif

#include"EdgeButton.h"
#include"GlobalObjects.h"
#include"Motor.h"
#include "xprintf.h"
#include"stm32f4xx_hal.h"
#include<stdint.h>

extern uint8_t uart_dma_data[255];

void uart2_putc(uint8_t c);

void mortorDebug(EdgeButton* button, PWMMortor* mortorL, PWMMortor* mortorR);

void mortorDebugReset(PWMMortor* mortorL, PWMMortor* mortorR);

#ifdef __cplusplus
}
#endif
