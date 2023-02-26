#pragma once

#ifdef __cplusplus
extern "C"{
#endif

#include "cmsis_os.h"

extern osThreadId soundTaskHandle;
extern osThreadId mortorTaskHandle;
extern osThreadId uartTaskHandle;
extern osThreadId IRSensorTaskHandle;
extern osThreadId IMUTaskHandle;
extern osThreadId pathTaskHandle;

#ifdef __cplusplus
}
#endif

void StartSoundTask(void const *argument);
void StartMortorTask(void const *argument);
void StartUartTask(void const *argument);
void StartIRSensorTask(void const *argument);
void StartIMUTask(void const *argument);
void PathTask(void const *argument);

#define prescale_exec(code, prescale, st_uq_value)                             \
  static uint8_t st_uq_value = 0;                                              \
  if (st_uq_value++ % prescale == 0) {                                         \
    code;                                                                      \
  }
