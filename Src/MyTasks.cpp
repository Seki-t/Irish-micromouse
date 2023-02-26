#include "MyTasks.h"
extern "C"
{
#include "stm32f4xx_hal.h"
#include "Debug.h"
#include "IMU.h"
#include "Motor.h"
#include "xprintf.h"
#include "IrSensor.h"
#include "PIDController.h"
#include "Encoder.h"
#include "GlobalObjects.h"
#include "EncoderedMortor.h"
#include "GlobalParameter.h"
#include "EdgeButton.h"
#include"UartBuffer.h"

  extern ADC_HandleTypeDef hadc1;
  extern ADC_HandleTypeDef hadc2;
  extern DMA_HandleTypeDef hdma_adc1;
  extern DMA_HandleTypeDef hdma_adc2;
  extern I2C_HandleTypeDef hi2c2;
  extern SPI_HandleTypeDef hspi1;
  extern TIM_HandleTypeDef htim1;
  extern TIM_HandleTypeDef htim2;
  extern TIM_HandleTypeDef htim3;
  extern TIM_HandleTypeDef htim4;
  extern TIM_HandleTypeDef htim5;
  extern TIM_HandleTypeDef htim6;
  extern UART_HandleTypeDef huart2;
  extern DMA_HandleTypeDef hdma_usart2_tx;

  osThreadId soundTaskHandle;
  osThreadId mortorTaskHandle;
  osThreadId uartTaskHandle;
  osThreadId IRSensorTaskHandle;
  osThreadId IMUTaskHandle;
  osThreadId pathTaskHandle;
}
#include "Path.hpp"
#include "PathController.hpp"
#include "Factory.hpp"
#include "Position.hpp"
#include "ActionManager.hpp"

extern uint16_t adc2_dma_buffer[3];
IRSensorInfo k_sensor_info;
int path_index = 0;


void mortorDebugTask(EdgeButton* btn, PWMMortor* pwm_mortorL, PWMMortor* pwm_mortorR);

void StartSoundTask(void const *argument)
{

  for (;;)
  {

    osDelay(1000);
  }
}

void PathTask(void const *argument)
{

  PathController *path_con = GetPathController();
  int start_count = 0;
  for (;;)
  {

    if (start_count++ < 100)
    {
      MovementManagerSetReference(&movement, 0, 0);
    }
    else
    {

      MortionParams pos = MeasurementGetPosition(&measurement);

      if (!path_con->isGoal())
      {
        MovementReference mr =
            path_con->calcNextReference(PositionF(pos.x, pos.y, pos.theta));
        MovementManagerSetReference(&movement, mr.V, mr.W);
      }
      else
      {
        if (path_index < PATH_INDEX_MAX - 1)
        {
          path_index++;
          path_con->resetStart(path_arr[path_index],
                               PositionF(pos.x, pos.y, pos.theta));
        }
        else
        {
          MovementManagerSetReference(&movement, 0, 0);
        }
      }
    }

    osDelay(10);
  }
}

void StartMortorTask(void const *argument)
{

  float Kp = 2.0, Ki = 0.004f;
  EncoderedMortor mortorL, mortorR;
  PIDParams mortorL_pid, mortorR_pid;
  PWMMortor pwm_mortorL, pwm_mortorR;
  EdgeButton button;
  
  EdgeButtonInitialize(&button);
  
  MortorInitialize(&pwm_mortorL, MORTOR_L, 1);
  MortorInitialize(&pwm_mortorR, MORTOR_R, 1);

  PIDInitialize(&mortorL_pid, Kp, Ki, 0, 0.01, 0);
  PIDInitialize(&mortorR_pid, Kp, Ki, 0, 0.01, 0);

  EncoderInitialize(&left_encoder, TIM4_ENCODER, TIER_RADIUS, 1);
  EncoderInitialize(&right_encoder, TIM3_ENCODER, TIER_RADIUS, -1);
  
  EncoderedMortorInitialize(&mortorL, &left_encoder, mortorL_pid, MORTOR_L, 1);
  EncoderedMortorInitialize(&mortorR, &right_encoder, mortorR_pid, MORTOR_R, 1);

  PositionF robot_sp(GetRobotStartPosition());
  MortionParams rsp = {robot_sp.x, robot_sp.y, robot_sp.theta};
  
  MeasurementInitialize(&measurement, &left_encoder, &right_encoder, DISTANCE_BETWEEN_WHEELS, rsp);

  MovementManagerInitialize(&movement, &measurement, &mortorL, &mortorR, DISTANCE_BETWEEN_WHEELS);

  ActionManager *act_manager = GetActionManager();
  KizunaAI *search_ai = GetKizunaAI();

  for (;;)
  {
    //Input
    MeasurementUpdate(&measurement);
    IRSensorBufferUpdate();
    k_sensor_info = getIRSensorInfo();
    MortionParams mp = MeasurementGetPosition(&measurement);

//#define MORTOR_DEBUG_MODE
#ifdef MORTOR_DEBUG_MODE
    mortorDebugTask(&button, &pwm_mortorL, &pwm_mortorR);
#else
      //Internal Calculation
      PositionF now_pos(mp.x, mp.y, mp.theta);

      //位置情報の更新と壁情報の更新は関数としてわけた方がよい
      //それぞれの目的が独立だし、１マス移動・次の行動へ移るタイプの実装をするなら、その方がイイ。
      //もしなめらかな実装をするなら、カーブの経路を事前に作って実装しないといけない。それは難易度高い
      bool isMoved = search_ai->updateMapAndPosition(&k_sensor_info, now_pos);

      //壁をupdateしたときしかis_wall_updateedがかえってこないので、calcNextPosが実施されない
      //壁を発見したときではなく、移動したときにcalcNextPosすべき
      if (isMoved){
        search_ai->calcNextPos();
      }
      debug_pos = search_ai->getNowRefPos();
      
      act_manager->update(k_sensor_info);

      //Output    
      MovementManagerUpdate(&movement);
      
      //Debug
      Position pos = search_ai->getNowRefPos();
      g_debug_x = pos.x;
      g_debug_y = pos.y;
      g_debug_theta = pos.theta;

#endif
    adcStart();
    g_global_tick += 1;
    osDelay(10);
  }
}

void mortorDebugTask(EdgeButton* btn, PWMMortor* pwm_mortorL, PWMMortor* pwm_mortorR){

    static bool is_running = false;
    static float V = 0.0f;  // mm/s
    static float Omega = 0.0f;  // rad/s
    
    GPIO_PinState sw4_state = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3); // SW4
    
    if (sw4_state == GPIO_PIN_RESET) is_running = true;
    
    if (is_running){
      EdgeButtonUpdate(btn, sw4_state == GPIO_PIN_SET);
      if(EdgeButtonGetState(btn) == 1){
          V += 10.0f;
      }
      MovementManagerSetReference(&movement, V, Omega);
      MovementManagerUpdate(&movement);
      //mortorDebug(btn, pwm_mortorL, pwm_mortorR);
    }
    else
    {
      MovementManagerSetReference(&movement, 0.0f, 0.0f);
    }
}

void StartUartTask(void const *argument)
{

  uint16_t value = 0;
  for (;;)
  {
    // xprintf("i = %d\r\n",value);
    value++;
    osDelay(1000);
  }
}

void StartIRSensorTask(void const *argument)
{

  static int vb_start_flag = 0;

  initializeUartBuffer();
  for (;;)
  {
    if (vb_start_flag++ < 3)
    {
      DEBUG_PRINT("V,%d\r\n", (int32_t)(adc2_dma_buffer[2] * 2.57875f)); // 2.57875 = 3300 * 32 / 10 / 4095
    }
    
    //HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_8B_R, 0);
    MortionParams pos = MeasurementGetPosition(&measurement);
    Position now_pos_i;
    now_pos_i.x = (int)(MAP_X_SIZE * (pos.x / FIELD_X_SIZE));
    now_pos_i.y = (int)(MAP_Y_SIZE * (pos.y / FIELD_Y_SIZE));
    now_pos_i.theta = int(180.0f * (pos.theta / MY_PI));
    float theta = MY_PI * IMUGetAttitude() / 180.0f;

    //wall sensor debug 
    //DEBUG_PRINT("S,%d,%d,%d,%d\r\n",k_sensor_info.left_side / 100,k_sensor_info.left_front / 100,k_sensor_info.right_front / 100,k_sensor_info.right_side / 100);


    //action task Debug 
    DEBUG_PRINT("D,%c:",g_action_direction);
    UartBufferUpdate();
    //DEBUG_PRINT("D,%d\r\n", path_index);
    
    //Float Position Debug
    //DEBUG_PRINT("X,%d,%d,%d\r\n",(int32_t)pos.x,(int32_t)pos.y,(int32_t)(180.0f * pos.theta / MY_PI));
    //DEBUG_PRINT(":X,%d,%d,%d,", debug_pos.x, debug_pos.y, debug_pos.theta);
    //DEBUG_PRINT("I,%d,",(int32_t)(now_pos_i.theta) );

    //box position debug 
    //DEBUG_PRINT("X,%d,%d,%d,", now_pos_i.x, now_pos_i.y, now_pos_i.theta);
    
    //Velocity Debug 
    DEBUG_PRINT(",M,%d,",(int32_t)(movement.V) );
    DEBUG_PRINT("%d,",(int32_t)(movement.Omega * 100.0f) );
    //DEBUG_PRINT("U,%d,%d\r\n",(int32_t)(debug_val_u_left*100),(int32_t)(debug_val_u_right*100) );
    DEBUG_PRINT("U,%d,%d\r\n",(int32_t)(g_pwm_outL),(int32_t)(g_pwm_outR) );
    //DEBUG_PRINT("E,%d,%d\r\n",(int32_t)EncoderGetSpeed(&left_encoder),(int32_t)EncoderGetSpeed(&right_encoder));

#ifdef MORTOR_DEBUG_MODE
    //DEBUG_PRINT("L,%d,%d,",g_debug_outL,(int32_t)EncoderGetSpeed(&left_encoder));
    //DEBUG_PRINT("%d,%d,R\r\n",g_debug_outR,(int32_t)EncoderGetSpeed(&right_encoder));
#endif

    osDelay(100);
  }
}

void StartIMUTask(void const *argument)
{
  static int32_t vb_state = 0;
  for (;;)
  {

    int32_t battery_volt = adc2_dma_buffer[2] * 2.57875f;
    if (battery_volt < 7800)
    {
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, (vb_state++ % 50 == 1) ? GPIO_PIN_SET : GPIO_PIN_RESET); // LED2
    }
    IMUUpdate();
    osDelay(GYRO_UPDATE_PERIOD_MS);
  }
}
