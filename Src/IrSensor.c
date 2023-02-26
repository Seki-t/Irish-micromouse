#include"IrSensor.h"
#include"xprintf.h"
#include"Fourie.h"

extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;
extern ADC_HandleTypeDef hadc2;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim5;

IRSensorInfo ir_sensor_info;

uint16_t irSensor_dma_buffer[4 * PULSE_NUM * SUMPLE_FOR_PULSE];

int32_t ir_sensor_can_read = 0;

IRSensorInfo info_buffer[4];

//基準クロックが168/2 = 84Mhz. プリスケーラ = 83, カウンタ上限 = 1000なので、PWM周期が1000 * 1 / (84*10^6 / 84) = 10^(-3)
//この1kHzの波を、１パルスあたり20回で４パルスの計80点サンプリングする
//つまり計測には約4msかかる
void adcInit(){
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 499);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 499);

    ir_sensor_info.left_side = 0;
    ir_sensor_info.left_front = 0;
    ir_sensor_info.right_side = 0;
    ir_sensor_info.right_front = 0;

    for (int i = 0; i < 4; i++) {
      info_buffer[i].left_side = 0;
      info_buffer[i].left_front = 0;
      info_buffer[i].right_side = 0;
      info_buffer[i].right_front = 0;
    }

    HAL_TIM_PWM_Stop(&htim5,TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim5,TIM_CHANNEL_2);
}

void adcStart(){
    HAL_TIM_Base_Start(&htim2);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
    HAL_ADC_Start_DMA(&hadc1, irSensor_dma_buffer, 4 * PULSE_NUM * SUMPLE_FOR_PULSE);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
    if(hadc == &hadc1){
        HAL_ADC_Stop_DMA(hadc);
        HAL_TIM_PWM_Stop(&htim5,TIM_CHANNEL_1);
        HAL_TIM_PWM_Stop(&htim5,TIM_CHANNEL_2);
        //__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 0);
        //__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 0);

        float* spectols = caluculateFourie(irSensor_dma_buffer);
        //xprintf("sp = %d,%d,%d,%d\r\n",(int32_t)spectols[0],(int32_t)spectols[3],(int32_t)spectols[2],(int32_t)spectols[1]);

        //こうすると read中に上書きされることを防げる
        //if(ir_sensor_can_read)return;

        ir_sensor_info.left_side = (int32_t)spectols[0];
        ir_sensor_info.left_front = (int32_t)spectols[3];
        ir_sensor_info.right_front = (int32_t)spectols[1];
        ir_sensor_info.right_side = (int32_t)spectols[2];
        
    //    ir_sensor_info.is_wall_ls = ir_sensor_info.left_side > 20000 ? 1 : 0;
    //    ir_sensor_info.is_wall_lf = ir_sensor_info.left_front > 20000 ? 1 : 0;
    //    ir_sensor_info.is_wall_rs = ir_sensor_info.right_side > 20000 ? 1 : 0;
    //    ir_sensor_info.is_wall_rf = ir_sensor_info.right_front > 20000 ? 1 : 0;
        
        //for(int j = 0; j < PULSE_NUM * SUMPLE_FOR_PULSE; j++){
        //	for(int i = 0 + 4 * j; i < 4 * (j + 1); i++){
        //		xprintf("%d,", irSensor_dma_buffer[i]);
        //	}
        //	xprintf("\r\n");
        //}
    }
}

void adcStop(){
    HAL_ADC_Stop_DMA(&hadc1);
    HAL_TIM_Base_Stop(&htim2);
    HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_2);
}

int32_t isIRSensorUpdate(){
    return ir_sensor_can_read;
}

IRSensorInfo getIRSensorInfo(){
    //割り込み防ぐために先に保存しておく
    //IRSensorInfo irs = ir_sensor_info;
    ir_sensor_can_read = 0;
    return ir_sensor_info;
}

//irSensor値にローパスをかける
void IRSensorBufferUpdate(){
    IRSensorInfo tmp_info = ir_sensor_info;

    float ls_sum = tmp_info.left_side;
    float lf_sum = tmp_info.left_front;
    float rs_sum = tmp_info.right_side;
    float rf_sum = tmp_info.right_front;

    for(int i = 0; i < 4; i++){
        ls_sum += info_buffer[i].left_side;
        lf_sum += info_buffer[i].left_front;
        rs_sum += info_buffer[i].right_side;
        rf_sum += info_buffer[i].right_front;
    }

    ir_sensor_info.left_side = ls_sum / 5; 
    ir_sensor_info.left_front = lf_sum / 5; 
    ir_sensor_info.right_side = rs_sum / 5; 
    ir_sensor_info.right_front = rf_sum / 5; 
    
    ir_sensor_info.is_wall_ls = ir_sensor_info.left_side > 18000 ? 1 : 0;
    ir_sensor_info.is_wall_lf = ir_sensor_info.left_front > 30000 ? 1 : 0;
    ir_sensor_info.is_wall_rs = ir_sensor_info.right_side > 18000 ? 1 : 0;
    ir_sensor_info.is_wall_rf = ir_sensor_info.right_front > 30000 ? 1 : 0;
    
    for(int i = 0 ; i < 3; i++){
        info_buffer[i] = info_buffer[i+1];
    }
    info_buffer[3] = tmp_info;
    ir_sensor_can_read = 1;
}
