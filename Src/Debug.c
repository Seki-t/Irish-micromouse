#include"Debug.h"
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_tx;

uint8_t uart_dma_data[255];

void uart2_putc(uint8_t c){
    uint8_t s[1] = {c};

    HAL_UART_Transmit(&huart2,s, sizeof(s),100);
}


void mortorDebug(EdgeButton* button, PWMMortor* mortorL, PWMMortor* mortorR){

    if(EdgeButtonGetState(button) == 1){
        g_debug_outL += 10;
        g_debug_outR += 10;
    }

    MortorOutput(mortorL, g_debug_outL);
    MortorOutput(mortorR, g_debug_outR);
}


void mortorDebugReset(PWMMortor* mortorL, PWMMortor* mortorR){
    g_debug_outL = 0;
    g_debug_outR = 0;
    MortorOutput(mortorL, g_debug_outL);
    MortorOutput(mortorR, g_debug_outR);
}