#pragma once
#include<stdint.h>


typedef struct{
    uint8_t buffer[255];
    uint8_t head;
    uint8_t tail;
}UartBuffer;

extern UartBuffer unique_ubuffer;

void initializeUartBuffer();

void UartBufferUpdate();

void UartBufferPush(uint8_t c);

