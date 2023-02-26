#include"UartBuffer.h"
#include"Debug.h"

UartBuffer unique_ubuffer;

void initializeUartBuffer(){
    unique_ubuffer.head = 0;
    unique_ubuffer.tail = 0;
}


void UartBufferUpdate(){

    while(unique_ubuffer.head != unique_ubuffer.tail){
        uart2_putc(unique_ubuffer.buffer[unique_ubuffer.tail]);
        unique_ubuffer.tail++;
    }
}


void UartBufferPush(uint8_t c){
    
    unique_ubuffer.buffer[unique_ubuffer.head] = c;
    unique_ubuffer.head++;
}