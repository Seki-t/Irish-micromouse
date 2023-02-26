#pragma once

#ifdef __cplusplus
 extern "C" {
#endif

#include"Motor.h"
#include"Encoder.h"
#include"PIDController.h"


typedef struct{
    PWMMortor mortor;
    MortorEncoder* encoder;
    PIDParams pid;
    float speed_reference;    
}EncoderedMortor;


void EncoderedMortorInitialize(EncoderedMortor* enc_mortor,MortorEncoder* encoder, PIDParams pid, MortorId id, int32_t dir);

void EncoderedMortorUpdate(EncoderedMortor* enc_mortor);

void EncoderedMortorSetReference(EncoderedMortor* enc_mortor, float speed);


#ifdef __cplusplus
}
#endif