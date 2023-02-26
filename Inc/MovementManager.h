#pragma once

#ifdef __cplusplus
 extern "C" {
#endif
#include"Measurement.h"
#include"Motor.h"
#include"EncoderedMortor.h"

typedef struct {
    Measurement* met;
    EncoderedMortor* left_mortor;
    EncoderedMortor* right_mortor;
    float V,Omega,L;
}Movement;

typedef struct{
    float V;
    float W;
}Velocity;


void MovementManagerSetReference(Movement* movement, float V, float Omega);

void MovementManagerUpdate(Movement* movement);

void MovementManagerInitialize(Movement* movement, Measurement* met, EncoderedMortor* l_m, EncoderedMortor* r_m, float L);
#ifdef __cplusplus
}
#endif