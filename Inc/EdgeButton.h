#pragma once

#ifdef __cplusplus
extern "C"{
#endif

#include<stdint.h>

typedef struct{
    int32_t state;
    int32_t bk_state;
}EdgeButton;


void EdgeButtonInitialize(EdgeButton* button);

void EdgeButtonUpdate(EdgeButton* button, int32_t value);

int32_t EdgeButtonGetState(EdgeButton* button);

#ifdef __cplusplus
}
#endif