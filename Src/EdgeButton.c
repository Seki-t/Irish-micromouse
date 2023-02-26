#include"EdgeButton.h"
#include<stdint.h>

void EdgeButtonInitialize(EdgeButton* button){
    button->state = 0;
    button->bk_state = 0;
}

void EdgeButtonUpdate(EdgeButton* button, int32_t value){
    button->state = 0;
    if(button->bk_state == 0 && value == 1){
        button->state = 1;
    }
    button->bk_state = value;
}

int32_t EdgeButtonGetState(EdgeButton* button){
    return button->state;
}
