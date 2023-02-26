#include"MovementManager.h"


void MovementManagerInitialize(Movement* movement,Measurement* met, EncoderedMortor* l_m, EncoderedMortor* r_m, float L){
    movement->met = met;
    movement->left_mortor =  l_m;
    movement->right_mortor =  r_m;
    movement->L = L;
    movement->V = 0.0f;
    movement->Omega = 0.0f;
}

void MovementManagerSetReference(Movement* movement, float V, float Omega){
    movement->V = V;
    movement->Omega = Omega;
}

void MovementManagerUpdate(Movement* movement){

    const float v_left = movement->V + movement->L * movement->Omega;
    const float v_right = movement->V - movement->L * movement->Omega;

    EncoderedMortorSetReference(movement->left_mortor, v_left);
    EncoderedMortorSetReference(movement->right_mortor, v_right);

    EncoderedMortorUpdate(movement->left_mortor);
    EncoderedMortorUpdate(movement->right_mortor);
}