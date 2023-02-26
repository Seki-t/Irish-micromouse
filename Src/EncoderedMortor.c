#include"EncoderedMortor.h"


void EncoderedMortorInitialize(EncoderedMortor* enc_mortor,MortorEncoder* encoder, PIDParams pid, MortorId id, int32_t dir){
    enc_mortor->encoder = encoder;
    enc_mortor->pid = pid;
    enc_mortor->speed_reference = 0.0f;
    MortorInitialize(&enc_mortor->mortor, id, dir);
}

void EncoderedMortorUpdate(EncoderedMortor* enc_mortor){

    float now_speed = EncoderGetSpeed(enc_mortor->encoder);

    float u = PIDOutput( &enc_mortor->pid, enc_mortor->speed_reference, now_speed);
    MortorOutput(&enc_mortor->mortor,u);
}

void EncoderedMortorSetReference(EncoderedMortor* enc_mortor, float speed){
    enc_mortor->speed_reference = speed;
}