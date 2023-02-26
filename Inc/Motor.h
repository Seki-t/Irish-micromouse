#pragma once
#include<stdint.h>
/*
 * pwm1はデッドタイム生成のために相補的なPWMを出力している。
 * これはDRV8835に必要なの？調べたい
 * MotorPWMはそれ用の出力関数があるはずなのでAPI表で調べる
 */

/*
 * 今回はMODE = GNDのためdrv8835のIN/IN modeを使ってる.
 * よって、AIN1に1を流すと正回転、AIN2に１を流すと逆回転となっている。
 * ゆえに、それぞれのPWMを独立にduty決めて出力してやればいい。
 * 今の設定でそれができるのかは、試せば分かる、または調べれば分かる。
 *
 * 今は相補的に出すようになっており、PWM1の中でdutyを決めるのは1箇所sConfigOC.Pulseのみ
 * 相補的が上手く動いてるなら、上手くいくかも？どう動くのか調べよう
 */

typedef enum{
    MORTOR_L,
    MORTOR_R
}MortorId;

typedef struct{
    MortorId id;
    int8_t dir;

}PWMMortor;

void MortorInitialize(PWMMortor* mortor,MortorId id, int32_t dir);
void MortorOutput(PWMMortor* mortor, int32_t duty);
void MortorOutputVoltage(PWMMortor* mortor, float voltage);

void OutputR(int32_t duty);
void OutputL(int32_t duty);
