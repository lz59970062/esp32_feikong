#ifndef __GETHIGHT_H
#define __GETHIGHT_H
#ifdef PRESS
#include<BMP280_DEV.h>
#endif
void gethigh_init();
void gethight(float* hight);
void gethight(float *hight, float *d_value);

typedef struct 
{
    float LastP;//上次估算协方差 初始化值为0.02
    float Now_P;//当前估算协方差 初始化值为0
    float out;//卡尔曼滤波器输出 初始化值为0
    float Kg;//卡尔曼增益 初始化值为0
    float Q;//过程噪声协方差 初始化值为0.001
    float R;//观测噪声协方差 初始化值为0.543
}KFP;
float kalmanFilter(KFP *kfp,float input);
//2. 以高度为例 定义卡尔曼结构体并初始化参数

#endif 
