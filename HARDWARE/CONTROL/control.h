#ifndef CONTROL_H
#define CONTROL_H
#include <stm32f4xx.h>

//使能驱动1
#define LIN1 PFout(0)
#define LIN2 PFout(1)//vo
#define LIN3 PFout(2)
#define LIN4 PFout(3)//vi
//使能驱动2
#define RIN1 PFout(4)
#define RIN2 PFout(5)//li
#define RIN3 PFout(6)
#define RIN4 PFout(7)//lo

//电机位置
#define VerticalIn      3
#define VerticalOut     4
#define LevelOut        2
#define LevelIn         1
#define StopAll         0

//方便调试，定义电机方位
#define Motor           TIM8
#define MVerticalIn     CCR3    //垂直方向内侧电机,M--电机前缀，Vertical--电机方向垂直方向，Out--电机位置外侧电机
#define MVerticalOut    CCR4    //垂直方向外侧电机
#define MLevelOut       CCR2    //水平方向外侧电机
#define MLevelIn        CCR1   //水平方向内侧电机

//选择模式的宏定义
#define Task4

#ifdef Task4
#define TargetRoll  0
#define TargetPitch 0
#define TargetDis   0
#endif

//圆周率
#define PI 3.1415926
//到地面距离
#define H 0.875
//周期
#define T 1.678

//PID结构体
typedef struct PID
{
    float Kp;
    float Ki;
    float Kd;
    float (*PIDControl)(float target, float feedback,struct PID* pid);
    
};


/* 基础  */

void Motor_PWM_TIM8_Init();
void GetPolar(float roll,float pitch);



void StopAllMotor();


/* 第一项  */

float PidControl_LineMove(float target, float feedback,struct PID* pid);
void Task1_LineMove(float R,uint8_t forward);

/* 第二项  */


/* 第三项  */
extern uint8_t State_Data;

void Task3_AngleMove(float angle,float R);
float (*T3State_Update(float angle,float R,float roll,float pitch))[5];
void T3Motor_CmdCombination(float Vo,float Lo,uint8_t a);

/* 第四项  */

float PidControl_Stop(float target, float feedback,struct PID* pid);
void Task4_StopFast();
void PWM_Allocation(float Output);
void MotorState(float pitch,float roll);


/* 第五项  */
void Task5_CircleMove(float R);
void T5Motor_CmdCombination(int sit,float Vo,float Lo);
void T5_2Motor_CmdCombination(int sit,float Vo,float Lo);
float (*T5LowPassFilter(float Vinput,float Linput,float a))[3];

#endif 