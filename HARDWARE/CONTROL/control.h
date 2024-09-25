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
#define VerticalIn      1
#define VerticalOut     2
#define LevelOut        3
#define LevelIn         4
#define StopAll         0

//电机制动使能
#define ControlMotor(x,y) 


/* 基础  */

void Motor_PWM_TIM8_Init();
void MotorState(float pitch,float roll);
void StopAllMotor();
void GetDef(float roll,float pitch);

/* 第一项  */



/* 第二项  */


/* 第三项  */


/* 第四项  */

float PidControl_Stop(float target, float feedback);
void Task4_StopFast();


#endif 