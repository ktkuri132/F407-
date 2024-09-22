/*

这个文件  实现了  PID控制计算，PWM输出  是风力摆控制中调用的主要函数

    水平方向 pitch轴  ---向外 + 向内 -
    前后方向 roll轴  ---向外 - 向内 +
    旋转方向 yaw轴（未使用）

*/


#include "stdio.h"
#include <stm32f4xx.h>
#include <bsp.h>
#include <sys.h>


extern float LSKp, LSKi, LSKd;
extern float SWKp, SWKi, SWKd;


/// @brief PID直线控制函数
/// @param target 目标值
/// @param feedback 当前值
/// @return 计算值（浮点型，有正负）
float PidControl_LineState(float target, float feedback)
{

    // 定义误差变量
    static float LSerror = 0;
    static float LSlastError = 0;
    static float LSintegral = 0;
    static float LSderivative = 0;

    // 定义积分限幅变量
    static float LSintegralMin = 0;
    static float LSintegralMax = 0;

    // 定义PID输出变量
    static float output = 0;

    // 计算误差
    LSerror = target - feedback;

    // 计算积分项
    LSintegral += LSerror;

    // 限制积分项在积分限幅范围内
    if (LSintegral < LSintegralMin) {
        LSintegral = LSintegralMin;
    } else if (LSintegral > LSintegralMax) {
        LSintegral = LSintegralMax;
    }

    // 计算微分项
    LSderivative = LSerror - LSlastError;

    // 计算PID输出
    output = LSKp * LSerror + LSKi * LSintegral + LSKd * LSderivative;

    // 更新误差变量
    LSlastError = LSerror;

    return output;
}

/// @brief PID摆动控制函数
/// @param target 目标值
/// @param feedback 当前值
/// @return 计算值（浮点型，有正负）
float PidControl_SwingState(float target, float feedback)
{

    // 定义误差变量
    static float SWerror = 0;
    static float SWlastError = 0;
    static float SWintegral = 0;
    static float SWderivative = 0;

    // 定义积分限幅变量
    static float SWintegralMin = 0;
    static float SWintegralMax = 0;

    // 定义PID输出变量
    static float output = 0;

    // 计算误差
    SWerror = target - feedback;

    // 计算积分项
    SWintegral += SWerror;

    // 限制积分项在积分限幅范围内
    if (SWintegral < SWintegralMin) {
        SWintegral = SWintegralMin;
    } else if (SWintegral > SWintegralMax) {
        SWintegral = SWintegralMax;
    }

    // 计算微分项
    SWderivative = SWerror - SWlastError;

    // 计算PID输出
    output = SWKp * SWerror + SWKi * SWintegral + SWKd * SWderivative;

    // 更新误差变量
    SWlastError = SWerror;

    return output;
}

//PWM输出，采用高级定时器TIM8，输出4路PWM
void Motor_PWM_TIM8_Init()
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

    GPIO_Config(GPIOC, GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9, GPIO_Mode_AF, GPIO_PuPd_NOPULL);

    GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM8);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM8);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_TIM8);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_TIM8);

    // 配置TIM8
    TIM_TimeBaseInitTypeDef TIM8_TimeBaseInitStruct;
    TIM8_TimeBaseInitStruct.TIM_Period = 8400-1;  
    TIM8_TimeBaseInitStruct.TIM_Prescaler = 1;  
    TIM8_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM8_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM8, &TIM8_TimeBaseInitStruct);


    // 配置TIM8的4个通道为PWM模式
    TIM_OCInitTypeDef TIM8_OCInitStruct;
    TIM8_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
    TIM8_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
    TIM8_OCInitStruct.TIM_OutputNState = TIM_OutputNState_Disable;
    TIM8_OCInitStruct.TIM_Pulse = 0;
    TIM8_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM8_OCInitStruct.TIM_OCNPolarity = TIM_OCNPolarity_High;
    TIM8_OCInitStruct.TIM_OCIdleState = TIM_OCIdleState_Reset;
    TIM8_OCInitStruct.TIM_OCNIdleState = TIM_OCNIdleState_Reset;

    TIM_OC1Init(TIM8, &TIM8_OCInitStruct);
    TIM_OC2Init(TIM8, &TIM8_OCInitStruct);
    TIM_OC3Init(TIM8, &TIM8_OCInitStruct);
    TIM_OC4Init(TIM8, &TIM8_OCInitStruct);

    TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(TIM8, TIM_OCPreload_Enable);
    TIM_OC4PreloadConfig(TIM8, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(TIM8, ENABLE);

    printf("->Motor->TIM8_OC1_OC2_OC3_OC4 Init  done\n");

    // 使能TIM8主输出
    TIM_CtrlPWMOutputs(TIM8, ENABLE);
    // 使能TIM8
    TIM_Cmd(TIM8, ENABLE);

    printf("->Motor->TIM1_TIM8 Enable  done\n");
}


//使能驱动1
#define LIN1 PFout(0)
#define LIN2 PFout(1)
#define LIN3 PFout(2)
#define LIN4 PFout(3)
//使能驱动2
#define RIN1 PFout(4)
#define RIN2 PFout(5)
#define RIN3 PFout(6)
#define RIN4 PFout(7)


/// @brief 电机使能函数
/// @param x MLevelOut 
//           MLevelIlOut
//           MVerticalOut
//           MVerticalIn
///
/// @param y 
void Motor_Enable(uint8_t x,uint8_t y)
{
    
}