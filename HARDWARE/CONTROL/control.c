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
#include <control.h>

//T--任务，4--第四项，S--停止，制动
extern float T4SKp, T4SKi, T4SKd;


/// @brief PID控制制动
/// @param target 目标值
/// @param feedback 当前值
/// @return 计算值（浮点型，有正负）
float PidControl_Stop(float target, float feedback)
{

    // 定义误差变量
    static float T4Serror = 0;
    static float T4SlastError = 0;
    static float T4Sintegral = 0;
    static float T4Sderivative = 0;

    // 定义积分限幅变量
    static float T4SintegralMin = 0;
    static float T4SintegralMax = 0;

    // 定义PID输出变量
    static float output = 0;

    // 计算误差
    T4Serror = target - feedback;

    // 计算积分项
    T4Sintegral += T4Serror;

    // 限制积分项在积分限幅范围内
    if (T4Sintegral < T4SintegralMin) {
        T4Sintegral = T4SintegralMin;
    } else if (T4Sintegral > T4SintegralMax) {
        T4Sintegral = T4SintegralMax;
    }

    // 计算微分项
    T4Sderivative = T4Serror - T4SlastError;

    // 计算PID输出
    output = T4SKp * T4Serror + T4SKi * T4Sintegral + T4SKd * T4Sderivative;

    // 更新误差变量
    T4SlastError = T4Serror;

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




/// @brief 电机使能函数
/// @param x LevelOut 
//           LevelIn
//           VerticalOut
//           VerticalIn
///
/// @param y ENABLE/DISABLE
void Motor_Cmd(uint8_t MotorSit,FunctionalState NewState)
{
    switch (MotorSit)
    {
    case LevelOut:
        if(NewState==ENABLE)
        {
            RIN3=1;
            RIN4=0;
        }
        else
        {
            RIN3=0;
            RIN4=0;
        }
        break;
    case LevelIn:
        if(NewState==ENABLE)
        {
            RIN1=0;
            RIN2=1;
        }
        else
        {
            RIN1=0;
            RIN2=0;
        }
        break;
    case VerticalOut:
        if(NewState==ENABLE)
        {
            LIN1=1;
            LIN2=0;
        }
        else
        {
            LIN1=0;
            LIN2=0;
        }
        break;
    case VerticalIn:
        if(NewState==ENABLE)
        {
            LIN3=1;
            LIN4=0;
        }
        else
        {
            LIN3=0;
            LIN4=0;
        }
        break;
    case StopAll:
        LIN1=0;
        LIN2=0;
        LIN3=0;
        LIN4=0;
        RIN1=0;
        RIN2=0;
        RIN3=0;
        RIN4=0;
        break;

    default:
        break;
    }
}


/*
依据角度判断风机方位，调用电机使能函数

*/
void MotorState(float pitch,float roll)
{
    if(roll>0&&pitch>0)
    {
        Motor_Cmd(LevelOut,DISABLE);
        Motor_Cmd(LevelIn,ENABLE);
        Motor_Cmd(VerticalOut,ENABLE);
        Motor_Cmd(VerticalIn,DISABLE);
    }
    else if(roll>0&&pitch<0)
    {
        Motor_Cmd(LevelOut,ENABLE);
        Motor_Cmd(LevelIn,DISABLE);
        Motor_Cmd(VerticalOut,ENABLE);
        Motor_Cmd(VerticalIn,DISABLE);
    }
    else if(roll<0&&pitch>0)
    {
        Motor_Cmd(LevelOut,DISABLE);
        Motor_Cmd(LevelIn,ENABLE);
        Motor_Cmd(VerticalOut,DISABLE);
        Motor_Cmd(VerticalIn,ENABLE);
    }
    else if(roll<0&&pitch<0)
    {
        Motor_Cmd(LevelOut,ENABLE);
        Motor_Cmd(LevelIn,DISABLE);
        Motor_Cmd(VerticalOut,DISABLE);
        Motor_Cmd(VerticalIn,ENABLE);
    }
    else
    {
        Motor_Cmd(StopAll,DISABLE);
    }
}