/*

����ļ�  ʵ����  PID���Ƽ��㣬PWM���  �Ƿ����ڿ����е��õ���Ҫ����

    ˮƽ���� pitch��  ---���� + ���� -
    ǰ���� roll��  ---���� - ���� +
    ��ת���� yaw�ᣨδʹ�ã�

*/


#include "stdio.h"
#include <stm32f4xx.h>
#include <bsp.h>
#include <sys.h>
#include <control.h>


extern float LSKp, LSKi, LSKd;
extern float SWKp, SWKi, SWKd;


/// @brief PIDֱ�߿��ƺ���
/// @param target Ŀ��ֵ
/// @param feedback ��ǰֵ
/// @return ����ֵ�������ͣ���������
float PidControl_LineState(float target, float feedback)
{

    // ����������
    static float LSerror = 0;
    static float LSlastError = 0;
    static float LSintegral = 0;
    static float LSderivative = 0;

    // ��������޷�����
    static float LSintegralMin = 0;
    static float LSintegralMax = 0;

    // ����PID�������
    static float output = 0;

    // �������
    LSerror = target - feedback;

    // ���������
    LSintegral += LSerror;

    // ���ƻ������ڻ����޷���Χ��
    if (LSintegral < LSintegralMin) {
        LSintegral = LSintegralMin;
    } else if (LSintegral > LSintegralMax) {
        LSintegral = LSintegralMax;
    }

    // ����΢����
    LSderivative = LSerror - LSlastError;

    // ����PID���
    output = LSKp * LSerror + LSKi * LSintegral + LSKd * LSderivative;

    // ����������
    LSlastError = LSerror;

    return output;

}

/// @brief PID�ڶ����ƺ���
/// @param target Ŀ��ֵ
/// @param feedback ��ǰֵ
/// @return ����ֵ�������ͣ���������
float PidControl_SwingState(float target, float feedback)
{

    // ����������
    static float SWerror = 0;
    static float SWlastError = 0;
    static float SWintegral = 0;
    static float SWderivative = 0;

    // ��������޷�����
    static float SWintegralMin = 0;
    static float SWintegralMax = 0;

    // ����PID�������
    static float output = 0;

    // �������
    SWerror = target - feedback;

    // ���������
    SWintegral += SWerror;

    // ���ƻ������ڻ����޷���Χ��
    if (SWintegral < SWintegralMin) {
        SWintegral = SWintegralMin;
    } else if (SWintegral > SWintegralMax) {
        SWintegral = SWintegralMax;
    }

    // ����΢����
    SWderivative = SWerror - SWlastError;

    // ����PID���
    output = SWKp * SWerror + SWKi * SWintegral + SWKd * SWderivative;

    // ����������
    SWlastError = SWerror;

    return output;
    
}

//PWM��������ø߼���ʱ��TIM8�����4·PWM
void Motor_PWM_TIM8_Init()
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

    GPIO_Config(GPIOC, GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9, GPIO_Mode_AF, GPIO_PuPd_NOPULL);

    GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM8);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM8);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_TIM8);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_TIM8);

    // ����TIM8
    TIM_TimeBaseInitTypeDef TIM8_TimeBaseInitStruct;
    TIM8_TimeBaseInitStruct.TIM_Period = 8400-1;  
    TIM8_TimeBaseInitStruct.TIM_Prescaler = 1;  
    TIM8_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM8_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM8, &TIM8_TimeBaseInitStruct);


    // ����TIM8��4��ͨ��ΪPWMģʽ
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

    // ʹ��TIM8�����
    TIM_CtrlPWMOutputs(TIM8, ENABLE);
    // ʹ��TIM8
    TIM_Cmd(TIM8, ENABLE);

    printf("->Motor->TIM1_TIM8 Enable  done\n");
}




/// @brief ���ʹ�ܺ���
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