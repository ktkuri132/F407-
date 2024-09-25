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
#include <math.h>
#include <stdlib.h>

extern uint8_t Stop_flag;
extern float pitch,roll,yaw,def,dis,polar;

#define heigh 86


//���㼫����,�Լ������
void GetPolar(float roll,float pitch)
{
    /*
        �����def�ļ��㹫ʽ
        tan?(def)=tan?(roll)+tan?(pitch)
        def=atan(sqrt(tan?(roll)+tan?(pitch)))
    */
    float absroll,abspitch;
    if(roll<0)
    {
        absroll=-roll;
    }
    else
    {
        absroll=roll;
    }
    if(pitch<0)
    {
        abspitch=-pitch;
    }
    else
    {
        abspitch=pitch;
    }
    float rad_roll=(absroll/180)*3.1415926;
    float rad_pitch=(abspitch/180)*3.1415926;
    float a= sqrtf(tanf(rad_roll)*tanf(rad_roll)+tanf(rad_pitch)*tanf(rad_pitch));
    def = (atanf(a)*180.0)/3.1415926;

    /*
        λ��dis�ļ��㹫ʽ
        dis=heigh*tan(def)
    
    */
    dis = tanf((def/180)*3.1415926)*heigh;
    float b = (tanf(rad_pitch)/tanf(rad_roll));
    polar = (atanf(b)*180.0)/3.1415926;
    
    if(pitch<0&&roll>0)
    {
        polar=polar;
    }
    else if(pitch<0&&roll<0)
    {
        polar = 180-polar;
    }
    else if(pitch>0&&roll<0)
    {
        polar = 180+polar;
    }
    else if(pitch>0&&roll>0)
    {
        polar = 360-polar;
    }

    //�涨������Ч����
    if(dis<1)
    {
        polar=0;
    }
    
}


//T--����4--�����S--ֹͣ���ƶ�
extern float T4SKp, T4SKi, T4SKd;


/// @brief PID�����ƶ�
/// @param target Ŀ��ֵ
/// @param feedback ��ǰֵ
/// @return ����ֵ�������ͣ���������
float PidControl_Stop(float target, float feedback)
{

    // ����������
    static float T4Serror = 0;
    static float T4SlastError = 0;
    static float T4Sintegral = 0;
    static float T4Sderivative = 0;

    // ��������޷�����
    static float T4SintegralMin = -8400;
    static float T4SintegralMax = 8400;

    // ����PID�������
    static float output = 0;

    // �������
    T4Serror = target - feedback;

    // ���������
    T4Sintegral += T4Serror;

    if(def<7.23&&def>7.01)
    {
        T4Sintegral=0;
    }

    // ���ƻ������ڻ����޷���Χ��
    if (T4Sintegral < T4SintegralMin) {
        T4Sintegral = T4SintegralMin;
    } else if (T4Sintegral > T4SintegralMax) {
        T4Sintegral = T4SintegralMax;
    }

    // ����΢����
    T4Sderivative = T4Serror - T4SlastError;

    // ����PID���
    output = T4SKp * T4Serror + T4SKi * T4Sintegral + T4SKd * T4Sderivative;


    // ����������
    T4SlastError = T4Serror;



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
            RIN3=0;
            RIN4=1;
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
            RIN1=1;
            RIN2=0;
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
            LIN3=0;
            LIN4=1;
        }
        else
        {
            LIN3=0;
            LIN4=0;
        }
        break;
    case VerticalIn:
        if(NewState==ENABLE)
        {
            LIN1=0;
            LIN2=1;
        }
        else
        {
            LIN1=0;
            LIN2=0;
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

void StopAllMotor()
{
    if(Stop_flag)
    {
        Motor_Cmd(StopAll,DISABLE);
        for(;;);
    }
    if(Stop_flag==0)
    {
        Motor_Cmd(StopAll,ENABLE);
    }
}


//���λ��
uint8_t MotorLocation;

/*
���ݽǶ��жϷ����λ�����õ��ʹ�ܺ���
*/
void MotorState(float pitch,float roll)
{
    if(roll>0&&pitch>0)
    {
        Motor_Cmd(LevelOut,DISABLE);
        Motor_Cmd(LevelIn,ENABLE);
        Motor_Cmd(VerticalOut,ENABLE);
        Motor_Cmd(VerticalIn,DISABLE);
        MotorLocation = 1;
    }
    else if(roll>0&&pitch<0)
    {
        Motor_Cmd(LevelOut,ENABLE);
        Motor_Cmd(LevelIn,DISABLE);
        Motor_Cmd(VerticalOut,ENABLE);
        Motor_Cmd(VerticalIn,DISABLE);
        MotorLocation = 2;
    }
    else if(roll<0&&pitch>0)
    {
        Motor_Cmd(LevelOut,DISABLE);
        Motor_Cmd(LevelIn,ENABLE);
        Motor_Cmd(VerticalOut,DISABLE);
        Motor_Cmd(VerticalIn,ENABLE);
        MotorLocation = 3;
    }
    else if(roll<0&&pitch<0)
    {
        Motor_Cmd(LevelOut,ENABLE);
        Motor_Cmd(LevelIn,DISABLE);
        Motor_Cmd(VerticalOut,DISABLE);
        Motor_Cmd(VerticalIn,ENABLE);
        MotorLocation = 4;
    }
    else
    {
        Motor_Cmd(StopAll,DISABLE);
        MotorLocation = 0;
    }
}

/*
���ݼ�����ǶȺ�λ�ƣ���PID���ֵ�����ֽ�
*/
void PWM_Allocation(float VerticalOutput,float LevelOutput)
{
    float VerticalOutput1;
    float LevelOutput1;

    float VerticalOutput2;
    float LevelOutput2;

    float VerticalOutput3;
    float LevelOutput3;
    
    float VerticalOutput4;
    float LevelOutput4;

    VerticalOutput=VerticalOutput>8400?8400:(VerticalOutput<0?(-VerticalOut):VerticalOutput);
    LevelOutput=LevelOutput>8400?8400:(LevelOutput<0?(-LevelOutput):LevelOutput);

    if(polar>=0&&polar<=90)
    {
        VerticalOutput1 = VerticalOutput*cosf((polar/180)*3.1415926);
        LevelOutput1 = LevelOutput*sinf((polar/180)*3.1415926);
        Motor->MVerticalOut=(uint32_t)VerticalOutput1;
        Motor->MLevelOut=(uint32_t)LevelOutput1;
        //printf("VerticalOutput1:%f  LevelOutput1:%f\n",VerticalOutput1,LevelOutput1);
    }
    else if(polar>90&&polar<=180)
    {
        VerticalOutput2 = VerticalOutput*cosf((180-polar)/180*3.1415926);
        LevelOutput2 = LevelOutput*sinf((180-polar)/180*3.1415926);
        Motor->MVerticalOut=(uint32_t)VerticalOutput2;
        Motor->MLevelIn=(uint32_t)LevelOutput2;
        //printf("VerticalOutput2:%f  LevelOutput2:%f\n",VerticalOutput2,LevelOutput2);
    }
    else if(polar>180&&polar<=270)
    {
        VerticalOutput3 = VerticalOutput*cosf((polar-180)/180*3.1415926);
        LevelOutput3 = LevelOutput*sinf((polar-180)/180*3.1415926);
        Motor->MVerticalIn=(uint32_t)VerticalOutput3;
        Motor->MLevelIn=(uint32_t)LevelOutput3;
        //printf("VerticalOutput3:%f  LevelOutput3:%f  polar:%f\n",VerticalOutput3,LevelOutput3,polar);
    }
    else if(polar>270&&polar<=360)
    {
        VerticalOutput4 = VerticalOutput*cosf((360-polar)/180*3.1415926);
        LevelOutput4 = LevelOutput*sinf((360-polar)/180*3.1415926);
        Motor->MVerticalIn=(uint32_t)VerticalOutput4;
        Motor->MLevelOut=(uint32_t)LevelOutput4;
        //printf("VerticalOutput4:%f  LevelOutput4:%f\n",VerticalOutput4,LevelOutput4);
    }
    else
    {
        VerticalOutput = 0;
        LevelOutput = 0;
    }
    
}