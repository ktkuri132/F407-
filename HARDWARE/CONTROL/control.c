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
extern float pitch,roll,yaw,def,dis,polar,Opolar;
uint8_t State_Data;

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
    Opolar = (atanf(b)*180.0)/3.1415926;
    
    if(pitch<0&&roll>0)
    {
        polar=Opolar;
    }
    else if(pitch<0&&roll<0)
    {
        polar = 180-Opolar;
    }
    else if(pitch>0&&roll<0)
    {
        polar = 180-Opolar;
    }
    else if(pitch>0&&roll>0)
    {
        polar = Opolar;
    }

    //�涨������Ч����
    if(dis<1)
    {
        polar=0;
    }
    
}

//����


//T--����4--�����S--ֹͣ���ƶ�
extern float T4SKp, T4SKi, T4SKd;
//extern float T1LKp, T1LKi, T1LKd;

/// @brief PID�����ƶ�
/// @param target Ŀ��ֵ
/// @param feedback ��ǰֵ
/// @return ����ֵ�������ͣ���������
float PidControl_Stop(float target, float feedback,struct PID* pid)
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
    T4Serror = feedback - target;

    // ���������
    T4Sintegral += T4Serror;

    if(def<0.8)
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
    output = pid->Kp * T4Serror + pid->Ki * T4Sintegral + pid->Kd * T4Sderivative;

    // ����������
    T4SlastError = T4Serror;

    return output;

}

/// @brief PID���������ƶ�
/// @param target Ŀ��ֵ
/// @param feedback ��ǰֵ
/// @return ����ֵ�������ͣ���������
float PidControl_LineMove(float target, float feedback,struct PID* pid)
{
    // ����������
    static float T1Lerror = 0;
    static float T1LlastError = 0;
    static float T1Lintegral = 0;
    static float T1Lderivative = 0;

    // ��������޷�����
    static float T1LintegralMin = -8400;
    static float T1LintegralMax = 8400;

    // ����PID�������
    static float output = 0;

    // �������
    T1Lerror = feedback - target;

    // ���������
    T1Lintegral += T1Lerror;

    if(def<0.8)
    {
        T1Lintegral=0;
    }

    // ���ƻ������ڻ����޷���Χ��
    if (T1Lintegral < T1LintegralMin) {
        T1Lintegral = T1LintegralMin;
    } else if (T1Lintegral > T1LintegralMax) {
        T1Lintegral = T1LintegralMax;
    }

    // ����΢����
    T1Lderivative = T1Lerror - T1LlastError;

    // ����PID���
    output = pid->Kp * T1Lerror + pid->Ki * T1Lintegral + pid->Kd * T1Lderivative;

    // ����������
    T1LlastError = T1Lerror;

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
            LIN3=0;
            LIN4=1;
        }
        else
        {
            LIN3=0;
            LIN4=0;
        }
        break;
    case LevelIn:
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
    case VerticalOut:
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
    case VerticalIn:
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

/// @brief ����Ĭ���˶�����90�������ϣ�180�������£���������뷽λ�����Ƶ��
/// @param Vo ��ֱ�������
/// @param Lo ˮƽ�������
__INLINE void T3Motor_CmdCombination(float Vo,float Lo)
{
   /*  PID������ ����ǰֵ-Ŀ��ֵ */

    if(State_Data)  /* ����90�˶���  */
    {
        if(Vo>0)    /* ��ǰ��ֱ�����������  */
        {
            Motor_Cmd(VerticalOut,DISABLE);
            Motor_Cmd(VerticalIn,ENABLE);
            Motor->MVerticalIn =(uint32_t)Vo;
        }
        else        /* ��ǰ��ֱ�����������  */
        {
            Motor_Cmd(VerticalOut,ENABLE);
            Motor_Cmd(VerticalIn,DISABLE);
            Motor->MVerticalOut =(uint32_t)(-Vo);
        }

        if(Lo>0)
        {
            Motor_Cmd(LevelOut,DISABLE);
            Motor_Cmd(LevelIn,ENABLE);
            Motor->MLevelIn =(uint32_t)Lo;
        }
        else
        {
            Motor_Cmd(LevelOut,ENABLE);
            Motor_Cmd(LevelIn,DISABLE);
            Motor->MLevelOut =(uint32_t)(-Lo);
        }
    }
    else            /* ����180�˶���  */
    {
        if(Vo>0)
        {
            Motor_Cmd(VerticalOut,ENABLE);
            Motor_Cmd(VerticalIn,DISABLE);
            Motor->MVerticalOut =(uint32_t)Vo;
        }
        else
        {
            Motor_Cmd(VerticalOut,DISABLE);
            Motor_Cmd(VerticalIn,ENABLE);
            Motor->MVerticalIn =(uint32_t)(-Vo);
        }

        if(Lo>0)
        {
            Motor_Cmd(LevelOut,DISABLE);
            Motor_Cmd(LevelIn,ENABLE);
            Motor->MLevelOut =(uint32_t)Lo;
        }
        else
        {
            Motor_Cmd(LevelOut,ENABLE);
            Motor_Cmd(LevelIn,DISABLE);
            Motor->MLevelIn =(uint32_t)(-Lo);
        }
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


/// @brief ����Ҫ��ĽǶȺ�λ�ƣ��������ǰλ����Ϣ�� ����Ŀ�����Һ���ֵ
/// @param angle λ�ýǶ�
/// @param R �ڶ��뾶
/// @return ����ʱ�䣬��ֱ����Ŀ��Ƕȣ�ˮƽ����Ŀ��Ƕȵ�����
__INLINE float (*T3State_Update(float angle,float R,float roll,float pitch))[5]
{
    static float Output[5];
    
    

    /* ������180�ĽǶ�ȫ��ת������  */
    if(angle>180)
    {
        angle=360-angle;
    }
    
/************************************************************************* */

    float target_roll,target_pitch;
    float time=0;
    float Vtarget_angle,Ltarget_angle;

    time+=0.0104223;
    if(time>T)
    {
        time=0;
    }

    angle = angle*PI/180.0f;

    target_roll = atanf(R*cosf(angle)/0.86)/PI*180;
    target_pitch = atanf(R*sinf(angle)/0.86)/PI*180;

    float wt = 2*PI*(time/T);
			 
    Vtarget_angle =sinf(wt)*target_roll;
    Ltarget_angle =sinf(wt)*target_pitch;


/**************************************************************************** */

    /* �˴�Ŀ����Ҫ������Ŀ��Ƕȼ��Բ�ƥ���ʵ�ʽǶȱ任���� ��������λ״̬��90������ or 180������ */
    /* �жϵ�ǰҪ�����Ǹ�������ֱ��  PID������ ����ǰֵ-Ŀ��ֵ */

    if(angle>=90)/* ����90�㷶Χ��  */
    {
        pitch = -pitch;
        State_Data = 1;
    }
    else/* ����90�㷶Χ  */
    {
        if((wt>=0&&(wt<PI)))   /* 0<=wt<PI ��180������·� ��ʱsin(wt)>0,Avsin(wt)<0,Alsin(wt)>0 */
        {
            pitch = -pitch;
        }
        else if((wt>=PI&&(wt<2*PI)))/* PI<=wt<2PI ��180������Ϸ� ��ʱsin(wt)<0,Avsin(wt)>0,Alsin(wt)<0 */
        {
            roll = -roll;
        }
        State_Data = 0;
    }

/***************************************************************************** */

//  Output[0] = 0;      ��0λ����
    Output[1] = Vtarget_angle;
    Output[2] = Ltarget_angle;
    Output[3] = roll;
    Output[4] = pitch;

    return &Output;
}



//���λ��
uint8_t MotorLocation=0;

#ifdef Function_For_Task4
/*
���ݽǶ��жϷ����λ�����õ��ʹ�ܺ���
*/
__attribute__((__weak__)) void MotorState(float pitch,float roll) 
{
    if(roll>0&&pitch>0)
    {
        Motor_Cmd(LevelOut,DISABLE);
        Motor_Cmd(LevelIn,ENABLE);
        Motor_Cmd(VerticalOut,ENABLE);
        Motor_Cmd(VerticalIn,DISABLE);
        MotorLocation = 4;
    }
    else if(roll>0&&pitch<0)
    {
        Motor_Cmd(LevelOut,ENABLE);
        Motor_Cmd(LevelIn,DISABLE);
        Motor_Cmd(VerticalOut,ENABLE);
        Motor_Cmd(VerticalIn,DISABLE);
        MotorLocation = 1;
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
        MotorLocation = 2;
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
void PWM_Allocation(float Output)
{
    

    Output=Output>8400?8400:(Output<0?(-Output):Output);
    
    switch (MotorLocation)
    {
        case 1:
        {
            Motor->MVerticalOut=(uint32_t)(Output*cosf((Opolar/180)*3.1415926));
            Motor->MLevelOut=(uint32_t)(Output*sinf((Opolar/180)*3.1415926));
            //printf("cos(p):%f sin(p):%f Oplar:%f\n",cosf((Opolar/180)*3.1415926),sinf((Opolar/180)*3.1415926),Opolar);
        }break;

        case 2:
        {
            Motor->MVerticalIn=(uint32_t)(Output*cosf((Opolar/180)*3.1415926));
            Motor->MLevelOut=(uint32_t)(Output*sinf((Opolar/180)*3.1415926));
        }break;

        case 3:
        {
            Motor->MVerticalIn=(uint32_t)(Output*cosf((Opolar/180)*3.1415926));
            Motor->MLevelIn=(uint32_t)(Output*sinf((Opolar/180)*3.1415926));
        }break;

        case 4:
        {
            Motor->MVerticalOut=(uint32_t)(Output*cosf((Opolar/180)*3.1415926));
            Motor->MLevelIn=(uint32_t)(Output*sinf((Opolar/180)*3.1415926));
        }break;

        default :break;
    }

   
    
}

#endif