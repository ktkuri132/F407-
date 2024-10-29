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
extern float pitch,roll,abspitch,absroll, yaw,def,dis,polar,Opolar,target_R;
uint8_t State_Data;

//���㼫����,�Լ������
void GetPolar(float roll,float pitch)
{
    /*
        �����def�ļ��㹫ʽ
        tan?(def)=tan?(roll)+tan?(pitch)
        def=atan(sqrt(tan?(roll)+tan?(pitch)))
    */
    
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
    dis = tanf((def/180)*3.1415926)*H;
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
        polar = 180+Opolar;
    }
    else if(pitch>0&&roll>0)
    {
        polar = 360-Opolar;
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

    if((T1Lerror<0.8)||(T1Lerror> -0.8))
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
__INLINE void T3Motor_CmdCombination(float Vo,float Lo,uint8_t a)
{
    if(a)
    {
        if(Vo>0)
        {
            Motor_Cmd(VerticalOut,ENABLE);
            Motor_Cmd(VerticalIn,DISABLE);
            Motor_Cmd(LevelOut,ENABLE);
            Motor_Cmd(LevelIn,DISABLE);
            Motor->MVerticalOut =(uint32_t)Vo;
            Motor->MLevelOut =(uint32_t)Lo;
        }
        else
        {
            Motor_Cmd(VerticalOut,DISABLE);
            Motor_Cmd(VerticalIn,ENABLE);
            Motor_Cmd(LevelOut,DISABLE);
            Motor_Cmd(LevelIn,ENABLE);
            Motor->MVerticalIn =(uint32_t)(-Vo);
            Motor->MLevelIn =(uint32_t)(-Lo);
        }
    }
    
    else
    {
        if(Vo>0)
        {
            Motor_Cmd(VerticalOut,DISABLE);
            Motor_Cmd(VerticalIn,ENABLE);
            Motor_Cmd(LevelOut,ENABLE);
            Motor_Cmd(LevelIn,DISABLE);
            Motor->MVerticalIn =(uint32_t)Vo;
            Motor->MLevelOut =(uint32_t)Lo;
        }
        else
        {
            Motor_Cmd(VerticalOut,ENABLE);
            Motor_Cmd(VerticalIn,DISABLE);
            Motor_Cmd(LevelOut,DISABLE);
            Motor_Cmd(LevelIn,ENABLE);
            Motor->MVerticalOut =(uint32_t)(-Vo);
            Motor->MLevelIn =(uint32_t)(-Lo);
        }
    }


    
}

//��ͨ�˲���
float (*T5LowPassFilter(float Vinput,float Linput,float a))[3]
{
    static float last_Vinput;
    static float last_Linput;

    static float Output[3];

    Output[0]=a*Vinput+(1-a)*last_Vinput;
    Output[1]=a*Linput+(1-a)*last_Linput;

    last_Vinput=Vinput;
    last_Linput=Linput;

    return &Output;
}

//#define VcovL
#define AllVL

extern float Target_dis;

// ��5�������ƺ���
__INLINE void T5Motor_CmdCombination(int sit,float Vo,float Lo,float tR)
{   
    
    Vo = Vo<0?(-Vo):Vo; //ȡ����ֵ
    Lo = Lo<0?(-Lo):Lo;
    
   
/* 
    float R = tanf((def/180)*PI)*H;//��ǰ��Rֵ

    float delta_R = 1-((Target_dis-R)/Target_dis);  //����R�Ĳ�ֵ�İٷֱ�
    delta_R = delta_R>1?1:delta_R;  //����R�Ĳ�ֵ�İٷֱ���0-1֮��
    if(delta_R<0.5)
    {
        delta_R += 0.6;
    }
    if(delta_R>0.8)
    {
        delta_R = 1;
    }

    Vo = Vo*delta_R;    //����R�Ĳ�ֵ�İٷֱȵ���Vo��ֵ
    Lo = Lo*delta_R;    //����R�Ĳ�ֵ�İٷֱȵ���Lo��ֵ
     */
    //��ͨ�˲�
    float (*FilterOutputArr)[3] =  T5LowPassFilter(Vo,Lo,0.7);
    Vo = (*FilterOutputArr)[0];
    Lo = (*FilterOutputArr)[1];

    

    switch (sit)
    {
        case 1:
        {
            
            #ifdef AllVL
            Motor_Cmd(LevelOut,ENABLE);
            Motor_Cmd(LevelIn,DISABLE);
            Motor->MLevelOut =(uint32_t)(Lo);

            Motor_Cmd(VerticalIn,ENABLE);
            Motor_Cmd(VerticalOut,DISABLE);
            Motor->MVerticalIn =(uint32_t)(Vo);
            #else
                #ifdef VcovL
                    Motor_Cmd(LevelOut,ENABLE);
                    Motor_Cmd(LevelIn,DISABLE);
                    Motor->MLevelOut =(uint32_t)(Lo);
                #else
                    Motor_Cmd(VerticalIn,ENABLE);
                    Motor_Cmd(VerticalOut,DISABLE);
                    Motor->MVerticalIn =(uint32_t)(Vo);
                #endif
            #endif
        }break;

        case 2:
        {
            
            #ifdef AllVL
            Motor_Cmd(LevelOut,DISABLE);
            Motor_Cmd(LevelIn,ENABLE);
            Motor->MLevelIn =(uint32_t)(Lo);

            Motor_Cmd(VerticalIn,ENABLE);
            Motor_Cmd(VerticalOut,DISABLE);
            Motor->MVerticalIn =(uint32_t)(Vo);
            #else
                #ifdef VcovL
                    Motor_Cmd(LevelOut,DISABLE);
                    Motor_Cmd(LevelIn,ENABLE);
                    Motor->MLevelIn =(uint32_t)(Lo);
                #else
                    Motor_Cmd(VerticalIn,ENABLE);
                    Motor_Cmd(VerticalOut,DISABLE);
                    Motor->MVerticalIn =(uint32_t)(Vo);
                #endif
            #endif
            
        }break;
        
        case 3:
        {
            #ifdef AllVL
            Motor_Cmd(LevelOut,DISABLE);
            Motor_Cmd(LevelIn,ENABLE);
            Motor->MLevelIn =(uint32_t)(Lo);

            Motor_Cmd(VerticalOut,ENABLE);
            Motor_Cmd(VerticalIn,DISABLE);
            Motor->MVerticalOut =(uint32_t)(Vo);
            #else
                #ifdef VcovL
                    Motor_Cmd(LevelOut,DISABLE);
                    Motor_Cmd(LevelIn,ENABLE);
                    Motor->MLevelIn =(uint32_t)(Lo);
                #else
                    Motor_Cmd(VerticalOut,ENABLE);
                    Motor_Cmd(VerticalIn,DISABLE);
                    Motor->MVerticalOut =(uint32_t)(Vo);
                #endif
            #endif

        }break;

        case 4:
        {
            
            #ifdef AllVL
                Motor_Cmd(LevelOut,ENABLE);
                Motor_Cmd(LevelIn,DISABLE);
                Motor->MLevelOut =(uint32_t)(Lo);

                Motor_Cmd(VerticalOut,ENABLE);
                Motor_Cmd(VerticalIn,DISABLE);
                Motor->MVerticalOut =(uint32_t)(Vo);
            #else
                #ifdef VcovL
                    Motor_Cmd(LevelOut,ENABLE);
                    Motor_Cmd(LevelIn,DISABLE);
                    Motor->MLevelOut =(uint32_t)(Lo);
                #else
                    Motor_Cmd(VerticalOut,ENABLE);
                    Motor_Cmd(VerticalIn,DISABLE);
                    Motor->MVerticalOut =(uint32_t)(Vo);
                #endif
            #endif
           
        }break;
 
    default:break;
        
    }
}

//�Ƕ���ƭ+Ŀ�껺��  ���ڵ�����
float T5Angle_Deceive(float def, float target_R)
{
    static float last_R;    //��һ�ε�Rֵ
    float output_R;     //�����Rֵ
    float R = tanf((def/180)*PI)*H;//��ǰ��Rֵ

    float arv_R = R*0.5+last_R*0.5;    //ƽ��Rֵ

    if(target_R<=0.16)  //����R��Ŀ��ֵ��15cm
    {
        if(arv_R>=0.11) //�ж�����R��ֵ�Ƿ�ﵽ10cm
        {           
            //����ﵽ10cm���ͽ�Ŀ��뾶��ֵ����Ϊ15cm
            output_R = 0.15;
        }
        else
        {
            R = arv_R;
        }
    }

    if(R>=0.24)
    {
        R=0.24;
    }
    
    R += 0.05;
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
uint8_t MotorLocation=0;

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

__attribute__((__weak__)) void Task4_StopFast()
{
    GetPolar(roll,pitch);

    static float VerticalOutput;
    static float LevelOutput;

    static struct PID Taks4Pid1={800,-1,0,PidControl_Stop};
    static struct PID Taks4Pid2={800,-1,0,PidControl_Stop};

    VerticalOutput = Taks4Pid1.PIDControl(TargetRoll,absroll, &Taks4Pid1);
    LevelOutput = Taks4Pid2.PIDControl(TargetPitch,abspitch, &Taks4Pid2);

    printf("%f,%f\r\n",VerticalOutput,LevelOutput);

    VerticalOutput=VerticalOutput>8400?8400:(VerticalOutput<0?(-VerticalOutput):VerticalOutput);
    LevelOutput=LevelOutput>8400?8400:(LevelOutput<0?(-LevelOutput):LevelOutput);

    switch (MotorLocation)
    {
        case 4:
        {
            Motor->MLevelIn =(uint32_t)(LevelOutput);
            Motor->MVerticalOut =(uint32_t)(VerticalOutput);
        }break;
        
        case 1:
        {
            Motor->MLevelOut =(uint32_t)(LevelOutput);
            Motor->MVerticalOut =(uint32_t)(VerticalOutput);
        }break;

        case 3:
        {
            Motor->MLevelIn =(uint32_t)(LevelOutput);
            Motor->MVerticalIn =(uint32_t)(VerticalOutput);
        }break;

        case 2:
        {
            Motor->MLevelOut =(uint32_t)(LevelOutput);
            Motor->MVerticalIn =(uint32_t)(VerticalOutput);
        }break;  
    
        default:break;
            
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

