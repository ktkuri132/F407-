/*


����ļ� װ����  �����ڵĺ��Ĳ���---���Ʋ���



*/

//ѡ��ģʽ�ĺ궨��
#define Task4

#include <stm32f4xx.h>
#include <bsp.h>
#include <control.h>
#include <usart.h>
#include <mpu6050.h>
#include <inv_mpu.h>
#include <stm32f4xx_exti.h>
#include <OLED.h>

//������ԣ���������λ
#define Motor           TIM8
#define MVerticalOutput    CCR1    //��ֱ���������,M--���ǰ׺��Vertical--�������ֱ����Out--���λ�������
#define MVerticalIn     CCR2    //��ֱ�����ڲ���
#define MLevelOut       CCR3    //ˮƽ���������
#define MLevelIn        CCR4    //ˮƽ�����ڲ���


#ifdef Task4
#define TargetRoll  0
#define TargetPitch 0 
#endif


extern float pitch,roll,yaw;


//�����ƶ�PID����
float T4SKp=300,
      T4SKi=0,
      T4SKd=0;



/// @brief MPU6050��ֵ�������ⲿ�жϺ���      
void EXTI15_10_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line15) != RESET)
    {
        EXTI_ClearITPendingBit(EXTI_Line15);
        
        mpu_dmp_get_data(&pitch, &roll, &yaw);      //��ȡMPU6050����
        
        Task4_StopFast();    //���ÿ��ƺ���--->��4��

    }
}


/*
    ������  �����ƶ�
*/
void Task4_StopFast()
{
    //��ֱ�ƶ����
    static float VerticalOutput;
    //ˮƽ�ƶ����
    static float LevelOutput;
    
    VerticalOutput=PidControl_Stop(TargetRoll, pitch);
    LevelOutput= PidControl_Stop(TargetPitch, roll);

    if(VerticalOutput<0)
    {
        VerticalOutput=-VerticalOutput;
    }
    if(LevelOutput<0)
    {
        LevelOutput=-LevelOutput;
    }

    Motor->MVerticalOutput=(uint32_t)VerticalOutput;
    Motor->MVerticalIn=(uint32_t)VerticalOutput;
    Motor->MLevelOut=(uint32_t)LevelOutput;
    Motor->MLevelIn=(uint32_t)LevelOutput;
  
}