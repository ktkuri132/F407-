/*


����ļ� װ����  �����ڵĺ��Ĳ���---���Ʋ���



*/



#include <stm32f4xx.h>
#include <bsp.h>
#include <control.h>
#include <usart.h>
#include <mpu6050.h>
#include <inv_mpu.h>
#include <stm32f4xx_exti.h>
#include <OLED.h>



extern float pitch,roll,yaw;


//�����ƶ�PID����
float T4SKp=900,
      T4SKi=0,
      T4SKd=0;



/// @brief MPU6050��ֵ�������ⲿ�жϺ���      
void EXTI15_10_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line15) != RESET)
    {
        EXTI_ClearITPendingBit(EXTI_Line15);
        
        mpu_dmp_get_data(&pitch, &roll, &yaw);      //��ȡMPU6050����
        
        GetPolar(roll, pitch);    

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
    
    VerticalOutput=PidControl_Stop(TargetRoll, roll);
    LevelOutput= PidControl_Stop(TargetPitch, pitch);

    PWM_Allocation(VerticalOutput,LevelOutput);
  
}