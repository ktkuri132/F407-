/*


����ļ� װ����  �����ڵĺ��Ĳ���---���Ʋ���



*/

//ʹ��ǰȡ��ע��

//ʹ�ö�ʱ��2�ж�
//#define __TIM2_IRQn__
//ʹ���ⲿ�ж�15
#define __EXTI15_10_IRQn__

#include <stm32f4xx.h>
#include <bsp.h>
#include <control.h>
#include <usart.h>
#include <mpu6050.h>
#include <inv_mpu.h>
#include <stm32f4xx_exti.h>
#include <OLED.h>
#include <math.h>


extern float pitch,roll,yaw,dis;


//�����ƶ�PID����
float T4SKp=900,
      T4SKi=-1,
      T4SKd=0;

float T1LKp=400,//520
      T1LKi=0,
      T1LKd=0;//-65.5

/*

15cm 500,0,-65.5,00983
20cm


*/



#ifdef __EXTI15_10_IRQn__

/// @brief MPU6050��ֵ�������ⲿ�жϺ���      
void EXTI15_10_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line15) != RESET)
    {
        EXTI_ClearITPendingBit(EXTI_Line15);
        
        mpu_dmp_get_data(&pitch, &roll, &yaw);      //��ȡMPU6050����
        
        GetPolar(roll, pitch);    


        //Task4_StopFast();    //���ÿ��ƺ���--->��4��
        Task1_LineMove(0.2);    //���ÿ��ƺ���--->��1��
    }
}

#endif

#ifdef __TIM2_IRQn__

void TIM2_IRQHandler(void)
{
    if(TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
    {
        //mpu_dmp_get_data(&pitch, &roll, &yaw);      //��ȡMPU6050����
        
        //GetPolar(roll, pitch);    



        //Task4_StopFast();    //���ÿ��ƺ���--->��4��
        Task1_LineMove(10);    //���ÿ��ƺ���--->��1��

    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);


    }
}

#endif


/*
    ������  �����ƶ�
*/
void Task4_StopFast()
{
    /*
    //��ֱ�ƶ����
    static float VerticalOutput;
    //ˮƽ�ƶ����
    static float LevelOutput;
    */
    static float Output;
    Output = PidControl_Stop(TargetDis,dis);
    //printf("Output:%f\n",Output);
    PWM_Allocation(Output);
  
}


#define T 1.678

float target_angle;

/*
    ��һ��  �����˶�
*/
void Task1_LineMove(float R)
{
    static float VOutput,LOutput;
    static float A;
    static float time=0;

    A = atanf(R/0.86)*180.0f/PI;
    target_angle = A*sinf(2*PI*time/T);
    
    


    VOutput = PidControl_LineMove(target_angle,roll);
    LOutput = PidControl_LineMove(0,pitch);
//0.00983---0.15
    time+=0.0097;
    if(time<T)
    {
        if(VOutput>0)
        {
            Motor_Cmd(VerticalOut,ENABLE);
            Motor_Cmd(VerticalIn,DISABLE);
            Motor->MVerticalOut =(uint32_t)VOutput;
        }
        else
        {
            Motor_Cmd(VerticalOut,DISABLE);
            Motor_Cmd(VerticalIn,ENABLE);
            Motor->MVerticalIn =(uint32_t)(-VOutput);
        }
    }
    else time=0;
    
}



