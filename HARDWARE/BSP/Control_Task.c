/*


这个文件 装载了  风力摆的核心部分---控制部分



*/



#include <stm32f4xx.h>
#include <bsp.h>
#include <control.h>
#include <usart.h>
#include <mpu6050.h>
#include <inv_mpu.h>
#include <stm32f4xx_exti.h>
#include <OLED.h>



extern float pitch,roll,yaw,dis;


//控制制动PID参数
float T4SKp=550,
      T4SKi=0,
      T4SKd=300;


.

/// @brief MPU6050读值触发的外部中断函数      
void EXTI15_10_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line15) != RESET)
    {
        EXTI_ClearITPendingBit(EXTI_Line15);
        
        mpu_dmp_get_data(&pitch, &roll, &yaw);      //读取MPU6050数据
        
        GetPolar(roll, pitch);    

        Task4_StopFast();    //调用控制函数--->第4项

    }
}


/*
    第四项  快速制动
*/
void Task4_StopFast()
{
    /*
    //垂直制动输出
    static float VerticalOutput;
    //水平制动输出
    static float LevelOutput;
    */
    static float Output;
    Output = PidControl_Stop(TargetDis,dis);
    //printf("Output:%f\n",Output);
    PWM_Allocation(Output);
  
}