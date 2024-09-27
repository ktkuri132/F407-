/*


这个文件 装载了  风力摆的核心部分---控制部分



*/

//使用前取消注释

//使用定时器2中断
//#define __TIM2_IRQn__
//使用外部中断15
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


//控制制动PID参数
float T4SKp=900,
      T4SKi=-1,
      T4SKd=0;

float T1LKp=750,//520
      T1LKi=0,
      T1LKd=-350;//-65.5

/*

15cm 500,0,-65.5,00983
25cm 300,0,0,0.0104223  <---这个具有普遍性

750,0,-350,0.0104223


*/

float Target_dis=0.15;

#ifdef __EXTI15_10_IRQn__

/// @brief MPU6050读值触发的外部中断函数      
void EXTI15_10_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line15) != RESET)
    {
        EXTI_ClearITPendingBit(EXTI_Line15);
        
        mpu_dmp_get_data(&pitch, &roll, &yaw);      //读取MPU6050数据
        
        GetPolar(roll, pitch);    


        //Task4_StopFast();    //调用控制函数--->第4项
        Task1_LineMove(Target_dis);    //调用控制函数--->第1项
    }
}

#endif

#ifdef __TIM2_IRQn__

void TIM2_IRQHandler(void)
{
    if(TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
    {
        //mpu_dmp_get_data(&pitch, &roll, &yaw);      //读取MPU6050数据
        
        //GetPolar(roll, pitch);    



        //Task4_StopFast();    //调用控制函数--->第4项
        Task1_LineMove(10);    //调用控制函数--->第1项

    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);


    }
}

#endif


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


#define T 1.678

float target_angle;

/*
    第一,二项  单摆运动+幅度可调
*/
void Task1_LineMove(float R)
{
    static float VOutput,LOutput;
    static float A;
    static float time=0;

    A = atanf(R/0.86)*180.0f/PI;
    target_angle = A*sinf(2*PI*time/T);
    
    printf("%f,%f\r\n",target_angle,roll);

    VOutput = PidControl_LineMove(target_angle,roll);
    LOutput = PidControl_LineMove(0,pitch);
//0.00983---0.15
    time+=0.0104223;
    if(time<T)
    {
        if(VOutput>0)
        {
            Motor_Cmd(VerticalOut,ENABLE);
            Motor_Cmd(VerticalIn,DISABLE);
            //Motor_Cmd(LevelOut,ENABLE);
            //Motor_Cmd(LevelIn,DISABLE);
            //Motor->MLevelOut =(uint32_t)LOutput;
            Motor->MVerticalOut =(uint32_t)VOutput;
        }
        else
        {
            Motor_Cmd(VerticalOut,DISABLE);
            Motor_Cmd(VerticalIn,ENABLE);
            //Motor_Cmd(LevelOut,DISABLE);
            //Motor_Cmd(LevelIn,ENABLE);
            //Motor->MLevelIn =(uint32_t)(-LOutput);
            Motor->MVerticalIn =(uint32_t)(-VOutput);
        }
    }
    else time=0;
    
}


/*
    第三项  角度可调的单摆运动
*/
void Task3_AngleMove()
{
    

}
