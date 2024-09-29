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
        //Task1_LineMove(Target_dis);    //调用控制函数--->第1项
        //Task3_AngleMove(45,0.15);
        Task5_CircleMove(0.2);
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
    static struct PID Taks4Pid={900,-1,0,PidControl_Stop};
    
    Output = Taks4Pid.PIDControl(TargetDis,dis, &Taks4Pid);
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
    static struct PID Taks1Pid={750,0,-350,PidControl_LineMove};
    
    A = atanf(R/0.86)*180.0f/PI;
    target_angle = A*sinf(2*PI*time/T);
    
    printf("%f，%f\r\n",target_angle,roll);

    VOutput = Taks1Pid.PIDControl(target_angle,roll, &Taks1Pid);
    LOutput = Taks1Pid.PIDControl(0,pitch, &Taks1Pid);
//0.00983---0.15
    time+=0.0104223;
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



/*
    第三项  角度可调的单摆运动
*/
void Task3_AngleMove(float angle,float R)
{
    static float VOutput,LOutput;
    static float A1,A2;
    static float time=0;
    static float Vtarget_angle,Ltarget_angle;
    static struct PID Taks3Pid={150,0,0,PidControl_LineMove};
    static struct  PID Taks3Pid2={100,0,0,PidControl_LineMove};

    

    //angle = angle+5;
    angle = angle*PI/180.0f;

    A1 = asinf(R*cosf(angle)/0.86)*180.0f/PI;
    A2 = asinf(R*sinf(angle)/0.86)*180.0f/PI;
	
		//printf("%f,%f\r\n",A1,A2);
			
    Vtarget_angle = A1*sinf(2*PI*time/T);
    Ltarget_angle = A2*sinf(2*PI*time/T);

    printf("%f,%f\r\n",Vtarget_angle,Ltarget_angle);

    VOutput = Taks3Pid.PIDControl(Vtarget_angle,roll,&Taks3Pid);
    LOutput = Taks3Pid2.PIDControl(Ltarget_angle,pitch,&Taks3Pid2);

    time+=0.0104223;
    if(time<T)
    {
        if(VOutput>0)
        {
            Motor_Cmd(VerticalOut,ENABLE);
            Motor_Cmd(VerticalIn,DISABLE);
            Motor->MVerticalOut =(uint32_t)VOutput;
            if(LOutput>0)
            {
                Motor_Cmd(LevelOut,ENABLE);
                Motor_Cmd(LevelIn,DISABLE);
                Motor->MLevelOut =(uint32_t)LOutput;
            }
            else
            {
                Motor_Cmd(LevelOut,DISABLE);
                Motor_Cmd(LevelIn,ENABLE);
                Motor->MLevelIn =(uint32_t)(-LOutput);
            }
        }
        else
        {
            Motor_Cmd(VerticalOut,DISABLE);
            Motor_Cmd(VerticalIn,ENABLE);
            Motor->MVerticalIn =(uint32_t)(-VOutput);
            if(LOutput>0)
            {
                Motor_Cmd(LevelOut,ENABLE);
                Motor_Cmd(LevelIn,DISABLE);
                Motor->MLevelOut =(uint32_t)LOutput;
            }
            else
            {
                Motor_Cmd(LevelOut,DISABLE);
                Motor_Cmd(LevelIn,ENABLE);
                Motor->MLevelIn =(uint32_t)(-LOutput);
            }
        }
    }
    else time=0;
}


/*
    画圆
*/
void Task5_CircleMove(float R)
{
    static float VOutput,LOutput;
    static float A;
    static float time=0;
    static float Vtarget_angle,Ltarget_angle;
    static struct PID Taks5Pid={150,0,0,PidControl_LineMove};
    static struct PID Taks5Pid2={90,0,0,PidControl_LineMove};

    A = asinf(R/0.86)*180.0f/PI;
    
    
        //printf("%f,%f\r\n",A1,A2);
            
    Vtarget_angle = A*sinf(2*PI*(time/T));
    Ltarget_angle = A*sinf(2*PI*(time/T)+PI/2);

    printf("%f,%f\r\n",Vtarget_angle,Ltarget_angle);

    VOutput = Taks5Pid.PIDControl(Vtarget_angle,roll,&Taks5Pid);
    LOutput = Taks5Pid2.PIDControl(Ltarget_angle,pitch,&Taks5Pid2);

    time+=0.0104223;
    if(time<T)
    {
        if(VOutput>0)
        {
            Motor_Cmd(VerticalOut,ENABLE);
            Motor_Cmd(VerticalIn,DISABLE);
            Motor->MVerticalOut =(uint32_t)VOutput;
            if(LOutput>0)
            {
                Motor_Cmd(LevelOut,ENABLE);
                Motor_Cmd(LevelIn,DISABLE);
                Motor->MLevelOut =(uint32_t)LOutput;
            }
            else
            {
                Motor_Cmd(LevelOut,DISABLE);
                Motor_Cmd(LevelIn,ENABLE);
                Motor->MLevelIn =(uint32_t)(-LOutput);
            }
        }
        else
        {
            Motor_Cmd(VerticalOut,DISABLE);
            Motor_Cmd(VerticalIn,ENABLE);
            Motor->MVerticalIn =(uint32_t)(-VOutput);
            if(LOutput>0)
            {
                Motor_Cmd(LevelOut,ENABLE);
                Motor_Cmd(LevelIn,DISABLE);
                Motor->MLevelOut =(uint32_t)LOutput;
            }
            else
            {
                Motor_Cmd(LevelOut,DISABLE);
                Motor_Cmd(LevelIn,ENABLE);
                Motor->MLevelIn =(uint32_t)(-LOutput);
            }
        }
    }
    else time=0;
}