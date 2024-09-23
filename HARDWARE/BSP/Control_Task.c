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

//方便调试，定义电机方位
#define Motor       TIM8
#define LineOut     CCR3
#define LineIn      CCR4
#define SwingOut    CCR1
#define SwingIn     CCR2


//定义电机x的y方向
#define MotorState(x,y) Motor_Cmd(x,y)

#define TargetRoll  0

extern float pitch,roll,yaw;

//直线状态控制PID参数
float LSKp=1000,
      LSKi=0,
      LSKd=0;

//摆动状态控制PID参数
float SWKp=20,
      SWKi=0,
      SWKd=0;


/// @brief MPU6050读值触发的外部中断函数      
void EXTI15_10_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line15) != RESET)
    {
        EXTI_ClearITPendingBit(EXTI_Line15);
        
        mpu_dmp_get_data(&pitch, &roll, &yaw);      //读取MPU6050数据
        
        Task_MoveLine();    //调用控制函数--->第一项

    }
}


/*
    PID——1闭环控制直线状态不偏离
    PID——2闭环控制直线摆动不越界
*/
void Task_MoveLine()
{

    static float TargetPitch=15;
    //摆动状态输出
    static float SwingState_Output;
    //直线状态输出
    static float LineState_Output;
    //检查当前摆体状态
    if(pitch<15&&pitch>0)          //处在外侧
    {
        SwingState_Output = PidControl_SwingState(TargetPitch, pitch);
    }
    else if(pitch<-15&&pitch>0)  //处在内侧
    {
        SwingState_Output = PidControl_SwingState(TargetPitch, pitch);
    }
    else if(pitch==15)          //达到外侧极限
    {
        TargetPitch=-15;
    }
    else if(pitch==-15)         //达到内侧极限
    {
        TargetPitch=15;
    }

    //规定水平直线状态时，偏离角度roll为0

    LineState_Output= PidControl_LineState(TargetRoll, roll);

    //假设OUt边全部为正，IN边全部为负
    if(LineState_Output>0)
    {
        MotorState(LevelIn,ENABLE);
        MotorState(LevelOut,DISABLE);

        Motor->LineOut = 0;
        Motor->LineIn = (uint32_t)SwingState_Output;
        if(SwingState_Output>0)
        {
            MotorState(VerticalOut,ENABLE);
            MotorState(VerticalIn,DISABLE);

            Motor->SwingOut = (uint32_t)LineState_Output;
            Motor->SwingIn= 0;
        }
        else if(SwingState_Output<0)
        {
            MotorState(VerticalOut,DISABLE);
            MotorState(VerticalIn,ENABLE);

            Motor->SwingOut = 0;
            Motor->SwingIn = -(uint32_t)LineState_Output;
        }
        else
        {
            MotorState(VerticalOut,DISABLE);
            MotorState(VerticalIn,DISABLE);

            Motor->SwingOut = 0;
            Motor->SwingIn = 0;
        }
    }
    else if(LineState_Output<0)
    {
        MotorState(LevelOut,ENABLE);
        MotorState(LevelIn,DISABLE);

        Motor->LineOut = -(uint32_t)SwingState_Output;
        Motor->LineIn = 0;
        if(SwingState_Output>0)
        {
            MotorState(VerticalOut,ENABLE);
            MotorState(VerticalIn,DISABLE);

            Motor->SwingOut = (uint32_t)LineState_Output;
            Motor->SwingIn = 0;
        }
        else if(SwingState_Output<0)
        {
            MotorState(VerticalOut,DISABLE);
            MotorState(VerticalIn,ENABLE);

            Motor->SwingOut = 0;
            Motor->SwingIn = -(uint32_t)LineState_Output;
        }
        else
        {
            MotorState(VerticalOut,DISABLE);
            MotorState(VerticalIn,DISABLE);

            Motor->SwingOut = 0;
            Motor->SwingIn = 0;
        }
    }
    else
    {
        MotorState(LevelOut,DISABLE);
        MotorState(LevelIn,DISABLE);

        Motor->LineOut = 0;
        Motor->LineIn = 0;
        if(SwingState_Output>0)
        {
            MotorState(VerticalOut,ENABLE);
            MotorState(VerticalIn,DISABLE);

            Motor->SwingOut = (uint32_t)LineState_Output;
            Motor->SwingIn = 0;
        }
        else if(SwingState_Output<0)
        {
            MotorState(VerticalOut,DISABLE);
            MotorState(VerticalIn,ENABLE);
            Motor->SwingOut = 0;
            Motor->SwingIn = -(uint32_t)LineState_Output;
        }
        else
        {
            MotorState(VerticalOut,DISABLE);
            MotorState(VerticalIn,DISABLE);
            Motor->SwingOut = 0;
            Motor->SwingIn = 0;
        }
    }
}