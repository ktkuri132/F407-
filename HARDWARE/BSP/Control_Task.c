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

//������ԣ���������λ
#define Motor       TIM8
#define LineOut     CCR3
#define LineIn      CCR4
#define SwingOut    CCR1
#define SwingIn     CCR2


//������x��y����
#define MotorState(x,y) Motor_Cmd(x,y)

#define TargetRoll  0

extern float pitch,roll,yaw;

//ֱ��״̬����PID����
float LSKp=1000,
      LSKi=0,
      LSKd=0;

//�ڶ�״̬����PID����
float SWKp=20,
      SWKi=0,
      SWKd=0;


/// @brief MPU6050��ֵ�������ⲿ�жϺ���      
void EXTI15_10_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line15) != RESET)
    {
        EXTI_ClearITPendingBit(EXTI_Line15);
        
        mpu_dmp_get_data(&pitch, &roll, &yaw);      //��ȡMPU6050����
        
        Task_MoveLine();    //���ÿ��ƺ���--->��һ��

    }
}


/*
    PID����1�ջ�����ֱ��״̬��ƫ��
    PID����2�ջ�����ֱ�߰ڶ���Խ��
*/
void Task_MoveLine()
{

    static float TargetPitch=15;
    //�ڶ�״̬���
    static float SwingState_Output;
    //ֱ��״̬���
    static float LineState_Output;
    //��鵱ǰ����״̬
    if(pitch<15&&pitch>0)          //�������
    {
        SwingState_Output = PidControl_SwingState(TargetPitch, pitch);
    }
    else if(pitch<-15&&pitch>0)  //�����ڲ�
    {
        SwingState_Output = PidControl_SwingState(TargetPitch, pitch);
    }
    else if(pitch==15)          //�ﵽ��༫��
    {
        TargetPitch=-15;
    }
    else if(pitch==-15)         //�ﵽ�ڲ༫��
    {
        TargetPitch=15;
    }

    //�涨ˮƽֱ��״̬ʱ��ƫ��Ƕ�rollΪ0

    LineState_Output= PidControl_LineState(TargetRoll, roll);

    //����OUt��ȫ��Ϊ����IN��ȫ��Ϊ��
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