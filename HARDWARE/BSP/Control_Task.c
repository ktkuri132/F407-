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
#define Motor           TIM8
#define MVerticalOut    CCR1    //��ֱ���������,M--���ǰ׺��Vertical--�������ֱ����Out--���λ�������
#define MVerticalIn     CCR2    //��ֱ�����ڲ���
#define MLevelOut       CCR3    //ˮƽ���������
#define MLevelIn        CCR4    //ˮƽ�����ڲ���


//������x��y����
#define MotorState(x,y) Motor_Cmd(x,y)  //xΪ�����ţ�yΪ���״̬

#define TargetRoll  0

extern float pitch,roll,yaw;

//ֱ��״̬����PID����
float LSKp=5000,
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
    //�ð���ֱ�߰ڶ������
    static float SwingState_Output;
    //�ð��屣��ֱ��״̬�����
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

    if(LineState_Output>0)  //�������0����ʱ������ڴ�ֱ��࣬Ӧ�򿪴�ֱ��෽����,�ر��ڲ���
    {
        MotorState(VerticalOut,ENABLE);
        MotorState(VerticalIn,DISABLE);

        Motor->MVerticalOut = (uint32_t)LineState_Output;

    }
    else if(LineState_Output<0)  //���С��0����ʱ������ڴ�ֱ�ڲ࣬Ӧ�򿪴�ֱ�ڲ෽����,�ر������
    {
        MotorState(VerticalOut,DISABLE);
        MotorState(VerticalIn,ENABLE);

        Motor->MVerticalIn = (uint32_t)(-LineState_Output);
    }
    else
    {
        MotorState(VerticalOut,DISABLE);
        MotorState(VerticalIn,DISABLE);
    }
   

}