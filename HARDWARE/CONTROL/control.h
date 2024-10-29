#ifndef CONTROL_H
#define CONTROL_H
#include <stm32f4xx.h>

//ʹ������1
#define LIN1 PFout(0)
#define LIN2 PFout(1)//vo
#define LIN3 PFout(2)
#define LIN4 PFout(3)//vi
//ʹ������2
#define RIN1 PFout(4)
#define RIN2 PFout(5)//li
#define RIN3 PFout(6)
#define RIN4 PFout(7)//lo

//���λ��
#define VerticalIn      3
#define VerticalOut     4
#define LevelOut        2
#define LevelIn         1
#define StopAll         0

//������ԣ���������λ
#define Motor           TIM8
#define MVerticalIn     CCR3    //��ֱ�����ڲ���,M--���ǰ׺��Vertical--�������ֱ����Out--���λ�������
#define MVerticalOut    CCR4    //��ֱ���������
#define MLevelOut       CCR2    //ˮƽ���������
#define MLevelIn        CCR1   //ˮƽ�����ڲ���

//ѡ��ģʽ�ĺ궨��
#define Task4

#ifdef Task4
#define TargetRoll  0
#define TargetPitch 0
#define TargetDis   0
#endif

//Բ����
#define PI 3.1415926
//���������
#define H 0.875
//����
#define T 1.678

//PID�ṹ��
typedef struct PID
{
    float Kp;
    float Ki;
    float Kd;
    float (*PIDControl)(float target, float feedback,struct PID* pid);
    
};


/* ����  */

void Motor_PWM_TIM8_Init();
void GetPolar(float roll,float pitch);



void StopAllMotor();


/* ��һ��  */

float PidControl_LineMove(float target, float feedback,struct PID* pid);
void Task1_LineMove(float R,uint8_t forward);

/* �ڶ���  */


/* ������  */
extern uint8_t State_Data;

void Task3_AngleMove(float angle,float R);
float (*T3State_Update(float angle,float R,float roll,float pitch))[5];
void T3Motor_CmdCombination(float Vo,float Lo,uint8_t a);

/* ������  */

float PidControl_Stop(float target, float feedback,struct PID* pid);
void Task4_StopFast();
void PWM_Allocation(float Output);
void MotorState(float pitch,float roll);


/* ������  */
void Task5_CircleMove(float R);
void T5Motor_CmdCombination(int sit,float Vo,float Lo,float tR);
float (*T5LowPassFilter(float Vinput,float Linput,float a))[3];

#endif 