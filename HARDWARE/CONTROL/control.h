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
#define VerticalIn      1
#define VerticalOut     2
#define LevelOut        3
#define LevelIn         4
#define StopAll         0

//������ԣ���������λ
#define Motor           TIM8
#define MVerticalIn     CCR1    //��ֱ���������,M--���ǰ׺��Vertical--�������ֱ����Out--���λ�������
#define MVerticalOut    CCR2    //��ֱ�����ڲ���
#define MLevelOut       CCR3    //ˮƽ���������
#define MLevelIn        CCR4    //ˮƽ�����ڲ���

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
#define heigh 86

//PID�ṹ��
typedef struct PID
{
    float Kp;
    float Ki;
    float Kd;
    float (*PIDControl)(float target, float feedback,struct PID* pid);
    
};




//����ƶ�ʹ��
#define ControlMotor(x,y) 


/* ����  */

void Motor_PWM_TIM8_Init();
void MotorState(float pitch,float roll);
void StopAllMotor();
void GetPolar(float roll,float pitch);


/* ��һ��  */

float PidControl_LineMove(float target, float feedback,struct PID* pid);
void Task1_LineMove(float R);

/* �ڶ���  */


/* ������  */
void Task3_AngleMove(float angle,float R);

/* ������  */

float PidControl_Stop(float target, float feedback,struct PID* pid);
void Task4_StopFast();
void PWM_Allocation(float Output);


/* ������  */
void Task5_CircleMove(float R);
#endif 