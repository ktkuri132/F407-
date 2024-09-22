#ifndef CONTROL_H
#define CONTROL_H

/* 基础  */
void Motor_PWM_TIM8_Init();

/* 第一项  */
float PidControl_LineState(float target, float feedback);
float PidControl_SwingState(float target, float feedback);
void Task_MoveLine();

/* 第二项  */



#endif 