#ifndef CONTROL_H
#define CONTROL_H

float pidControl(float target, float feedback);
void Motor_PWM_TIM8_Init();

void Task_MoveLine();

#endif 