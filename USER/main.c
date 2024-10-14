/**
    ******************************************************************************
    * @file    Project/STM32F4xx_StdPeriph_Templates/main.c 
    * @author  MCD Application Team
    * @version V1.8.1
    * @date    27-January-2022
    * @brief   Main program body
    ******************************************************************************
    * @attention
    *
    * Copyright (c) 2016 STMicroelectronics.
    * All rights reserved.
    *
    * This software is licensed under terms that can be found in the LICENSE file
    * in the root directory of this software component.
    * If no LICENSE file comes with this software, it is provided AS-IS.
    *
    ******************************************************************************
    */

/* Includes ------------------------------------------------------------------*/



#include "stm32f4xx.h"
#include "bsp.h"
#include <OLED.h>
#include <usart.h>
#include <delay.h>
#include <shell.h>
#include <mpu6050.h>
#include <inv_mpu.h>
#include <control.h>

//��������Ƕ�
float pitch,roll,yaw;
//���������
float def;
//����λ��
float dis;
//���弫����Ƕ�
float polar;
//�õ��ļ������ԭʼ�Ƕ�
float Opolar;
extern float target_angle;

/*

ÿһ�����ж��е�����Ϣ��������Ҳ���õ������Ҳ�������bug��

*/
int main(void) 
{
    
    /* ��������ĳ�ʼ�������������������  */
    if(BSP_Init())
    {
        printf("peripheral init failed\n");
        
    }
    else
    {
        printf("peripheral init done\n");
    }

/*
    TIM8->CCR1=600;//ˮƽ�����ڲ���
    TIM8->CCR2=600;//ˮƽ���������
    TIM8->CCR3=600;//��ֱ�����ڲ���
    TIM8->CCR4=600;//��ֱ���������

    LIN1=1;//ˮƽ�����ڲ���
    LIN2=0;

    LIN3=0;//ˮƽ���������
    LIN4=1;

    RIN1=0;//��ֱ���������
    RIN2=1;

    RIN3=0;//��ֱ�����ڲ���
    RIN4=1;
*/
 
    /* ǰ̨������ѯ  */
    while (1)
    {
        //MotorState(pitch,roll);
        OLED_Printf(0,0,OLED_6X8,"pitch:%f",pitch);
        OLED_Printf(0,16,OLED_6X8,"roll:%f",roll);
        OLED_Printf(0,32,OLED_6X8,"def:%f",def);
        //printf("dis:%f  polar:%f\n",dis,polar);
        //printf("%f,%f\r\n",roll,target_angle);
        OLED_Update();
    }
    


    
        

}





