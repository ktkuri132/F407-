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

//定义三轴角度
float pitch,roll,yaw;
//定义三轴角速度
//short 

/*

每一步都有都有调试信息，妈妈再也不用担心我找不出程序bug了

*/
int main(void) 
{
    
    /* 所有外设的初始化函数都在这个里面了  */
    if(BSP_Init())
    {
        printf("peripheral init failed\n");
        
    }
    else
    {
        printf("peripheral init done\n");
    }
     


    
    /* 前台程序轮询  */
    while (1)
    {
        MotorState(pitch,roll);
        OLED_Printf(0,0,OLED_6X8,"pitch:%f",pitch);
        OLED_Printf(0,16,OLED_6X8,"roll:%f",roll);
        OLED_Update();
    }
    


    for(;;);
        

}





