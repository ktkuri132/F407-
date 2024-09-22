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

extern float pitch,roll,yaw;

/// @brief MPU6050读值触发的外部中断函数      
void EXTI15_10_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line15) != RESET)
    {
        EXTI_ClearITPendingBit(EXTI_Line15);

        mpu_dmp_get_data(&pitch, &roll, &yaw);

        

        
    }
}


/* 风力摆走直线模式  */
void Task_MoveLine()
{
    
    
}