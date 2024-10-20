#if !defined __BSP__
#define __BSP__

#include <OLED.h>
#include <stm32f4xx.h>
#include <usart.h>
#include <bsp.h>
#include <control.h>
#include <usart.h>
#include <mpu6050.h>
#include <inv_mpu.h>
#include <stm32f4xx_exti.h>
#include <delay.h>
#include <hmi.h>

extern uint8_t mode;

void LED_Init();
void GPIO_Config(GPIO_TypeDef *GPIOx,uint32_t GPIO_Pin_x,uint32_t GPIO_Mode_x,uint32_t GPIO_PuPd_x);
int BSP_Init();
void BSP_NVIC_IT_Config();
void EXIT15_Init();
void TIM2_Init(uint32_t ms);
void ADC_Config();
uint16_t ADCvalue_convert(uint16_t adcValue);

void function_goto(int (*pfunction)());

int main(void);

#endif