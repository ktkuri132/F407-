#if !defined __BSP__
#define __BSP__

#include <OLED.h>



void LED_Init();
void GPIO_Config(GPIO_TypeDef *GPIOx,uint32_t GPIO_Pin_x,uint32_t GPIO_Mode_x,uint32_t GPIO_PuPd_x);
int BSP_Init();
void BSP_NVIC_IT_Config();
void EXIT15_Init();
void ADC_Config();
uint16_t ADCvalue_convert(uint16_t adcValue);
#endif