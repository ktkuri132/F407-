#ifndef __HMI_H_
#define __HMI_H_

//�ӿ�ͷ�ļ�
#include <stm32f4xx.h>
#include <usart.h>  


//������STM32F103����STM32F407��Ҫ����ʵ�ֵײ�ӿ�
#if !defined (STM32F10X_MD) && !defined (STM32F40_41xxx)
    
#define USART_TypeDef
#define delay_ms(x)
#define USART_SendData(x,y)
#define USART_GetFlagStatus(x,y)
#define USART1_IRQHandler()
#define USART_GetITStatus(x,y)

#else

#define __STM32

//#define USART_PORT_1 USART1
#define USART_PORT_2 USART2
//#define USART_PORT_3 USART3

void HMISendstart(USART_TypeDef* USARTx);

void HMI_ResCheck();
void HMI_Mode1_2();
void HMI_Mode3();
void HMI_Mode4();
void HMI_Mode5();



#endif

#endif