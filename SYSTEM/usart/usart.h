#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "stm32f4xx_conf.h"
#include "sys.h" 


#define USART_REC_LEN  			    200  	//定义最大接收字节数 200
#define EN_USART1_IT_RX 			1		//使能（1）/禁止（0）串口1接收
	  	
extern u8  USART_RX_BUF[USART_REC_LEN]; //接收缓冲,最大USART_REC_LEN个字节.末字节为换行符 
extern u16 USART_RX_STA;         		//接收状态标记	
//如果想串口中断接收，请不要注释以下宏定义
void usart1_init(u32 bound);
void SendTo429(uint16_t *dataf);
void USART_Send_Data(USART_TypeDef *USARTx, char data);
void USART_Send_String(USART_TypeDef *USARTx,char *String);
void U2printf(char *format, ...);
#endif

