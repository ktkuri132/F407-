#include "sys.h"
#include "usart.h"	
#include "bsp.h"
#include "stdarg.h"

/// @brief 专门用于蓝牙串口2的printf
/// @param format 字符串
/// @param  像普通printf一样用就行了
void U2printf(char *format, ...)
{
	char String[256];						
	va_list arg;							
	va_start(arg, format);					
	vsprintf(String, format, arg);			
	va_end(arg);			
	USART_Send_String(USART2,String);
}

#ifdef __GNUC__		/* GCC编译下的printf调用具有延迟，会在多次进入中断接收数据时漏掉数据  */
__asm (".global __use_no_semihosting\n\t");

int _write (int fd, char *pBuffer, int size)
{
    for (int i = 0; i < size; i++)
    {
        while((USART1->SR&0X40)==0);//等待上一次串口数据发送完成
        USART1->DR = (uint8_t) pBuffer[i];       //写DR,串口1将发送数据
    }
    return size;
}


#elif __CC_ARM

#pragma import(__use_no_semihosting)

struct __FILE
{
    int handle;
    /* Whatever you require here. If the only file you are using is */
    /* standard output using printf() for debugging, no file handling */
    /* is required. */
};



int _ttywrch(int ch)
{
    ch = ch;
    return ch;
}


void _sys_exit(int x)
{
    x = x;
}

char *_sys_command_string(char *cmd, int len)
{
    return NULL;
}


FILE __stdout;
FILE __stdin;



int fputc(int ch, FILE *f)
{
    while ((USART1->SR & 0X40) == 0);     

    USART1->DR = (uint8_t)ch;             
    return ch;
}

int fgetc(FILE *f)
{
  while(USART_GetFlagStatus(UART4, USART_FLAG_RXNE) == RESET);
	USART_ClearFlag(USART1,USART_FLAG_RXNE);
  return (int)USART_ReceiveData(USART1);
}


#endif

uint8_t Stop_flag=0;

extern float Target_dis,target_R;

//串口1中断服务程序 	
u8 USART_RX_BUF[USART_REC_LEN];     
u16 USART_RX_STA=0;       


void usart1_init(u32 bound){
   	//GPIO端口设置
  	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //使能GPIOA时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//使能USART1时钟
 
	//串口1对应引脚复用映射
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); //GPIOA9复用为USART1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); //GPIOA10复用为USART1
	
	//USART1端口配置
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; //GPIOA9与GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOA,&GPIO_InitStructure); //初始化PA9，PA10

   //USART1 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  	USART_Init(USART1, &USART_InitStructure); //初始化串口1
	
  	USART_Cmd(USART1, ENABLE);  //使能串口1 
	

	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启相关中断

	
}



void usart2_init(u32 bound)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //使能GPIOA时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);//使能USART2时钟

	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2); //GPIOA2复用为USART2->TX
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2); //GPIOA3复用为USART2->RX

	GPIO_Config(GPIOA,GPIO_Pin_2|GPIO_Pin_3,GPIO_Mode_AF,GPIO_PuPd_UP);

	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = bound;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  	USART_Init(USART2, &USART_InitStructure); //初始化串口1
	
  	USART_Cmd(USART2, ENABLE);  //使能串口1 
	
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//开启相关中断

}

//串口1中断服务程序-->用于串口打印
void USART1_IRQHandler(void)          	
{
	
	u8 Res;
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
	{
		Res =USART_ReceiveData(USART1);//(USART1->DR);	//读取接收到的数据
		
		if((USART_RX_STA&0x8000)==0)//接收未完成
		{
			if(USART_RX_STA&0x4000)//接收到了0x0d
			{
				if(Res!=0x0a)USART_RX_STA=0;//接收错误,重新开始 
				else USART_RX_STA|=0x8000;	//接收完成了 
		
			}
			else //还没收到0X0D
			{	
				if(Res==0x0d)USART_RX_STA|=0x4000;
				else
				{
					USART_RX_BUF[USART_RX_STA&0X3FFF]=Res ;
					USART_RX_STA++;
					if(USART_RX_STA>(USART_REC_LEN-1))USART_RX_STA=0;//接收数据错误,重新开始接收	  
				}		 
			}
		}   	
  	} 
} 

/* 串口发送函数  */
void USART_Send_Data(USART_TypeDef *USARTx, char data)
{
	USART_SendData(USARTx,data);
	while(USART_GetFlagStatus(USARTx,USART_FLAG_TC)!=1);
}

/* 串口发送字符串函数  */
void USART_Send_String(USART_TypeDef *USARTx,char *String)
{
	u16 len,j;
	
	len=strlen(String);
	for(j=0;j<len;j++)
	{
		USART_Send_Data(USARTx, *String++);
	}
}

#ifndef __HMI_H_

void SendTo429(uint16_t *dataf)
{
	USART_Send_Data(USART1,0x01);	//起始帧

	USART_Send_String(USART1,(char *)&dataf);	//发送数据

	USART_Send_Data(USART1,0x02);	//结束帧
	
}

//串口2中断函数
void USART2_IRQHandler(void)
{
	static uint16_t Res;
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
	{
		Res =USART_ReceiveData(USART2);
		switch (Res)
		{
			case 'a':
			{
				Target_dis +=0.05;
				U2printf("Target_dis:%f\n",Target_dis);
			}	break;

			case 'b':
			{
				Target_dis -=0.05;
				U2printf("Target_dis:%f\n",Target_dis);
			}	break;
			
			case 'c':
			{
				target_R +=0.05;
				U2printf("R:%f\n",target_R);
			}
			default:
				break;
		}

		USART_ClearITPendingBit(USART2,USART_IT_RXNE);
	}
}



#endif

