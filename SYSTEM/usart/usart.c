#include "sys.h"
#include "usart.h"	
#include "bsp.h"
#include "stdarg.h"

/// @brief ר��������������2��printf
/// @param format �ַ���
/// @param  ����ͨprintfһ���þ�����
void U2printf(char *format, ...)
{
	char String[256];						
	va_list arg;							
	va_start(arg, format);					
	vsprintf(String, format, arg);			
	va_end(arg);			
	USART_Send_String(USART2,String);
}

#ifdef __GNUC__		/* GCC�����µ�printf���þ����ӳ٣����ڶ�ν����жϽ�������ʱ©������  */
__asm (".global __use_no_semihosting\n\t");

int _write (int fd, char *pBuffer, int size)
{
    for (int i = 0; i < size; i++)
    {
        while((USART1->SR&0X40)==0);//�ȴ���һ�δ������ݷ������
        USART1->DR = (uint8_t) pBuffer[i];       //дDR,����1����������
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

extern float Target_dis;

//����1�жϷ������ 	
u8 USART_RX_BUF[USART_REC_LEN];     
u16 USART_RX_STA=0;       


void usart1_init(u32 bound){
   	//GPIO�˿�����
  	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //ʹ��GPIOAʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//ʹ��USART1ʱ��
 
	//����1��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); //GPIOA9����ΪUSART1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); //GPIOA10����ΪUSART1
	
	//USART1�˿�����
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; //GPIOA9��GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOA,&GPIO_InitStructure); //��ʼ��PA9��PA10

   //USART1 ��ʼ������
	USART_InitStructure.USART_BaudRate = bound;//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
  	USART_Init(USART1, &USART_InitStructure); //��ʼ������1
	
  	USART_Cmd(USART1, ENABLE);  //ʹ�ܴ���1 
	

	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//��������ж�

	
}



void usart2_init(u32 bound)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //ʹ��GPIOAʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);//ʹ��USART2ʱ��

	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2); //GPIOA2����ΪUSART2->TX
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2); //GPIOA3����ΪUSART2->RX

	GPIO_Config(GPIOA,GPIO_Pin_2|GPIO_Pin_3,GPIO_Mode_AF,GPIO_PuPd_UP);

	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = bound;//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
  	USART_Init(USART2, &USART_InitStructure); //��ʼ������1
	
  	USART_Cmd(USART2, ENABLE);  //ʹ�ܴ���1 
	
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//��������ж�

}

//����1�жϷ������-->���ڴ��ڴ�ӡ
void USART1_IRQHandler(void)          	
{
	
	u8 Res;
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //�����ж�(���յ������ݱ�����0x0d 0x0a��β)
	{
		Res =USART_ReceiveData(USART1);//(USART1->DR);	//��ȡ���յ�������
		
		if((USART_RX_STA&0x8000)==0)//����δ���
		{
			if(USART_RX_STA&0x4000)//���յ���0x0d
			{
				if(Res!=0x0a)USART_RX_STA=0;//���մ���,���¿�ʼ 
				else USART_RX_STA|=0x8000;	//��������� 
		
			}
			else //��û�յ�0X0D
			{	
				if(Res==0x0d)USART_RX_STA|=0x4000;
				else
				{
					USART_RX_BUF[USART_RX_STA&0X3FFF]=Res ;
					USART_RX_STA++;
					if(USART_RX_STA>(USART_REC_LEN-1))USART_RX_STA=0;//�������ݴ���,���¿�ʼ����	  
				}		 
			}
		}   	
  	} 
} 


void SendTo429(uint16_t *dataf)
{
	USART_Send_Data(USART1,0x01);	//��ʼ֡

	USART_Send_String(USART1,(char *)&dataf);	//��������

	USART_Send_Data(USART1,0x02);	//����֡
	
}

//����2�жϺ���
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
				Target_dis +=0.01;
				U2printf("Target_dis:%f\n",Target_dis);
			}	break;

			
			
			case 'b':
			{
				Target_dis -=0.01;
				U2printf("Target_dis:%f\n",Target_dis);
			}	break;
			
			default:
				break;
		}

		USART_ClearITPendingBit(USART2,USART_IT_RXNE);
	}
}

/* ���ڷ��ͺ���  */
void USART_Send_Data(USART_TypeDef *USARTx, char data)
{
	USART_SendData(USARTx,data);
	while(USART_GetFlagStatus(USARTx,USART_FLAG_TC)!=1);
}

/* ���ڷ����ַ�������  */
void USART_Send_String(USART_TypeDef *USARTx,char *String)
{
	u16 len,j;
	
	len=strlen(String);
	for(j=0;j<len;j++)
	{
		USART_Send_Data(USARTx, *String++);
	}
}



