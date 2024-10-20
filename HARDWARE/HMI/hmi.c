/*

串口屏协议分析：接口文件


*/

#include <hmi.h>

extern uint8_t mode;
uint16_t Res;


void HMISendstart(USART_TypeDef* USARTx)
{
	delay_ms(200);
	HMISendb(USARTx,0xff);
	delay_ms(200);
}


void HMISendb(USART_TypeDef* USARTx,u8 k)		         
{		 
	u8 i;
	 for(i=0;i<3;i++)
	 {
		if(k!=0)
		{
			USART_SendData(USARTx,k);  
			while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET){};
		}
		else
		{
			return ;
		}
	 } 
} 


void HMISends(USART_TypeDef* USARTx, char *buf1)		  
{
	u8 i=0;
	while(1)
	{
		if(buf1[i]!=0)
		{
			USART_SendData(USARTx,buf1[i]);  
			while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET){};
			i++;
		}
		else
		{
			return ;
		}
	}
}

#ifdef __STM32

#if defined (USART_PORT_1)
void USART1_IRQHandler(void)
{
	
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
	    static int a=0;
		USART_ClearFlag(USART1, USART_FLAG_RXNE);
	    char buf[50];
		Res=USART_ReceiveData(USART1);
      	if(Res==0x31)
		{	
			sprintf(buf,"page0.b0.txt=\"time++\"");
			HMISends(USART_PORT_1,buf);		
			HMISendb(USART_PORT_1,0xff);
			
			
			
		}
		else if(Res==0x32)
		{
			sprintf(buf,"page0.b1.txt=\"time--\"");
			HMISends(USART_PORT_1,buf);		
			HMISendb(USART_PORT_1,0xff);
			
			
		}
		
	}
}

#elif defined (USART_PORT_2)

void USART2_IRQHandler(void)
{
    
    if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
    {
        USART_ClearFlag(USART2, USART_FLAG_RXNE);
        Res=USART_ReceiveData(USART2);

		HMI_ResCheck();
		HMI_Mode1_2();
		HMI_Mode3();
		HMI_Mode4();
		HMI_Mode5();

        
    }
}

#elif defined (USART_PORT_3)
void USART3_IRQHandler(void)
{
    
    if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
    {
        static int a=0;
        USART_ClearFlag(USART3, USART_FLAG_RXNE);
        char buf[50];
        Res=USART_ReceiveData(USART3);
      	if(Res==0x31)
        {	
            sprintf(buf,"page0.b0.txt=\"time++\"");
            HMISends(USART_PORT_3,buf);		
            HMISendb(USART_PORT_3,0xff);
            
            
            
        }
        else if(Res==0x32)
        {
            sprintf(buf,"page0.b1.txt=\"time--\"");
            HMISends(USART_PORT_3,buf);		
            HMISendb(USART_PORT_3,0xff);
            
            
        }
        
    }
}

#endif

#endif

