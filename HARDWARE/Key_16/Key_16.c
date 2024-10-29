#include "Key_16.h"
#include "sys.h"
#include "delay.h"
#include "usart.h"

Question    My = {
	
	0,        // Q_Start;
	0,         //My.Mode
	0,
};

int key_num = 20;

void ch451_write(u16 command)
{
   u8 i;
   load_0;   
   for(i=0;i<12;i++)           //写12位命令  
   {
      if(command&0x0001) din_1;
      else               din_0;
      dclk_0;                  //给其一个上升沿
      dclk_1;
      command>>=1;
   }
   load_1;                         //加载进去
}



void ch451_init(void )
{
	GPIO_InitTypeDef  GPIO_InitStructure;	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15|GPIO_Pin_13|GPIO_Pin_11;	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;     // 普通输出模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;    //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;      //上拉
	GPIO_Init(GPIOE, &GPIO_InitStructure);     //初始化   GPIO
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;		
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;  
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;  	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;      //上拉
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	din_0;
  din_1;  
  ch451_write(0x0201);//芯片内部复位
  ch451_write(0x0403);	
}


u16 ch451_read(void)
{  
//	 float PID_OUT;
   u8 i,keycode=0;
   u16 command;
	 u8 t=0;
   command=0x07;            
   load_0;                
   for(i=0;i<4;i++)
   {
      if(command&0x0001) din_1;
      else               din_0;                //读取前四位按键代码命令
      dclk_0;
      dclk_1;
      command>>=1;
   }
   load_1;
   for(i=0;i<7;i++)
   {
      keycode<<=1;
      if(dout) keycode|=0X01;        //串口接收数据
      dclk_1;
      dclk_0; 
   }
   switch(keycode)
   {   /*方向是列*/
      case 67:t=0;;break;
      case 66:t=1;break;
      case 65:t=2;break;
      case 64:t=3;break;
		 
      case 75:t=4;break;
      case 74:t=5;break;
      case 73:t=6;break;
      case 72:t=7;break;
		 
      case 83:t=8;break;
      case 82:t=9;break;
      case 81:t=10;break;//给到100~180
      case 80:t=11;break;
		 
      case 91:t=12;break;
      case 90:t=13;break;
      case 89:t=14;break;
      case 88:t=15;break;
			default:t=99;
   }
	  t=t;
    return t; 	
}

