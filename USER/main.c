
/*

如果头文件飘红，那不是工程问题，那是因为vscode的搜索路径是我的电脑上的路径，
你们的电脑上的路径不一样，所以vscode找不到头文件，所以头文件飘红，这时只要创建一个新窗口重新打开keil工程，
然后在右下角的弹出框点OK，就会恢复成你自己电脑的路径了，这时头文件就不会飘红了
然后如果弹出：在父目录找到了git仓库，是否打开，点打开就行了

*/

#include "stm32f4xx.h"
#include "bsp.h"
#include <OLED.h>
#include <usart.h>
#include <delay.h>
#include <shell.h>
#include <mpu6050.h>
#include <inv_mpu.h>
#include <control.h>
#include "Key_16.h"

//定义三轴角度
float pitch,roll,yaw;
//定义任意角
float def;
//定义位移
float dis;
//定义极坐标角度
float polar;
//得到的极坐标的原始角度
float Opolar;
extern float target_angle;
//定义角度的绝对值
float absroll,abspitch;
//定义模式选择
extern uint8_t mode;
//任意位移和角度
extern float Target_dis,Target_angle;
extern int Num;
int HFY=0;



/*

每一步都有都有调试信息，妈妈再也不用担心我找不出程序bug了

*/
int CH451_SetNum(uint8_t x,uint8_t y);
float CH451_Setdis(uint8_t x,uint8_t y);

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


    while (1)
    {
		OLED_Clear();
		OLED_Printf(0,0,OLED_6X8,"pitch:%f",pitch);
		OLED_Printf(0,16,OLED_6X8,"roll:%f",roll);
		OLED_Printf(0,32,OLED_6X8,"def:%f",def);
		OLED_Update();
		Target_dis=0.15;
		
		switch (ch451_read())
		{ 
	// ************模式1~2**********************
			case 14:  //模式1~2
			do
			{
				OLED_Clear();
				OLED_Printf(0,10,OLED_8X16,"angle:%f",Target_angle);
				OLED_Printf(0,40,OLED_8X16,"dis:%f",Target_dis);
				OLED_Update();
				HFY=1;
				Target_angle=0;
				
				if(ch451_read()==1|| ch451_read()==2||ch451_read()==3)
				{
					delay_ms(50);
					Target_dis = CH451_Setdis(0,0);
					mode=1;
				}
			} while (ch451_read()!=15);
			HFY=0;
			mode=0;
			Target_angle=0;
			Target_dis=0;
			break;
	// ************模式3**********************
			case 13:  //模式3
			do
			{
				OLED_Clear();
				OLED_Printf(0,10,OLED_8X16,"angle:%f",Target_angle);
				OLED_Printf(0,40,OLED_8X16,"dis:%f",Target_dis);
				OLED_Update();
				HFY=1;
				
				if(ch451_read()==0||ch451_read()==1)//角度的"1"用81的返回值
				{
						delay_ms(50);
						Target_angle = CH451_SetNum(0,0);
						delay_ms(100);
						mode=3;
				}
				else if(ch451_read()==1|| ch451_read()==2||ch451_read()==3)//半径的"1"用66的返回值
				{
					delay_ms(50);
					Target_dis = CH451_Setdis(0,0);
					delay_ms(100);
					mode=3;
}
			} while (ch451_read()!=15); HFY=0;
			mode=0;
			Target_angle=0;
			Target_dis=0;
			break;
	// ************模式4**********************	
				case 12: //模式4
				do
				{
					OLED_Clear();
					OLED_Printf(0,0,OLED_8X16,"dis:%d",dis);
					OLED_Update();
					HFY=1;					
					mode=4;
					MotorState(pitch,roll);
				} while (ch451_read()!=15);HFY=0;
				mode=0;
				Target_angle=0;
				Target_dis=0;
				break;
	// ************模式五**********************		
			case 11:  //模式5
			do
			{
				OLED_Clear();
				OLED_Printf(0,20,OLED_6X8,"pitch:%f",pitch);
				OLED_Printf(0,40,OLED_8X16,"dis:%f",Target_dis);
				OLED_Update();
		
				
				HFY=1;
				if(ch451_read()==1||ch451_read()==2||ch451_read()==3)
				{
					delay_ms(50);
					Target_dis = CH451_Setdis(0,0);
					delay_ms(50);
					mode=5;
					TIM_Cmd(TIM2,ENABLE);
				}
			} while (ch451_read()!=15); HFY=0;
			mode=0;
			TIM_Cmd(TIM2,DISABLE);
			Target_angle=0;
			Target_dis=0;
			break;
				
		}
    }
}

/*输入角度的值*/
int CH451_SetNum(uint8_t x,uint8_t y)
{
	int ten=0 ,ge=0,bai=0;
	
		//输入百位
	do
	{
		bai = ch451_read();
		if(bai == 14);
			//OLED_ShowNum(x,y,0,1,16,1);
		else
		OLED_ShowNum(x,y,bai,1,OLED_8X16);
		OLED_Update();
	}while(bai==14);
	delay_ms(250);
	
	//输入十位
	do
	{
		ten = ch451_read();
		if(ten == 99);
			//OLED_ShowNum(x,y,0,1,16,1);
		else
		OLED_ShowNum(x,y,ten,1,OLED_8X16);
		OLED_Update();
	}while(ten==99);
	delay_ms(250);
	//输入个位
	do
	{
		ge = ch451_read();
		if(ge == 99);
			//OLED_ShowNum(x+8,y,0,1,16,1);
		else
	OLED_ShowNum(x,y,ge,1,OLED_8X16);
	OLED_Update();
	}while(ge==99);
		delay_ms(500);
	return bai*100+ ten*10+ge;
}	

/************************************/

/*输入半径的值*/
float CH451_Setdis(uint8_t x,uint8_t y)
{
	int ten=0 ,ge=0;
	
	//输入十位
	do
	{
		ten = ch451_read();
		if(ten == 99);
			//OLED_ShowNum(x,y,0,1,16,1);
		else
		OLED_ShowNum(x,y,ten,2,OLED_8X16);
		OLED_Update();
	}while(ten==99);
	delay_ms(250);
	//输入个位
	do
	{
		ge = ch451_read();
		if(ge == 99);
			//OLED_ShowNum(x+8,y,0,1,16,1);
		else
	OLED_ShowNum(x,y,ge,2,OLED_8X16);
	OLED_Update();
	}while(ge==99);
		delay_ms(500);
	return ten*0.1+ge*0.01;
}	