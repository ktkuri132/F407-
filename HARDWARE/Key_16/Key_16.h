#ifndef __KEY16_H
#define __KEY16_H
#include "sys.h"

//按键的GUI
typedef struct
{
	u16    Q_Start;
	u16    Mode;         //问题选择标志位
	u16    POS_Sec;
	u8     save_flag;    //保存标志位
	u8     flash_start;
    u8     change_mode;    //改变服务函数
}Question;
extern  Question  My;



#define  dclk_0  GPIO_ResetBits(GPIOE,GPIO_Pin_15)//clk
#define  dclk_1  GPIO_SetBits(GPIOE,GPIO_Pin_15)
#define  din_0   GPIO_ResetBits(GPIOE,GPIO_Pin_13)//din
#define  din_1   GPIO_SetBits(GPIOE,GPIO_Pin_13)
#define  load_0  GPIO_ResetBits(GPIOE,GPIO_Pin_11)//load
#define  load_1  GPIO_SetBits(GPIOE,GPIO_Pin_11)
#define  dout    GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_9) //dout
void  ch451_write(u16 command);
void  ch451_init(void);
u16   ch451_read(void);
u8 get_val(u8 a);
u16 get_value(u8 mode);
void CH451_Get_val(void);      //界面和键盘
void Keyscan(void);

#endif
