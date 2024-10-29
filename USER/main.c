
/*

如果头文件�?�红，那不是工程�?题，那是因为vscode的搜索路径是我的电脑上的�?径，
你们的电脑上的路径不一样，所�?vscode找不到头文件，所以头文件飘红，这时只要创建一�?新窗口重新打开keil工程�?
然后在右下�?�的弹出框点OK，就会恢复成你自己电脑的�?径了，这时头文件就不会�?�红�?
然后如果弹出：在父目录找到了git仓库，是否打开，点打开就�?�了

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

//定义三轴角度
float pitch,roll,yaw;
//定义任意�?
float def;
//定义位移
float dis;
//定义极坐标�?�度
float polar;
//得到的极坐标的原始�?�度
float Opolar;
extern float target_angle;
//定义角度的绝对�?
float absroll,abspitch;
//定义模式选择
extern uint8_t mode;
/*

每一步都有都有调试信�?，�?��?�再也不用担心我找不出程序bug�?

*/
int main(void) 
{

Init:
    /* 所有�?��?�的初�?�化函数都在这个里面�?  */
    if(BSP_Init())
    {
        printf("peripheral init failed\n");
        
    }
    else
    {
        printf("peripheral init done\n");
    }
    OLED_Clear();
    
    
/*
    TIM8->CCR1=600;//水平方向内侧电机
    TIM8->CCR2=600;//水平方向外侧电机
    TIM8->CCR3=600;//垂直方向内侧电机
    TIM8->CCR4=600;//垂直方向外侧电机

    LIN1=1;//水平方向内侧电机
    LIN2=0;

    LIN3=0;//水平方向外侧电机
    LIN4=1;

    RIN1=0;//垂直方向外侧电机
    RIN2=1;

    RIN3=0;//垂直方向内侧电机
    RIN4=1;

 */
    
strat:
    OLED_Clear();
    while (1)
    {
        OLED_Printf(0,0,OLED_8X16,"Mode chose...");
        OLED_Update();
        delay_ms(500);
        OLED_Printf(0,0,OLED_8X16,"             ");
        OLED_Update();
        delay_ms(500);
        if(mode)
        {
            goto next;
        }        
    }

next:
    if(mode==5)
    {
        TIM_Cmd(TIM2,ENABLE);
    }

    OLED_Clear();
    /* 前台程序�?�?  */
    while (1)
    {
        if(mode==4)
        {
            MotorState(pitch,roll);
        }
        while (!mode)
        {
            TIM_Cmd(TIM2,DISABLE);
            goto strat;
        }
        if((!pitch)&&(!roll))
        {
            OLED_Clear();
            OLED_Printf(0,0,OLED_8X16,"MPU6050 error");
            OLED_Printf(0,16,OLED_8X16,"Trying to Reset");
            OLED_Update();
            delay_ms(1000);
            return 0;
            
        }
        OLED_Printf(0,0,OLED_6X8,"pitch:%f",pitch);
        OLED_Update();
    }
    
}





