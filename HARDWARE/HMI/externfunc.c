/*

    这里搞了一些HMI的外部函数

*/


#include <hmi.h>

extern uint8_t mode;
extern uint16_t Res;
extern float Target_dis,Target_angle,target_R;

uint8_t AngleSet=0;

//通用串口屏协议
void HMI_ResCheck()
{
    /************初始页面选择模式*********** */
    switch (Res)
    {
        case 0x31:  //进入模式1-2
        {
            /*在这些地方随便改*/
            mode = 1;
            Target_angle=0;
            Target_dis=0.15;
        }break;

        case 0x33:  //进入模式3的角度设置
        {
            AngleSet=1;
            OLED_Clear();

        }break;
        
        case 0x34:  //进入模式4
        {
            mode = 4;
        }break;

        case 0x35:  //进入模式5
        {
            mode = 5;
            target_R=0.15;
        }break;

        default:break;
    }

}



void HMI_Mode1_2()
{
    char buf[50];
    if(mode==1)
    {
        switch (Res)
        {
            case 0x41:  //半径增加
            {
                Target_dis += 0.05;
                sprintf(buf,"page1.t2.txt=\"%d\"",(uint16_t)(Target_dis*100));
                HMISends(USART_PORT_2,buf);		
                HMISendb(USART_PORT_2,0xff);
            }break;
            
            case 0x42:  //半径减小
            {
                Target_dis -= 0.05;
                sprintf(buf,"page1.t2.txt=\"%d\"",(uint16_t)(Target_dis*100));
                HMISends(USART_PORT_2,buf);		
                HMISendb(USART_PORT_2,0xff);
            }break;
                
            case 0x43:  //返回初始页面
            {
                mode = 0;
            }break;
            
            default:
                break;
        }
    }
}




void HMI_Mode3()
{
    char buf[50];
    if((mode==3)||(AngleSet))
    {
        switch (Res)
        {
            case 0x52:  
            {
                Target_angle = 0;   //角度清零,跑模式1
                mode=1;
                sprintf(buf,"page2.t3.txt=\"%d\"",(uint16_t)Target_angle);
                HMISends(USART_PORT_2,buf);		
                HMISendb(USART_PORT_2,0xff);
                AngleSet=0;
            }break;
            
            case 0x53:  
            {
                Target_angle = 30;
                mode=3;
                sprintf(buf,"page2.t3.txt=\"%d\"",(uint16_t)Target_angle);
                HMISends(USART_PORT_2,buf);		
                HMISendb(USART_PORT_2,0xff);
                AngleSet=0;
            }break;
                
            case 0x54:  
            {
                Target_angle = 45;
                mode=3;
                sprintf(buf,"page2.t3.txt=\"%d\"",(uint16_t)Target_angle);
                HMISends(USART_PORT_2,buf);		
                HMISendb(USART_PORT_2,0xff);
                AngleSet=0;
            }break;

            case 0x55:  
            {
                Target_angle = 60;
                mode=3;
                sprintf(buf,"page2.t3.txt=\"%d\"",(uint16_t)Target_angle);
                HMISends(USART_PORT_2,buf);		
                HMISendb(USART_PORT_2,0xff);
                AngleSet=0;
            }break;

            case 0x56:  
            {
                Target_angle = 90;
                mode=3;
                sprintf(buf,"page2.t3.txt=\"%d\"",(uint16_t)Target_angle);
                HMISends(USART_PORT_2,buf);		
                HMISendb(USART_PORT_2,0xff);
                AngleSet=0;
            }break;

            case 0x57:  
            {
                Target_angle = 120;
                mode=3;
                sprintf(buf,"page2.t3.txt=\"%d\"",(uint16_t)Target_angle);
                HMISends(USART_PORT_2,buf);		
                HMISendb(USART_PORT_2,0xff);
                AngleSet=0;
            }break;

            case 0x58:  
            {
                Target_angle = 135;
                mode=3;
                AngleSet=0;
                sprintf(buf,"page2.t3.txt=\"%d\"",(uint16_t)Target_angle);
                HMISends(USART_PORT_2,buf);		
                HMISendb(USART_PORT_2,0xff);
                
            }break;

            case 0x59:  
            {
                Target_angle = 150;
                mode=3;
                sprintf(buf,"page2.t3.txt=\"%d\"",(uint16_t)Target_angle);
                HMISends(USART_PORT_2,buf);		
                HMISendb(USART_PORT_2,0xff);
                AngleSet=0;
            }break;

            case 0x51:  //返回初始页面
            {
                mode=0;
            }break;
            
            default:
                break;
        }
    }
}


void HMI_Mode4()
{
    if(mode==4)
    {
        switch (Res)
        {
            case 0x71:  //返回初始页面
            {
                mode=0;
            }break;
            
            default:
                break;
        }
    }
}

void HMI_Mode5()
{
    char buf[50];
    if(mode==5)
    {
        switch (Res)
        {
            case 0x61:  //半径增加
            {
                target_R += 0.05;
                sprintf(buf,"page4.t2.txt=\"%d\"",(uint16_t)(target_R*100));
                HMISends(USART_PORT_2,buf);		
                HMISendb(USART_PORT_2,0xff);
            }break;
            
            case 0x62:  //半径减小
            {
                target_R -= 0.05;
                sprintf(buf,"page4.t2.txt=\"%d\"",(uint16_t)(target_R*100));
                HMISends(USART_PORT_2,buf);		
                HMISendb(USART_PORT_2,0xff);
            }break;

            case 0x63:  //返回初始页面
            {
                mode=0;
            }break;

            default:
                break;
        }
    }
}