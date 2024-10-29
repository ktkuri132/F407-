/*

    �������һЩHMI���ⲿ����

*/


#include <hmi.h>

extern uint8_t mode;
extern uint16_t Res;
extern float Target_dis,Target_angle,target_R;


//ͨ�ô�����Э��
void HMI_ResCheck()
{
    /************��ʼҳ��ѡ��ģʽ*********** */
    switch (Res)
    {
        case 0x31:  //����ģʽ1-2
        {
            /*����Щ�ط�����*/
            mode = 1;
            Target_angle=0;
            Target_dis=0.15;
        }break;

        case 0x33:  //����ģʽ3�ĽǶ�����
        {
            mode = 3;
            Target_angle=30;
            Target_dis=0.15;
        }break;
        
        case 0x34:  //����ģʽ4
        {
            mode = 4;
        }break;

        case 0x35:  //����ģʽ5
        {
            mode = 5;
            Target_dis=0.15;
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
            case 0x41:  //�뾶����
            {
                Target_dis += 0.05;
                sprintf(buf,"page1.t2.txt=\"%d\"",(uint16_t)(Target_dis*100));
                HMISends(USART_PORT_2,buf);		
                HMISendb(USART_PORT_2,0xff);
            }break;
            
            case 0x42:  //�뾶��С
            {
                Target_dis -= 0.05;
                sprintf(buf,"page1.t2.txt=\"%d\"",(uint16_t)(Target_dis*100));
                HMISends(USART_PORT_2,buf);		
                HMISendb(USART_PORT_2,0xff);
            }break;
                
            case 0x43:  //���س�ʼҳ��
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
            case 0x52:  //�Ƕ�����
            {
                Target_angle += 5;
                sprintf(buf,"page2.t3.txt=\"%d\"",(uint16_t)Target_angle);
                HMISends(USART_PORT_2,buf);		
                HMISendb(USART_PORT_2,0xff);
            }break;

            case 0x53:  //�Ƕȼ�С
            {
                Target_angle -= 5;
                sprintf(buf,"page2.t3.txt=\"%d\"",(uint16_t)Target_angle);
                HMISends(USART_PORT_2,buf);		
                HMISendb(USART_PORT_2,0xff);
            }break;

            case 0x51:  //���س�ʼҳ��
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
            case 0x71:  //���س�ʼҳ��
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
            case 0x61:  //�뾶����
            {
                Target_dis += 0.05;
                sprintf(buf,"page4.t2.txt=\"%d\"",(uint16_t)(Target_dis*100));
                HMISends(USART_PORT_2,buf);		
                HMISendb(USART_PORT_2,0xff);
            }break;
            
            case 0x62:  //�뾶��С
            {
                Target_dis -= 0.05;
                sprintf(buf,"page4.t2.txt=\"%d\"",(uint16_t)(Target_dis*100));
                HMISends(USART_PORT_2,buf);		
                HMISendb(USART_PORT_2,0xff);
            }break;

            case 0x63:  //���س�ʼҳ��
            {
                mode=0;
            }break;

            default:
                break;
        }
    }
}