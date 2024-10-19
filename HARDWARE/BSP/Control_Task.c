/*


����ļ� װ����  �����ڵĺ��Ĳ���---���Ʋ���



*/



#include <stm32f4xx.h>
#include <bsp.h>
#include <control.h>
#include <usart.h>
#include <mpu6050.h>
#include <inv_mpu.h>
#include <stm32f4xx_exti.h>
#include <OLED.h>
#include <math.h>


extern float pitch,roll,yaw,dis,def,polar,Opolar,absroll,abspitch;





/*

15cm 500,0,-65.5,00983
25cm 395,0,0.0001,0.0104223  <---��������ձ���

750,0,-350,0.0104223

(17+0.73333*Target_angle)
(140.625-Target_angle)/3.75+Target_angle

*/


//         30 45 60 150  135   120  90
int de[7]={0,-5,-7,+12.5,+10.5,+9.9,-10 };


float Target_dis=0.15;
float Target_angle = 0;
uint8_t mode=1;


/// @brief MPU6050��ֵ�������ⲿ�жϺ���      
void EXTI15_10_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line15) != RESET)
    {
        EXTI_ClearITPendingBit(EXTI_Line15);
        
        mpu_dmp_get_data(&pitch, &roll, &yaw);      //��ȡMPU6050����

        
        switch (mode)
        {

            case 5:break;

            case 1:
                Task1_LineMove(Target_dis,Target_angle);
                break;    
            
            case 3:
                Task3_AngleMove(Target_angle,Target_dis);
                break;

            case 4:
                Task4_StopFast();    //���ÿ��ƺ���--->��4��
                break;


            default:break;
                
        }

    }
}


float target_R=0.15;

/*
    ��ʱ��2�жϺ���,��ʱ5ms����һ��PID
*/
void TIM2_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
        
        Task5_CircleMove(target_R);
    }
}




/*
    ������  �����ƶ�
*/
void Task4_StopFast()
{
    GetPolar(roll,pitch);

    /*
    //��ֱ�ƶ����
    static float VerticalOutput;
    //ˮƽ�ƶ����
    static float LevelOutput;
    */
    static float Output;
    static struct PID Taks4Pid={900,-1,0,PidControl_Stop};
    
    Output = Taks4Pid.PIDControl(TargetDis,dis, &Taks4Pid);
    //printf("Output:%f\n",Output);
    PWM_Allocation(Output);
  
}




float target_angle;

/*
    ��һ,����  �����˶�+���ȿɵ�
*/
void Task1_LineMove(float R,uint8_t forward)
{
    static float VOutput,LOutput;
    static float A;
    static float time=0;
	static uint8_t flag_1=0,flag_2=0,flag_3=0;
    /*0�� - 15cm����*/
    static struct PID Taks1Pid1={750,0,-295,PidControl_LineMove};
    /*0�� - 20~25cm*/
    static struct PID Taks1Pid2={950,0,-320,PidControl_LineMove};
    /*90�� - 15cm*/
    static struct PID Taks1Pid3={720,0,-420,PidControl_LineMove};
    /*90�� - 20~25cm*/
    static struct PID Taks1Pid4={830,0,-200,PidControl_LineMove};


    A = atanf(R/0.86)*180.0f/PI;
    target_angle = A*sinf(2*PI*time/T);
    
    //printf("%f,%f\r\n",target_angle,roll);
    if(forward==0)
    {
        if(R<=0.16)
        {
            VOutput = Taks1Pid1.PIDControl(target_angle,roll, &Taks1Pid1);
            LOutput = Taks1Pid1.PIDControl(0,pitch, &Taks1Pid1);
        }
        else if(R>=0.17)
        {
            VOutput = Taks1Pid2.PIDControl(target_angle,roll, &Taks1Pid2);
            LOutput = Taks1Pid2.PIDControl(0,pitch, &Taks1Pid2);
        }
        
    }
    

//0.00983---0.15
    time+=0.0104220;//��λ����λ
    if(time<T)
    {
        if(forward==0)
        {
            if(VOutput>0)
            {
                Motor_Cmd(VerticalOut,ENABLE);
                Motor_Cmd(VerticalIn,DISABLE);
                Motor->MVerticalOut =(uint32_t)VOutput;
            }
            else
            {
                Motor_Cmd(VerticalOut,DISABLE);
                Motor_Cmd(VerticalIn,ENABLE);
                Motor->MVerticalIn =(uint32_t)(-VOutput);
            }
        }
    }
    else time=0;
    
}



/*
    ������  �Ƕȿɵ��ĵ����˶�
*/
void Task3_AngleMove(float angle,float R)
{

    static float VOutput,LOutput;
    static float target_roll,target_pitch;
    static float time=0;
    static float Vtarget_angle,Ltarget_angle;
    
    static struct PID Taks3Pid1={380,0,-110,PidControl_LineMove};
    static struct  PID Taks3Pid2={380,0,-110,PidControl_LineMove};
    
    static struct PID Taks3Pid3={340,0,-90,PidControl_LineMove};
    static struct  PID Taks3Pid4={340,0,-90,PidControl_LineMove};

    static struct PID Taks3Pid5={380,0,-110,PidControl_LineMove};
    static struct  PID Taks3Pid6={380,0,-110,PidControl_LineMove};

    static struct PID Taks3Pid7={340,0,-90,PidControl_LineMove};
    static struct  PID Taks3Pid8={340,0,-90,PidControl_LineMove};

    int a;
    if(angle>90)
    {   
        //angle = (140.625-angle)/3.75+angle,0.15;
        angle = 180 - angle;
        //�����������Ϊ�����Ͷ�ֵ��ȷ��
        if(angle<=31)   
        {
            angle += de[3];
        }
        else if((angle>31)&&(angle<=46))
        {
            angle += de[4];
        }
        else if((angle>46)&&(angle<=61))
        {
            angle += de[5];
        }
        a=2;
    }
    
    else if(angle<90)
    {
        //angle = (17+0.73333*angle);
        if(angle<=31)
        {
            angle += de[0];
        }
        else if((angle>31)&&(angle<=46))
        {
            angle += de[1];
        }
        else if((angle>46)&&(angle<=61))
        {
            angle += de[2];
        }

        a=1;
    }
    else
    {
        angle +=de[6];
    }
    
   // angle = angle+5;
    angle = angle*PI/180.0f;

    target_roll = atanf(R*cosf(angle)/0.86)/PI*180;
    target_pitch = atanf(R*sinf(angle)/0.86)/PI*180;
	
//	printf("%f,%f\r\n",A1,A2);

    float wt = 2*PI*(time/T);

    Vtarget_angle =sinf(wt)*target_roll;
    Ltarget_angle =sinf(wt)*target_pitch;
 
    angle = (angle * 180) / PI;
	//printf("%f,%f\r\n",VOutput,LOutput);
/**************************************С��90��**************************************************************** */
    if(a==1)
    {
        printf("%f,%f\r\n",angle,PI/6);
        if(angle<=30+de[0])
        {
            VOutput = Taks3Pid1.PIDControl(Vtarget_angle,roll,&Taks3Pid1);
            LOutput = Taks3Pid2.PIDControl(Ltarget_angle,-pitch,&Taks3Pid2);

            time +=0.0104223;
            if(time<T)
            {
                T3Motor_CmdCombination(VOutput,LOutput,1);
            }
            else
            {
                time=0;
            }
        }
        else if((angle>30+de[0])&&(angle<=45+de[1]))
        {
            VOutput = Taks3Pid3.PIDControl(Vtarget_angle,roll,&Taks3Pid3);
            LOutput = Taks3Pid4.PIDControl(Ltarget_angle,-pitch,&Taks3Pid4);

            time +=0.0101223;
            if(time<T)
            {
                T3Motor_CmdCombination(VOutput,LOutput,1);
            }
            else
            {
                time=0;
            }
        }
        else if((angle>45+de[1])&&(angle<=60+de[2]))
        {
            VOutput = Taks3Pid5.PIDControl(Vtarget_angle,roll,&Taks3Pid5);
            LOutput = Taks3Pid6.PIDControl(Ltarget_angle,-pitch,&Taks3Pid6);

            time +=0.0102323;
            if(time<T)
            {
                T3Motor_CmdCombination(VOutput,LOutput,1);
            }
            else
            {
                time=0;
            }
        }
        else if((angle>60+de[2])&&(angle<=80))//90��
        {
            VOutput = Taks3Pid7.PIDControl(Vtarget_angle,roll,&Taks3Pid7);
            LOutput = Taks3Pid8.PIDControl(Ltarget_angle,-pitch,&Taks3Pid8);

            time +=0.0103423;
            if(time<T)
            {
                T3Motor_CmdCombination(VOutput,LOutput,1);
            }
            else
            {
                time=0;
            }
        }
        
    }
/*****************************************����90��************************************************************* */
    else if(a==2)
    {
        if(angle<=30+de[3])     //150   
        {
            VOutput = Taks3Pid1.PIDControl(Vtarget_angle,-roll,&Taks3Pid1);
            LOutput = Taks3Pid2.PIDControl(Ltarget_angle,-pitch,&Taks3Pid2);

            time +=0.0104123;
            if(time<T)
            {
                T3Motor_CmdCombination(VOutput,LOutput,0);
            }
            else
            {
                time=0;
            }
        }
        else if((angle>30+de[3])&&(angle<=45+de[4]))     //135 
        {
            VOutput = Taks3Pid3.PIDControl(Vtarget_angle,-roll,&Taks3Pid3);
            LOutput = Taks3Pid4.PIDControl(Ltarget_angle,-pitch,&Taks3Pid4);

            time +=0.0103423;
            if(time<T)
            {
                T3Motor_CmdCombination(VOutput,LOutput,0);
            }
            else
            {
                time=0;
            }
        }
        else if((angle>45+de[4])&&(angle<=60+de[5]))     //120 
        {
            VOutput = Taks3Pid5.PIDControl(Vtarget_angle,-roll,&Taks3Pid5);
            LOutput = Taks3Pid6.PIDControl(Ltarget_angle,-pitch,&Taks3Pid6);

            time +=0.0103323;
            if(time<T)
            {
                T3Motor_CmdCombination(VOutput,LOutput,0);
            }
            else
            {
                time=0;
            }
        }
        
        
    }
/******************************************����90*********************************************************** */

    
}



float timepp = 0.0102220;
/*
    ��Բ

    290,0,0  �� 15~20
    290,0,0
*/
void Task5_CircleMove(float R)
{

  
    static float VOutput,LOutput;
    static float A;
    static float time=0;
    static float Vtarget_angle,Ltarget_angle;
    
    static struct PID Taks5Pid={290,0,0,PidControl_LineMove};
    static struct PID Taks5Pid2={290,0,0,PidControl_LineMove};
    
    if(R>=0.24)
    {
        R=0.24;
    }
    
    R += 0.05;


    GetPolar(roll,pitch);
    A = asinf(R/0.875)*180.0f/PI;

    float wt = 2*PI/T*time;

    Vtarget_angle = A*sinf(wt);
    Ltarget_angle = A*cosf(wt);

    float feedback_angle = A*sinf((polar/180*PI)+PI);

    int sit;

    //ʵ��������Ŀ�����ҵıȽ�
    //printf("%f,%f\r\n",Ltarget_angle,feedback_angle);

    
    time+=timepp;
    if(time>T)
    {
        time=0;
        return;
    }

    if((wt<PI/2)&&(wt>=0))   //0-PI/2
    {
        sit = 1;
        VOutput = Taks5Pid.PIDControl(Vtarget_angle,absroll,&Taks5Pid);
        LOutput = Taks5Pid2.PIDControl(Ltarget_angle,abspitch,&Taks5Pid2);
    }
    else if((wt<PI)&&(wt>=PI/2)) //PI/2-PI
    {
        sit = 2;
        VOutput = Taks5Pid.PIDControl(Vtarget_angle,absroll,&Taks5Pid);
        LOutput = Taks5Pid2.PIDControl(Ltarget_angle,-abspitch,&Taks5Pid2);
    }
    else if((wt<3*PI/2)&&(wt>=PI))   //PI-3PI/2
    {
        sit = 3;
        VOutput = Taks5Pid.PIDControl(Vtarget_angle,-absroll,&Taks5Pid);
        LOutput = Taks5Pid2.PIDControl(Ltarget_angle,-abspitch,&Taks5Pid2);
    }
    else if((wt<2*PI)&&(wt>=3*PI/2)) //3PI/2-2PI
    {
        sit = 4;
        VOutput = Taks5Pid.PIDControl(Vtarget_angle,-absroll,&Taks5Pid);
        LOutput = Taks5Pid2.PIDControl(Ltarget_angle,abspitch,&Taks5Pid2);
    }
    
    
    //��ֱ�����ˮƽ����ıȽ�
    //printf("%f,%f\r\n",VOutput,LOutput);
    
    T5Motor_CmdCombination(sit,VOutput,LOutput);

}