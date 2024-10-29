/*


这个文件 装载了  风力摆的核心部分---控制部分



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
#include <stdint.h>

extern float pitch,roll,yaw,dis,def,polar,Opolar,absroll,abspitch;





/*

15cm 500,0,-65.5,00983
25cm 395,0,0.0001,0.0104223  <---这个具有普遍性

750,0,-350,0.0104223

(17+0.73333*Target_angle)
(140.625-Target_angle)/3.75+Target_angle

*/


//         30 45 60 150  135   120  90
int de[7]={-1,+0,-5,-7,+12.5,+10.5,-10 };


float Target_dis=0;
float Target_angle = 0;
uint8_t mode=0;


/// @brief MPU6050读值触发的外部中断函数      
void EXTI15_10_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line15) != RESET)
    {
        EXTI_ClearITPendingBit(EXTI_Line15);
        
        mpu_dmp_get_data(&pitch, &roll, &yaw);      //读取MPU6050数据

        
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
                Task4_StopFast();    //调用控制函数--->第4项
                break;


            default:
                    StopAllMotor();
            break;
                
        }

    }
}


float target_R=0.15;

/*
    定时器2中断函数,定时5ms计算一次PID
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
    第四项  快速制动
*/
void Task4_StopFast() 
{
    GetPolar(roll,pitch);

   
    static float Output;
    static struct PID Taks4Pid={38810,0,-300,PidControl_Stop};
    
    Output = Taks4Pid.PIDControl(TargetDis,dis, &Taks4Pid);
    //printf("Output:%f\n",Output);
    PWM_Allocation(Output);
  
}


float target_angle;

/*
    第一,二项  单摆运动+幅度可调
*/
void Task1_LineMove(float R,uint8_t forward)
{
    static float VOutput,LOutput;
    static float A;
    static float time=0;
	static uint8_t flag_1=0,flag_2=0,flag_3=0;
    /*0度 - 15cm以内*/
    static struct PID Taks1Pid1={750,0,-295,PidControl_LineMove};
    /*0度 - 20~25cm*/
    static struct PID Taks1Pid2={950,0,-320,PidControl_LineMove};
    /*90度 - 15cm*/
    static struct PID Taks1Pid3={720,0,-420,PidControl_LineMove};
    /*90度 - 20~25cm*/
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
    time+=0.0104220;//到位，到位
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
    第三项  角度可调的单摆运动
*/
void Task3_AngleMove(float angle,float R)
{

    static float VOutput,LOutput;
    static float target_roll,target_pitch;
    static float time=0;
    static float Vtarget_angle,Ltarget_angle;
    
    static struct PID Taks3Pid1={360,0,-90,PidControl_LineMove};
    static struct  PID Taks3Pid2={360,0,-90,PidControl_LineMove};
    
    static struct PID Taks3Pid3={345,0,-110,PidControl_LineMove};
    static struct  PID Taks3Pid4={345,0,-110,PidControl_LineMove};

    static struct PID Taks3Pid5={380,0,-120,PidControl_LineMove};
    static struct  PID Taks3Pid6={380,0,-120,PidControl_LineMove};

    static struct PID Taks3Pid7={340,0,-90,PidControl_LineMove};
    static struct  PID Taks3Pid8={340,0,-90,PidControl_LineMove};

    int a;
    if(angle>90)
    {   
        //angle = (140.625-angle)/3.75+angle,0.15;
        angle = 180 - angle;
        //这里搞多点是因为浮点型读值不确定
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
        else if((angle>31)&&(angle<=45))
        {
            angle += de[1];
        }
        else if((angle>45)&&(angle<=56))
        {
            angle += de[2];
        }

        a=1;
    }
    else if(angle==90)
    {
        angle +=de[6];
        a=1;
    }
    
   // angle = angle+5;
    angle = angle*PI/180.0f;

    target_roll = atanf(R*cosf(angle)/0.86)/PI*180;
    target_pitch = atanf(R*sinf(angle)/0.86)/PI*180;

    float wt = 2*PI*(time/T);

    Vtarget_angle =sinf(wt)*target_roll;
    Ltarget_angle =sinf(wt)*target_pitch;
 
    angle = (angle * 180) / PI;
	//printf("%f,%f\r\n",VOutput,LOutput);
/**************************************小于90度**************************************************************** */
    if(a==1)
    {
        //printf("%f,%f\r\n",angle,PI/6);
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
            Target_dis = 0.16;
            VOutput = Taks3Pid5.PIDControl(Vtarget_angle,roll,&Taks3Pid5);
            LOutput = Taks3Pid6.PIDControl(Ltarget_angle,-pitch,&Taks3Pid6);

            time +=0.0114323;
            if(time<T)
            {
                T3Motor_CmdCombination(VOutput,LOutput,1);
            }
            else
            {
                time=0;
            }
        }
        else if((angle>60+de[2])&&(angle<=80))//90度
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
/*****************************************大于90度************************************************************* */
    else if(a==2)
    {
        if(angle<=30+de[3])     //150   
        {
            VOutput = Taks3Pid1.PIDControl(Vtarget_angle,-roll,&Taks3Pid1);
            LOutput = Taks3Pid2.PIDControl(Ltarget_angle,-pitch,&Taks3Pid2);

            time +=0.0103123;
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

            time +=0.0102523;
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
/******************************************等于90*********************************************************** */

    
}



float timepp = 0.0102220;
/*
    画圆

    290,0,0  跑 15~20
    290,0,0
*/
void Task5_CircleMove(float R)
{

  
    static float VOutput,LOutput;
    static float A;
    static float time=0;
    static float Vtarget_angle,Ltarget_angle;
    
    static struct PID Taks5Pid={295,0,0,PidControl_LineMove};
    static struct PID Taks5Pid2={295,0,0,PidControl_LineMove};
    
    if((R>=0.24)&&(R<=0.26))
    {
        R=0.24;
    }
    else if((R>=0.29)&&(R<=0.31))
    {
        R=0.27;
    }
    else if((R>=0.34)&&(R<=0.36))
    {
        R=0.32;
    }
    

    R += 0.05;


    GetPolar(roll,pitch);
    A = asinf(R/0.875)*180.0f/PI;

    float wt = 2*PI/T*time;

    Vtarget_angle = A*sinf(wt);
    Ltarget_angle = A*cosf(wt);

    float feedback_angle = A*sinf((polar/180*PI)+PI);

    int sit;

    //实际正弦与目标正弦的比较
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
    
    
    //垂直输出和水平输出的比较
    //printf("%f,%f\r\n",VOutput,LOutput);
    
    T5Motor_CmdCombination(sit,VOutput,LOutput);

}