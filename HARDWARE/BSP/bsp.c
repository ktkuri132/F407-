/*


    这个文件 实现了 部分基本的外设的初始化函数，以及外设的初始化，带参数的函数都有用注释介绍



*/



#include <stm32f4xx.h>
#include <usart.h>
#include <bsp.h>
#include <control.h>
#include <usart.h>
#include <mpu6050.h>
#include <inv_mpu.h>
#include <stm32f4xx_exti.h>
#include <delay.h>


#define target 180


/// @brief 外设初始化
/// @return 0:成功;  
/// @return 1:MPU6050初始化失败
/// @return 2:MPU6050 DMP初始化失败
int BSP_Init()
{
    usart1_init(115200);
    usart2_init(9600);
    printf("peripheral init start......\n");
    delay_init(84);
    printf("->Delay Init done\n");
    BSP_NVIC_IT_Config();
    printf("->NVIC Init  done\n");
	LED_Init();
    printf("->LED Init  done\n");
    Motor_PWM_TIM8_Init();
    printf("->Motor Init  done\n");
    OLED_Init();
    printf("->OLED Init done\n");
    GPIO_Config(GPIOF, GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7, GPIO_Mode_OUT, GPIO_PuPd_NOPULL);
    printf("->GPIOF Init done\n");
    
    

    delay_ms(20);

    if(MPU_Init())
    {
        printf("****MPU6050 init failed return:%d\n",MPU_Init());
        return 1;
    }
    else
    {
        printf("->MPU6050 init success\n");
    }

    if(mpu_dmp_init())
    {
        printf("**************MPU6050 DMP init failed******>>Reset and Retry or Cheak\n");
        return 2;
    }
    else
    {
        printf("->MPU6050 DMP init success\n");
    }

    EXIT15_Init();
    printf("->ETIT15 Init done\n");
    //TIM2_Init(5);
    //printf("->TIM2 Init done\n");


    return 0;
}


void LED_Init()
{
    RCC->AHB1ENR |=1<<5;
    GPIOF->MODER |=1<<18;
    GPIOF->PUPDR |=0<<18;
    GPIOF->BSRRL |=1<<9;
}

/// @brief GPIO配置函数（默认速度100，默认推挽输出，调用前请自行使能时钟）
/// @param GPIOx 不必多说
/// @param GPIO_Pin_x 不必多说
/// @param GPIO_Mode_x GPIO模式
/// @param GPIO_PuPd_x 上拉、下拉、浮空
void GPIO_Config(GPIO_TypeDef *GPIOx,uint32_t GPIO_Pin_x,uint32_t GPIO_Mode_x,uint32_t GPIO_PuPd_x)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_x;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_x;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_x;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_Init(GPIOx, &GPIO_InitStructure);
}

//PG15的外部中断初始化
void EXIT15_Init()
{
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE);

    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOG, EXTI_PinSource15);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOG, &GPIO_InitStructure);

    EXTI_InitTypeDef EXTI_InitStructure;
    
    EXTI_InitStructure.EXTI_Line = EXTI_Line15;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

}


//定时器2初始化
void TIM2_Init(uint32_t ms)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    TIM_TimeBaseStructure.TIM_Period = 1000*ms - 1;
    TIM_TimeBaseStructure.TIM_Prescaler = 84 - 1;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

    TIM_Cmd(TIM2, ENABLE);
}



/* ADC配置  */
void ADC_Config()
{
    ADC_InitTypeDef ADC_InitStructure;
    ADC_CommonInitTypeDef ADC_CommonInitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    

    // Enable the ADC clock
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

    // Configure the ADC pin
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // ADC configuration
    ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfConversion = 1;
    ADC_Init(ADC1, &ADC_InitStructure);

    ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_480Cycles);

    // ADC common configuration

    ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;
    ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
    ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
    ADC_CommonInit(&ADC_CommonInitStructure);

    // Enable the ADC
    ADC_Cmd(ADC1, ENABLE);

    // Start ADC conversion
    ADC_SoftwareStartConv(ADC1);

    // Enable ADC end of conversion interrupt
    ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);

    // Configure and enable ADC interrupt in NVIC
    NVIC_InitStructure.NVIC_IRQChannel = ADC_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; // 设置抢占优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1; // 设置子优先级
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

}



// ADC interrupt handler
void ADC_IRQHandler(void)
{
    // Check if the end of conversion flag is set
    if (ADC_GetITStatus(ADC1, ADC_IT_EOC) != RESET)
    {
        // Clear the end of conversion flag
        ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);

        // Handle the ADC conversion result
        static uint16_t adcValue ;
        static uint16_t pidOutput;

        adcValue = ADC_GetConversionValue(ADC1);
        //pidOutput = pidControl(target, adcValue);
        //pidOutput>8400?pidOutput=8400:(pidOutput<0?pidOutput=0:pidOutput);
        TIM1->CCR1 = 3600;
        printf("ADC value: %d\n", ADCvalue_convert(adcValue));

        // 处理adcValue
    }
}

uint16_t ADCvalue_convert(uint16_t adcValue)
{
    uint16_t output;
    output = (uint16_t)(((float)adcValue / 3111.0) * 360.0);
    return output;
    
}


#ifdef __GNUC__
__INLINE void function_goto(int (*pfunction)())
{
    __ASM __IO 
    (
        "mov r0,%0 \n\t"
        "bx r0     \n\t"
        ::"r"(pfunction)
    );
}
#endif