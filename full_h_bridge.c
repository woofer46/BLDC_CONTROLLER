#include "full_h_bridge.h"

TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;      // Определение структуры инициализауии таймеров
TIM_OCInitTypeDef  TIM_OCInitStructure;              // Определение структуры для настройки PWM
uint16_t PrescalerValue = 0;                         // Определение предделителей для таймеров, ШИМ


void init_h_bridge(void)
{
	// Включение тактирования Таймеров
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);

	// Включение тактирования GPIO
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOE, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure; // Стукрутра инициализации GPIO
	//-----------------------------------------------------------------------
	// Двигатель 2, верхние ключи
	//-----------------------------------------------------------------------
    // Конфигурация портов TIM1 CH1(PA8), CH2(PE11), CH3(PA10)
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_8|GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	//-----------------------------------------------------------------------
	// Подключение портов к альтернативной функции TIM1
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_TIM1);
	//-----------------------------------------------------------------------
	// Двигатель 1, верхние ключи
	//-----------------------------------------------------------------------
	// Конфигурация портов TIM8 CH1(PC6), CH2(PC7), CH3(PC8)
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	//-----------------------------------------------------------------------
	// Подключение портов к альтернативной функции TIM8
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM8);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM8);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_TIM8);
	//-----------------------------------------------------------------------
    // Настройка таймеров
    // *To get TIM3 counter clock at 28 MHz, the prescaler is computed as follows:
    // *Prescaler = (TIM3CLK / TIM3 counter clock) - 1
    // *Prescaler = ((SystemCoreClock /2) /28 MHz) - 1
    //-----------------------------------------------------------------------
    PrescalerValue = (uint16_t) ((SystemCoreClock /2) / 28000000) - 1;
    //-----------------------------------------------------------------------
    // Настройка TIM1 Двигатель 2 RIGHT
    TIM_TimeBaseStructure.TIM_Period=2000;
    TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
    // PWM mode
    TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse=100;
    TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_High;
    //TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
    // CH1
    TIM_OC1Init(TIM1, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
    // CH2
    TIM_OC2Init(TIM1, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
    // CH3
    TIM_OC3Init(TIM1, &TIM_OCInitStructure);
    TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
    // Предзагрузка
    TIM_ARRPreloadConfig(TIM1, ENABLE);
    //-----------------------------------------------------------------------
    // Настройка TIM8 Двигатель 1 LEFT
    TIM_TimeBaseStructure.TIM_Period=2000;
    TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);
    // PWM mode
    TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse=100;
    TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_High;
    //TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
    // CH1
    TIM_OC1Init(TIM8, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);
    // CH2
    TIM_OC2Init(TIM8, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable);
    // CH3
    TIM_OC3Init(TIM8, &TIM_OCInitStructure);
    TIM_OC3PreloadConfig(TIM8, TIM_OCPreload_Enable);
    // Предзагрузка
    TIM_ARRPreloadConfig(TIM8, ENABLE);
    //-----------------------------------------------------------------------
    // Включение TIM1 и TIM8
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
    TIM_Cmd(TIM1, ENABLE);
	TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Disable);
	TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Disable);
	TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Disable);
	//-----------------------------------------------------------------------
    TIM_CtrlPWMOutputs(TIM8, ENABLE);
    TIM_Cmd(TIM8, ENABLE);
	TIM_CCxCmd(TIM8, TIM_Channel_1, TIM_CCx_Disable);
	TIM_CCxCmd(TIM8, TIM_Channel_2, TIM_CCx_Disable);
	TIM_CCxCmd(TIM8, TIM_Channel_3, TIM_CCx_Disable);
    //-----------------------------------------------------------------------
    //-----------------------------------------------------------------------
    // Двигатель 2, нижние ключи PB13, PB14, PB15
    //-----------------------------------------------------------------------
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14| GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	//-----------------------------------------------------------------------
    // Двигатель 1, нижние ключи PA7, PB0, PB1
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	//-------------------------------------------------------------------------------
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}
