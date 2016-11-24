﻿// Svyatoslav Mishin 2016
// Brushless EMF controllers

#include <stm32f4xx_conf.h>

TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;      // Определение структуры инициализауии таймеров
TIM_OCInitTypeDef  TIM_OCInitStructure;              // Определение структуры для настройки PWM
uint16_t PrescalerValue = 0;                         // Определение предделителей для таймеров, ШИМ
uint16_t del_count=0;                                // Счетчик для фунции delay_ms
uint16_t delay_timeBLDC1=0;                          // Счетчик задержки коммутирования состояния для бездатчикового управления (Двиг 1)
uint16_t delay_timeBLDC2=0;                          // --//--//-- (Двиг 2)
uint8_t CountStates_ticks = 0;                       // Счетчик времение счета прошедших состояний (для регулятора)

uint8_t Recive_buf[256];
uint8_t Recive_W=0;
uint8_t tmp=0;
uint8_t flag_start=0;                                // Флаг запуска программы управления по датчикам Холла
uint8_t control_emf_enable=0;                        // Флаг запуска управления по Обратной ЭДС

uint8_t count_step_statesBLDC1=0;                    // Количетсво комутаций в холостом режиме (Обр ЭДС)
uint8_t enable_stateBLDC1 =0;                        // Переменная хранящая текущее состояние 0, 1, 2, 3, 4, 5
uint8_t back_emf_enableBLDC1=0;                      // Флаг включающий управление по ЭДС, после того как прошел холостой режим
uint8_t current_stateBLDC1 =0;                       // Флаг того что состояние не изменилось с последнего раза
uint8_t previous_stateBLDC1 =0;                      // Предыдущее состояние
uint8_t CountStates_statesBLDC1 = 0;                 // Счетчик количества переключений по состояниям (для регулятора)
uint8_t emf_delayBLDC1=19;//3                        // Время удержания состояния (Обр Эдс)
uint8_t HallSwitchTime = 0;							// Время переключения между состояниями датчика Холла

uint8_t count_step_statesBLDC2=0;
uint8_t enable_stateBLDC2 =0;
uint8_t back_emf_enableBLDC2=0;
uint8_t current_stateBLDC2 =0;
uint8_t previous_stateBLDC2 =0;
uint8_t CountStates_statesBLDC2 = 0;
uint8_t emf_delayBLDC2=19;

uint8_t hallph1;
uint8_t hallph2;
uint8_t hallph3;


uint8_t SetHallControl = 1;                       // Режим управления по датчикам холла Флаг
uint8_t CurrentHallState =100;                    // Текущее состояние по датчикам холла

void SysTick_Handler(void);
void delay_ms(uint16_t del_temp);
void USART2_IRQHandler(void);
void str_to_usart(char* str);
void control_emf(void);
void disable_tim_chanels(void);

#define Speed200_1 TIM_SetCompare1(TIM8, 400)
#define Speed200_2 TIM_SetCompare2(TIM8, 400)
#define Speed200_3 TIM_SetCompare3(TIM8, 400)

#define Speed400_1 TIM_SetCompare1(TIM8, 800)
#define Speed400_2 TIM_SetCompare2(TIM8, 800)
#define Speed400_3 TIM_SetCompare3(TIM8, 800)

#define Speed900_1 TIM_SetCompare1(TIM8, 2000)
#define Speed900_2 TIM_SetCompare2(TIM8, 2000)
#define Speed900_3 TIM_SetCompare3(TIM8, 2000)

#define Speed1000_1 TIM_SetCompare1(TIM4, 2000)
#define Speed1000_2 TIM_SetCompare2(TIM4, 2000)
#define Speed1000_3 TIM_SetCompare3(TIM4, 2000)

//Двигатель 1
#define ReadPhase_U1 GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_0)
#define ReadPhase_V1 GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_1)
#define ReadPhase_W1 GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_2)
//Двигатель 2
#define ReadPhase_U2 GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_3)
#define ReadPhase_V2 GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_4)
#define ReadPhase_W2 GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_5)
//Двигатель 1
#define Disable_Ho_U1 TIM_CCxCmd(TIM8, TIM_Channel_1, TIM_CCx_Disable);
#define Disable_Ho_V1 TIM_CCxCmd(TIM8, TIM_Channel_2, TIM_CCx_Disable);
#define Disable_Ho_W1 TIM_CCxCmd(TIM8, TIM_Channel_3, TIM_CCx_Disable);

#define Enable_Ho_U1 TIM_CCxCmd(TIM8, TIM_Channel_1, TIM_CCx_Enable);
#define Enable_Ho_V1 TIM_CCxCmd(TIM8, TIM_Channel_2, TIM_CCx_Enable);
#define Enable_Ho_W1 TIM_CCxCmd(TIM8, TIM_Channel_3, TIM_CCx_Enable);

//Двигатель 2
#define Disable_Ho_U2 TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Disable);
#define Disable_Ho_V2 TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Disable);
#define Disable_Ho_W2 TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Disable);

#define Enable_Ho_U2 TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);
#define Enable_Ho_V2 TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);
#define Enable_Ho_W2 TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable);

//Двигатель 1
#define Disable_Lo_U1 GPIO_ResetBits(GPIOA, GPIO_Pin_7);
#define Disable_Lo_V1 GPIO_ResetBits(GPIOB, GPIO_Pin_0);
#define Disable_Lo_W1 GPIO_ResetBits(GPIOB, GPIO_Pin_1);

#define Enable_Lo_U1 GPIO_SetBits(GPIOA, GPIO_Pin_7);
#define Enable_Lo_V1 GPIO_SetBits(GPIOB, GPIO_Pin_0);
#define Enable_Lo_W1 GPIO_SetBits(GPIOB, GPIO_Pin_1);

//Двигатель 2
#define Disable_Lo_U2 GPIO_ResetBits(GPIOB, GPIO_Pin_13);
#define Disable_Lo_V2 GPIO_ResetBits(GPIOB, GPIO_Pin_14);
#define Disable_Lo_W2 GPIO_ResetBits(GPIOB, GPIO_Pin_15);

#define Enable_Lo_U2 GPIO_SetBits(GPIOB, GPIO_Pin_13);
#define Enable_Lo_V2 GPIO_SetBits(GPIOB, GPIO_Pin_14);
#define Enable_Lo_W2 GPIO_SetBits(GPIOB, GPIO_Pin_15);

// Чтение состояний датчиков холла для двигателя 1
#define ReadStateHall1Motor1 GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_2)
#define ReadStateHall2Motor1 GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_3)
#define ReadStateHall3Motor1 GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_4)

// Чтение состояний датчиков холла для двигателя 2
#define ReadStateHall1Motor2 GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_7)
#define ReadStateHall2Motor2 GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_10)
#define ReadStateHall3Motor2 GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_11)

int main(void)
{
	SystemInit();
	SysTick_Config(SystemCoreClock/1000);

	GPIO_InitTypeDef GPIO_InitStructure; // Стукрутра инициализации GPIO
	//-----------------------------------------------------------------------
	// Включение тактирования Таймеров
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
	//-----------------------------------------------------------------------
	// Включение тактирования GPIO
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOE, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
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
    // Настройка TIM1 Двигатель 2
    TIM_TimeBaseStructure.TIM_Period=2000;
    TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
    // PWM mode
    TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse=380;
    TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_High;
    //TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
    // CH1
    TIM_OC1Init(TIM1, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
    // CH2
    TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse=380;
    TIM_OC2Init(TIM1, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
    // CH3
    TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;//
    TIM_OCInitStructure.TIM_Pulse=380;//
    TIM_OC3Init(TIM1, &TIM_OCInitStructure);
    TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
    // Предзагрузка
    TIM_ARRPreloadConfig(TIM1, ENABLE);
    //-----------------------------------------------------------------------
    // Настройка TIM8 Двигатель 1
    TIM_TimeBaseStructure.TIM_Period=2000;
    TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);
    // PWM mode
    TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse=380;
    TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_High;
    //TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
    // CH1
    TIM_OC1Init(TIM8, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);
    // CH2
    TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse=380;
    TIM_OC2Init(TIM8, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable);
    // CH3
    TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;//
    TIM_OCInitStructure.TIM_Pulse=380;//
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
	//-----------------------------------------------------------------------
	// Двигатель 1, вход с компараторов PС0, PС1, PС2
	GPIO_InitTypeDef GPIO_InitComparator;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	GPIO_InitComparator.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2;
	GPIO_InitComparator.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitComparator.GPIO_OType = GPIO_OType_PP;
	GPIO_InitComparator.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitComparator.GPIO_PuPd = GPIO_PuPd_UP;//Подтянуть
	GPIO_Init(GPIOC, &GPIO_InitComparator);
	//-----------------------------------------------------------------------
	// Двигатель 2, вход с компараторов PС3, PС4, PС5
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	GPIO_InitComparator.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitComparator.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitComparator.GPIO_OType = GPIO_OType_PP;
	GPIO_InitComparator.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitComparator.GPIO_PuPd = GPIO_PuPd_UP;//Подтянуть
	GPIO_Init(GPIOC, &GPIO_InitComparator);
	//-----------------------------------------------------------------------
	// Вход с датчиков Холла, двигатель 1
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	//-----------------------------------------------------------------------
	// Настройка USART
	GPIO_InitTypeDef  GPIO_InitUSART;
	USART_InitTypeDef USART_InitUser;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	GPIO_InitUSART.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3;
	GPIO_InitUSART.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitUSART.GPIO_OType = GPIO_OType_PP;
	GPIO_InitUSART.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitUSART.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitUSART);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2); //Tx Трансивер-Передатчик
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2); //Rx Приемник

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	USART_InitUser.USART_BaudRate=115200;
	USART_InitUser.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
	USART_InitUser.USART_Mode=USART_Mode_Rx|USART_Mode_Tx;
	USART_InitUser.USART_Parity=USART_Parity_No;
	USART_InitUser.USART_StopBits=USART_StopBits_1 ;
	USART_InitUser.USART_WordLength=USART_WordLength_8b;
	USART_Init(USART2, &USART_InitUser);
	NVIC_EnableIRQ(USART2_IRQn);
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	USART_Cmd(USART2, ENABLE);
	//-------------------------------------------------------------------------------
	str_to_usart("Program is ready\n\r");
    while(1)
    {
		if(control_emf_enable)
		{
			control_emf();
			control_emf_2();
		}
		if(SetHallControl==1)
		{
			control_hall_motor1();
		}
    }
}
//Обработчик прерывания юарт
void USART2_IRQHandler(void)
{
	//-----------------------------------------------------------------------
	//Обработчик прерывания на прием с компьютера
	if(USART_GetITStatus(USART2, USART_IT_RXNE)==SET)
	{
		//-----------------------------------------------------------------------
		USART_ClearITPendingBit(USART2,USART_IT_RXNE);
		Recive_buf[Recive_W] = USART_ReceiveData(USART2);
		tmp=Recive_buf[Recive_W];
		Recive_W++;
		//-----------------------------------------------------------------------
		// Поиск конца строки
		if(tmp=='\r')
		{
			 if(strcmp(Recive_buf,"start\r")==0)
				{
					str_to_usart("Program start\r");
					memset(Recive_buf, 0, sizeof(Recive_buf));
					Recive_W=0;
					SetHallControl=1;
				}

			else if(strcmp(Recive_buf,"stop\r")==0)
				{
					flag_start=0;
					disable_tim_chanels();
					str_to_usart("Program stop\r");
					memset(Recive_buf, 0, sizeof(Recive_buf));
					Recive_W=0;
					SetHallControl=0;
				}

			else if(strcmp(Recive_buf,"Speed400\r")==0)
				{
					str_to_usart("Speed 400\n\r");
					Speed400_1;
					Speed400_2;
					Speed400_3;
					emf_delayBLDC1=8;
					emf_delayBLDC2=8;
					memset(Recive_buf, 0, sizeof(Recive_buf));
					Recive_W=0;
				}

			else if(strcmp(Recive_buf,"Speed200\r")==0)
				{
				 	if(flag_start)
				 	{
						str_to_usart("Speed 200\n\r");
						Speed200_1;
						Speed200_2;
						Speed200_3;
						emf_delayBLDC1=15;
					}
					else
					{
						str_to_usart("First start motor\n\r");
					}

				memset(Recive_buf, 0, sizeof(Recive_buf));
				Recive_W=0;
				}

			else if(strcmp(Recive_buf,"Speed900\r")==0)
				{
				 	//if(flag_start)
				 	{
						str_to_usart("Speed 900\n\r");
						Speed900_1;
						Speed900_2;
						Speed900_3;
						emf_delayBLDC1=3;
				 	}
					//else
					{
						//str_to_usart("First start motor\n\r");
					}

				memset(Recive_buf, 0, sizeof(Recive_buf));
				Recive_W=0;
				}

			else if(strcmp(Recive_buf,"Speed1000\r")==0)
				{
						str_to_usart("Speed 1000\n\r");
						Speed1000_1;
						Speed1000_2;
						Speed1000_3;

				//	else
					{
					//	str_to_usart("First start motor\n\r");
					}

				memset(Recive_buf, 0, sizeof(Recive_buf));
				Recive_W=0;
				}

			/*else if(strcmp(Recive_buf,"EMF\r")==0)
				{
				 	//if(flag_start)
				 	//{
						str_to_usart("EMF\n\r");
				 		//Speed1000_1;
				 		//Speed1000_2;
				 		//Speed1000_3;
						control_emf_enable=1;

					//}
					//else
					{
						//str_to_usart("First start motor\n\r");
					}

				memset(Recive_buf, 0, sizeof(Recive_buf));
				Recive_W=0;
				}*/
			/*else if(strcmp(Recive_buf,"Speedxxx"))
			{

			}

			else if (strcmp(Recive_buf))
			{

			}*/

			else
				{
				str_to_usart("You send: ");
				str_to_usart(Recive_buf);
				str_to_usart("But command undeclared.\r");
				memset(Recive_buf, 0, sizeof(Recive_buf));
				Recive_W=0;
				}


		}
	//-----------------------------------------------------------------------
	}
}
//-----------------------------------------------------------------------
// Функция отправки строчки в юарт
void str_to_usart(char* str)
{
	uint16_t TxCount=0;
	while(str[TxCount] != 0)
	{
		while(USART_GetFlagStatus(USART2, USART_FLAG_TC)== RESET){}
		USART_SendData(USART2, str[TxCount]);
		TxCount++;
	}
}
void delay_ms(uint16_t del_temp)
{
	del_count=del_temp;
	while(del_count)
	{}
}
void SysTick_Handler(void) // Таймер 1мс
{
	if(del_count>0)
		del_count--;
	if(delay_timeBLDC1>0)
		delay_timeBLDC1--; //Отсчет задержки
	if(delay_timeBLDC2>0)
		delay_timeBLDC2--;
	HallSwitchTime++;
	if(CountStates_ticks < 250) // Считаем 250 мс
	{
		CountStates_ticks++;
	}
	else
	{
		CountStates_ticks = 0; // Если досчитали, то отправляем количество пройденых меток
		USART_SendData(USART2, CountStates_statesBLDC2);// Для ПИД регулятора
		//CountStates_statesBLDC1 = 0;
		CountStates_statesBLDC2=0;
	}
}

//-------------------------------------------------------------------------------
// Управление по датчикам холла ДВИГАТЕЛЬ 1
void control_hall_motor1(void)
{
	if(SetHallControl==1)
	{
		hallph1=ReadStateHall1Motor1;
		hallph2=ReadStateHall2Motor1;
		hallph3=ReadStateHall3Motor1;

		if(hallph1 == 0 && hallph2 == 0 && hallph3 == 1)
		{
			if(CurrentHallState!=1)
			{
				Disable_Ho_U1;
				Disable_Ho_W1;
				Enable_Ho_V1; // V +
				Disable_Lo_U1;
				Disable_Lo_V1;
				Enable_Lo_W1; // W -
				CurrentHallState=1;
				int CurrentSpeed = 6000/(HallSwitchTime*14);
				char buf[6] = {0,0,0,'\n','\r',0};
				buf[0] = CurrentSpeed/100 + 48;
				buf[1] = (CurrentSpeed%100)/10 + 48;
				buf[2] = CurrentSpeed%10 + 48;
				str_to_usart(buf);
				HallSwitchTime = 0;

			}
		}
		else if(hallph1 == 0 && hallph2 == 1 && hallph3 == 1)
		{
			if(CurrentHallState!=2)
			{
				Disable_Ho_V1;
				Disable_Ho_W1;
				Enable_Ho_U1;   // U +
				Disable_Lo_U1;
				Disable_Lo_V1;
				Enable_Lo_W1;   // W -
				CurrentHallState=2;
				HallSwitchTime = 0;
			}
		}
		else if(hallph1 == 0 && hallph2 == 1 && hallph3 == 0)
		{
			if(CurrentHallState!=3)
			{
				Disable_Ho_V1;
				Disable_Ho_W1;
				Enable_Ho_U1;    // U +
				Disable_Lo_W1;
				Disable_Lo_U1;
				Enable_Lo_V1;    // V -
				CurrentHallState=3;
				char buf[6] = {0,0,0,'\n','\r',0};
				buf[0] = HallSwitchTime/100 + 48;
				buf[1] = (HallSwitchTime%100)/10 + 48;
				buf[2] = HallSwitchTime%10 + 48;
				//str_to_usart(buf);
				HallSwitchTime = 0;
			}
		}
		else if(hallph1 == 1 && hallph2 == 1 && hallph3 == 0)
		{
			if(CurrentHallState!=4)
			{
				Disable_Ho_U1;
				Disable_Ho_V1;
				Enable_Ho_W1;    // W +
				Disable_Lo_W1;
				Disable_Lo_U1;
				Enable_Lo_V1;    // V -
				CurrentHallState=4;
				HallSwitchTime = 0;
			}
		}
		else if(hallph1 == 1 && hallph2 == 0 && hallph3 == 0)
		{
			if(CurrentHallState!=5)
			{
				Disable_Ho_U1;
				Disable_Ho_V1;
				Enable_Ho_W1;    // W +
				Disable_Lo_W1;
				Disable_Lo_V1;
				Enable_Lo_U1;    // U -
				CurrentHallState=5;
				HallSwitchTime = 0;
			}
		}
		else if(hallph1 == 1 && hallph2 == 0 && hallph3 == 1)
		{
			if(CurrentHallState!=6)
			{
				Disable_Ho_U1;
				Disable_Ho_W1;
				Enable_Ho_V1;    // V +
				Disable_Lo_W1;
				Disable_Lo_V1;
				Enable_Lo_U1;    // U -
				CurrentHallState=6;
				HallSwitchTime = 0;
			}
		}
	}
}
// Управление по датчикам холла ДВИГАТЕЛЬ 2
void control_hall_motor2(void)
{
	if(SetHallControl==1)
	{
		hallph1=ReadStateHall1Motor2;
		hallph2=ReadStateHall2Motor2;
		hallph3=ReadStateHall3Motor2;

		if(hallph1 == 0 && hallph2 == 0 && hallph3 == 1)
		{
			if(CurrentHallState!=1)
			{
				Disable_Ho_U1;
				Disable_Ho_W1;
				Enable_Ho_V1; // V +
				Disable_Lo_U1;
				Disable_Lo_V1;
				Enable_Lo_W1; // W -
				CurrentHallState=1;
			}
		}
		else if(hallph1 == 0 && hallph2 == 1 && hallph3 == 1)
		{
			if(CurrentHallState!=2)
			{
				Disable_Ho_V1;
				Disable_Ho_W1;
				Enable_Ho_U1;   // U +
				Disable_Lo_U1;
				Disable_Lo_V1;
				Enable_Lo_W1;   // W -
				CurrentHallState=2;
			}
		}
		else if(hallph1 == 0 && hallph2 == 1 && hallph3 == 0)
		{
			if(CurrentHallState!=3)
			{
				Disable_Ho_V1;
				Disable_Ho_W1;
				Enable_Ho_U1;    // U +
				Disable_Lo_W1;
				Disable_Lo_U1;
				Enable_Lo_V1;    // V -
				CurrentHallState=3;
			}
		}
		else if(hallph1 == 1 && hallph2 == 1 && hallph3 == 0)
		{
			if(CurrentHallState!=4)
			{
				Disable_Ho_U1;
				Disable_Ho_V1;
				Enable_Ho_W1;    // W +
				Disable_Lo_W1;
				Disable_Lo_U1;
				Enable_Lo_V1;    // V -
				CurrentHallState=4;
			}
		}
		else if(hallph1 == 1 && hallph2 == 0 && hallph3 == 0)
		{
			if(CurrentHallState!=5)
			{
				Disable_Ho_U1;
				Disable_Ho_V1;
				Enable_Ho_W1;    // W +
				Disable_Lo_W1;
				Disable_Lo_V1;
				Enable_Lo_U1;    // U -
				CurrentHallState=5;
			}
		}
		else if(hallph1 == 1 && hallph2 == 0 && hallph3 == 1)
		{
			if(CurrentHallState!=6)
			{
				Disable_Ho_U1;
				Disable_Ho_W1;
				Enable_Ho_V1;    // V +
				Disable_Lo_W1;
				Disable_Lo_V1;
				Enable_Lo_U1;    // U -
				CurrentHallState=6;
			}
		}
	}
}
//-------------------------------------------------------------------------------
// Управление по обратной ЭДС ДВИГАТЕЛЬ 1
void control_emf(void)
{
		if(delay_timeBLDC1==0) // Если задержка удержания состояния истекла (см счет задержки в таймере)
		{
			  if(back_emf_enableBLDC1==0) // Запуск с нуля (двигатель еще не вращали), прокрутим N состояний вручную
				{
					if(count_step_statesBLDC1==6)// если 84, то 6 коммутаций * 7 пар полюсов * 2 оборот двигателя = число переключений за 2 оборота
						back_emf_enableBLDC1=1;// Если прокрутили двигатель на N состояний, переходим в режим по обратной ЭДС
					count_step_statesBLDC1++;
					if(enable_stateBLDC1>0)
						enable_stateBLDC1--;
					if(enable_stateBLDC1==0)
						enable_stateBLDC1=6;
					emf_delayBLDC1=26; // Задержка для начального вращения
				}
				//-------------------------------------------------------------------------------
				// Не переинициализировать таймер (шим) если состояние не изменилось с предыдущего раза
				if(previous_stateBLDC1 != enable_stateBLDC1)
				{
					previous_stateBLDC1 = enable_stateBLDC1;
					current_stateBLDC1 = 0;
					CountStates_statesBLDC1++;
				}
				//-------------------------------------------------------------------------------
				switch(enable_stateBLDC1) // 6 состояний статора
				{
					//-------------------------------------------------------------------------------
					// Состояние 1
					case 1:
						if(back_emf_enableBLDC1)// Если переключились в режим по ЭДС, то начинаем слушать обмотки
						{
							// Отлавливаем переход фазы U из + в -(перечение средней точки)
							if(ReadPhase_U1)//Читаем состояние фазы U, (компаратор дает инвертный сигнал)
							{
								//delay_ms(emf_delay);
								enable_stateBLDC1=6;  // Назначаем следующее состояние
								delay_timeBLDC1=emf_delayBLDC1; // Назначаем задержку до перехода в следующее стостояние
								break;
							}
						}

						if(current_stateBLDC1==0) // Устанавливаем коммутацию обмоток для текущего состояния
						{
							Disable_Ho_U1;
							Disable_Ho_W1;
							Enable_Ho_V1; // Фаза V +
						    //USART_SendData(USART2, 0xF1);
							Disable_Lo_U1;
							Disable_Lo_V1;
							Enable_Lo_W1; // Фаза W -
							current_stateBLDC1 = 1; // Больше не переопределять настройки ШИМ
						}
					break;
					//-------------------------------------------------------------------------------

					//-------------------------------------------------------------------------------
					// Состояние 2
						case 2:
							if(back_emf_enableBLDC1)
							{
								if(!ReadPhase_V1)
								{
									//delay_ms(emf_delay);
									delay_timeBLDC1=emf_delayBLDC1;
									enable_stateBLDC1=1;
								}
							}
							if(current_stateBLDC1 == 0)
							{
								Disable_Ho_V1;
								Disable_Ho_W1;
								Enable_Ho_U1;
								//USART_SendData(USART2, 0xF2);
								Disable_Lo_U1;
								Disable_Lo_V1;
								Enable_Lo_W1;
								current_stateBLDC1 = 1;
							}


							break;
						//-------------------------------------------------------------------------------

					// Состояние 3
					case 3:
						if(back_emf_enableBLDC1)
						{
							if(ReadPhase_W1)//C5
							{
								//delay_ms(emf_delay);
								enable_stateBLDC1=2;
								delay_timeBLDC1=emf_delayBLDC1;
								break;
							}
						}
						if(current_stateBLDC1==0)
						{
							Disable_Ho_V1;
							Disable_Ho_W1;
							Enable_Ho_U1;
							//USART_SendData(USART2, 0xF3);
							Disable_Lo_W1;
							Disable_Lo_U1;
							Enable_Lo_V1;
							current_stateBLDC1 = 1;
						}
						break;
					//-------------------------------------------------------------------------------
					// Состояние 4
					case 4:
						// Оранжевый провод  E9 фаза U
						//
						if(back_emf_enableBLDC1)
						{
							if(!ReadPhase_U1)//ждем когда компаратор прыгнет в +
							{
								//delay_ms(emf_delay);
								enable_stateBLDC1=3;
								delay_timeBLDC1=emf_delayBLDC1;
								break;
							}
						}
						if(current_stateBLDC1==0)
						{
							Disable_Ho_U1;
							Disable_Ho_V1;
							Enable_Ho_W1;
							Disable_Lo_W1;
							Disable_Lo_U1;
							Enable_Lo_V1;
							current_stateBLDC1 = 1;
						}
						break;
					//-------------------------------------------------------------------------------
					// Состояние 5
					case 5:
						if(back_emf_enableBLDC1)
						{
							// Отлавливаем переход фазы V из - в +
							// Синий провод E7 фаза V
							if(ReadPhase_V1) // в -
							{
								//delay_ms(emf_delay);
								enable_stateBLDC1=4;
								delay_timeBLDC1=emf_delayBLDC1;
								break;
							}
						}
						if(current_stateBLDC1==0)
						{
							Disable_Ho_U1;
							Disable_Ho_V1;
							Enable_Ho_W1;
							Disable_Lo_W1;
							Disable_Lo_V1;
							Enable_Lo_U1;
							current_stateBLDC1 = 1;
						}
						break;
					//-------------------------------------------------------------------------------
					// Состояние 6
					case 6:
						if(back_emf_enableBLDC1)
						{
							if(!ReadPhase_W1)
							{
								//delay_ms(emf_delay);
								enable_stateBLDC1=5;
								delay_timeBLDC1=emf_delayBLDC1;
								break;
							}
						}
						if(current_stateBLDC1==0)
						{
							Disable_Ho_U1;
							Disable_Ho_W1;
							Enable_Ho_V1;
							Disable_Lo_W1;
							Disable_Lo_V1;
							Enable_Lo_U1;
							current_stateBLDC1 = 1;
						}
						break;
						//-------------------------------------------------------------------------------
			}
			// -------------------------------------------------------------------------------
			// END Switch
	  }
	 //-------------------------------------------------------------------------------
	 // END if delay_time
}
//-------------------------------------------------------------------------------
// END void control EMF

// ДВИГАТЕЛЬ 2
void control_emf_2(void)
{
		if(delay_timeBLDC2==0)
		{
			  if(back_emf_enableBLDC2==0)
				{
					if(count_step_statesBLDC2==6)// если 84 то 6 коммутаций * 7 пар полюсов * 2 оборот двигателя = число переключений за 2 оборота
						back_emf_enableBLDC2=1;
					count_step_statesBLDC2++;
					if(enable_stateBLDC2>0)
						enable_stateBLDC2--;
					if(enable_stateBLDC2==0)
						enable_stateBLDC2=6;
					//delay_timeBLDC2=28;
					emf_delayBLDC1=26;
				}
				//-------------------------------------------------------------------------------
				// Не переопределять таймер если состояние не изменилось с предыдущего раза
				if(previous_stateBLDC2 != enable_stateBLDC2)
				{
					previous_stateBLDC2 = enable_stateBLDC2;
					current_stateBLDC2 = 0;
					CountStates_statesBLDC2++;
				}
				//-------------------------------------------------------------------------------
				switch(enable_stateBLDC2)
				{
					//-------------------------------------------------------------------------------
					// Состояние 1
					case 1:
						if(back_emf_enableBLDC2)
						{
							// Отлавливаем переход фазы U из + в -(перечение средней точки)
							if(ReadPhase_U2)//компаратор дает инвертный сигнал
							{
								//delay_ms(emf_delay);
								enable_stateBLDC2=6;
								delay_timeBLDC2=emf_delayBLDC2;
								break;
							}
						}

						if(current_stateBLDC2==0)
						{
							Disable_Ho_U2;
							Disable_Ho_W2;
							Enable_Ho_V2;
						    //USART_SendData(USART2, 0xF1);
							Disable_Lo_U2;
							Disable_Lo_V2;
							Enable_Lo_W2;
							current_stateBLDC2 = 1;
						}
					break;
					//-------------------------------------------------------------------------------
					// Состояние 2
						case 2:
							if(back_emf_enableBLDC2)
							{
								if(!ReadPhase_V2)
								{
									//delay_ms(emf_delay);
									delay_timeBLDC2=emf_delayBLDC2;
									enable_stateBLDC2=1;
								}
							}
							if(current_stateBLDC2 == 0)
							{
								Disable_Ho_V2;
								Disable_Ho_W2;
								Enable_Ho_U2;
								//USART_SendData(USART2, 0xF2);
								Disable_Lo_U2;
								Disable_Lo_V2;
								Enable_Lo_W2;
								current_stateBLDC2 = 1;
							}
							break;
					//-------------------------------------------------------------------------------
					// Состояние 3
					case 3:
						if(back_emf_enableBLDC2)
						{
							if(ReadPhase_W2)//C5
							{
								//delay_ms(emf_delay);
								enable_stateBLDC2=2;
								delay_timeBLDC2=emf_delayBLDC2;
								break;
							}
						}
						if(current_stateBLDC2==0)
						{
							Disable_Ho_V2;
							Disable_Ho_W2;
							Enable_Ho_U2;
							//USART_SendData(USART2, 0xF3);
							Disable_Lo_W2;
							Disable_Lo_U2;
							Enable_Lo_V2;
							current_stateBLDC2 = 1;
						}
						break;
					//-------------------------------------------------------------------------------
					// Состояние 4
					case 4:
						// Оранжевый провод  E9 фаза U
						//
						if(back_emf_enableBLDC2)
						{
							if(!ReadPhase_U2)//ждем когда компаратор прыгнет в +
							{
								//delay_ms(emf_delay);
								enable_stateBLDC2=3;
								delay_timeBLDC2=emf_delayBLDC2;
								break;
							}
						}
						if(current_stateBLDC2==0)
						{
							Disable_Ho_U2;
							Disable_Ho_V2;
							Enable_Ho_W2;
							Disable_Lo_W2;
							Disable_Lo_U2;
							Enable_Lo_V2;
							current_stateBLDC2 = 1;
						}
						break;
					//-------------------------------------------------------------------------------
					// Состояние 5
					case 5:
						if(back_emf_enableBLDC2)
						{
							// Отлавливаем переход фазы V из - в +
							// Синий провод E7 фаза V
							if(ReadPhase_V2) // в -
							{
								//delay_ms(emf_delay);
								enable_stateBLDC2=4;
								delay_timeBLDC2=emf_delayBLDC2;
								break;
							}
						}
						if(current_stateBLDC2==0)
						{
							Disable_Ho_U2;
							Disable_Ho_V2;
							Enable_Ho_W2;
							Disable_Lo_W2;
							Disable_Lo_V2;
							Enable_Lo_U2;
							current_stateBLDC2 = 1;
						}
						break;
					//-------------------------------------------------------------------------------
					// Состояние 6
					case 6:
						if(back_emf_enableBLDC2)
						{
							if(!ReadPhase_W2)
							{
								//delay_ms(emf_delay);
								enable_stateBLDC2=5;
								delay_timeBLDC2=emf_delayBLDC2;
								break;
							}
						}
						if(current_stateBLDC2==0)
						{
							Disable_Ho_U2;
							Disable_Ho_W2;
							Enable_Ho_V2;
							Disable_Lo_W2;
							Disable_Lo_V2;
							Enable_Lo_U2;
							current_stateBLDC2 = 1;
						}
						break;
						//-------------------------------------------------------------------------------
			}
			// -------------------------------------------------------------------------------
			// END Switch 2
	  }
	 //-------------------------------------------------------------------------------
	 // END if delay_time 2
}
//-------------------------------------------------------------------------------

// END void control EMF 2
void disable_tim_chanels(void)
{
	TIM_CCxCmd(TIM4, TIM_Channel_1, TIM_CCx_Disable);
	TIM_CCxCmd(TIM4, TIM_Channel_2, TIM_CCx_Disable);
	TIM_CCxCmd(TIM4, TIM_Channel_3, TIM_CCx_Disable);
}
