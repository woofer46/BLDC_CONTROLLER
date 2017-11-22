// Svyatoslav Mishin 2014-2017
// Brushless EMF controllers
// Anton Proskunin 2016
// Andrey Pushin 2016

#include <stm32f4xx_conf.h>
#include "hall_control.h"



uint16_t del_count=0;                                // Счетчик для фунции delay_ms
uint16_t delay_timeBLDC1=0;                          // Счетчик задержки коммутирования состояния для бездатчикового управления (Двиг 1)
uint16_t delay_timeBLDC2=0;                          // --//--//-- (Двиг 2)
uint8_t CountStates_ticks = 0;                       // Счетчик времение счета прошедших состояний (для регулятора)


uint8_t Temp_buf[6];
uint8_t Receive=0;
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

uint16_t HallCounterState = 42;
uint16_t SpeedSendTime = 0;							// тестируем измерение скорости
uint8_t count_step_statesBLDC2=0;
uint8_t enable_stateBLDC2 =0;
uint8_t back_emf_enableBLDC2=0;
uint8_t current_stateBLDC2 =0;
uint8_t previous_stateBLDC2 =0;
uint8_t CountStates_statesBLDC2 = 0;
uint8_t emf_delayBLDC2=19;
uint8_t ControlMode = 0x00;
uint8_t WayLength = 0;
uint8_t RotateAngle = 0;
uint8_t LeftPrevHallState = 0;
uint8_t RightPrevHallState = 0;
uint8_t Test14 = 0;




uint8_t hallph1;
uint8_t hallph2;
uint8_t hallph3;

uint8_t LeftSpeed=0;
uint8_t RightSpeed=0;
uint8_t LeftDir = 0;
uint8_t RightDir = 0;



uint16_t tBufFill=0;
uint16_t tBufRd=0;


uint8_t DriveMode=0x01;


uint8_t SetHallControl = 1;                       // Режим управления по датчикам холла Флаг
uint8_t CurrentHallState1 = 100;                    // Текущее состояние по датчикам холла
uint8_t CurrentHallState2 = 100;
int CurrentSpeed = 0;

void SysTick_Handler(void);
void delay_ms(uint16_t del_temp);
void USART2_IRQHandler(void);
void str_to_usart(char* str);
void control_emf(void);
void disable_tim_chanels(void);

#define SpeedMode 0x00
#define WayMode 0x01
#define AngleMode 0x02

//Двигатель 1 LEFT
#define ReadPhase_U1 GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_0)
#define ReadPhase_V1 GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_1)
#define ReadPhase_W1 GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_2)
//Двигатель 2 RIGHT
#define ReadPhase_U2 GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_3)
#define ReadPhase_V2 GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_4)
#define ReadPhase_W2 GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_5)
uint8_t dir_motor1 = 0;//r
uint8_t dir_motor2 = 0;//l
int main(void)
{
	SystemInit();
	SysTick_Config(SystemCoreClock/1000);

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
	//str_to_usart("Program is ready\n\r");
	//control_hall_motor1();

	hall_sensor_init();
    while(1)

    {
    	control_hall_motor2(dir_motor2);
		control_hall_motor1(dir_motor1);
    	//Disable_Lo_U1;
    	//Enable_Lo_U1;
    	//control_emf();
    }
}
//Обработчик прерывания юарт
uint8_t command_buffer[6];
#define BUF_SIZE 1000

uint32_t BufWr=0;
uint32_t BufFill=0;
uint32_t BufRd=0;
uint32_t Receive_buf[BUF_SIZE];
uint32_t read_index = 0;


#define LOG_SIZE 10000
uint8_t log[LOG_SIZE];
uint32_t log_w = 0;
uint32_t log_r = 0;
uint8_t log_start = 0;



void read_uart(void)
{
	char buffer[12];
	while(BufFill>5) // Если в буфере 5 байт или больше
	{
		// Теперь проверим что это наш пакет
		// 0б - 0х00 | 1б - скорость лево | 2б - скорость право | 3б - режим | 4б - \n | 5б - \r
		if ((Receive_buf[read_index] == 0x00) && (Receive_buf[read_index + 4] == 0x0A) && (Receive_buf[read_index + 5] == 0x0D))
		{
			log_start = 1;
			command_buffer[0] = Receive_buf[read_index + 1];
			command_buffer[1] = Receive_buf[read_index + 2];
			command_buffer[2] = Receive_buf[read_index + 3]; // DriveMode 0x01 - Hall, 0x02 - EMF, 0x00 - Disable

			read_index = read_index + 6;
			BufFill = BufFill - 6;

			dir_motor2 = command_buffer[0] >> 7;
			dir_motor1 = command_buffer[1] >> 7;
			if(dir_motor2 == 1)
				LeftSpeed = ~command_buffer[0] + 0x01;
			else
				LeftSpeed = command_buffer[0];

			if(dir_motor1 == 1)
				RightSpeed = ~command_buffer[1] + 0x01;
			else
				RightSpeed = command_buffer[1];

			TIM_SetCompare1(TIM1, (RightSpeed&0x7F)*5);
			TIM_SetCompare2(TIM1, (RightSpeed&0x7F)*5);
			TIM_SetCompare3(TIM1, (RightSpeed&0x7F)*5); //обрезаем 7 бит и пропорционально меняем 0-127 на 0- ~2000

			TIM_SetCompare1(TIM8, (LeftSpeed&0x7F)*5);
			TIM_SetCompare2(TIM8, (LeftSpeed&0x7F)*5);
			TIM_SetCompare3(TIM8, (LeftSpeed&0x7F)*5);


			str_to_usart(" L speed: ");
			utoa(LeftSpeed, buffer, 10);
			str_to_usart(buffer);

			str_to_usart(" L dir: ");
			utoa(dir_motor2, buffer, 10);
			str_to_usart(buffer);

			//str_to_usart(" L hall: ");
			//utoa(LeftHallCounter, buffer, 10);
			//str_to_usart(buffer);

			str_to_usart(" R speed: ");
			utoa(RightSpeed, buffer, 10);
			str_to_usart(buffer);

			str_to_usart(" R dir: ");
			utoa(dir_motor1, buffer, 10);
			str_to_usart(buffer);

			//str_to_usart(" R hall: ");
			//utoa(RightHallCounter, buffer, 10);
			//str_to_usart(buffer);



		}
		else
		{
			read_index = read_index + 1;
			BufFill = BufFill - 1;
			str_to_usart(" st 0");
		}
		if(read_index >= BUF_SIZE) // Если мы прочитали 255й байт из массива, то следующий должен быть 0й индекс массива
			read_index=0;
		if(BufWr >= BUF_SIZE)
			BufWr=0;

		utoa(BufFill, buffer, 10);
		str_to_usart(buffer);
		str_to_usart(" end  \r\n");
	}// - end of While
}


void USART2_IRQHandler(void)
{
	//-----------------------------------------------------------------------
	//Обработчик прерывания на прием с компьютера
	if(USART_GetITStatus(USART2, USART_IT_RXNE) == SET)
	{
		//-----------------------------------------------------------------------
		USART_ClearITPendingBit(USART2,USART_IT_RXNE);

		Receive_buf[BufWr] = USART_ReceiveData(USART2); // Закидываем байт в буфер
		BufFill++;                                      // Говорим что его размер увеличился на 1
		BufWr++;                                        // Чтобы следуюющий пришедший байт не перетер этот смещаем индекс на 1
		read_uart();
	}// - end of If
}// - end of Function
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
void int_to_usart(uint16_t val)
{
		while(USART_GetFlagStatus(USART2, USART_FLAG_TC)== RESET){}
		USART_SendData(USART2, val);
}

void log_to_usart()
{
	while (log_r != LOG_SIZE)
	{
		int_to_usart(log[log_r]);
		log_r++;
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
	if (SpeedSendTime < 1000)
		SpeedSendTime++;
	else
	{
		char buffer[12];


		//LeftHallCounter = 0;
		//RightHallCounter = 0;
		SpeedSendTime = 0;
	}
    /*
	if(del_count>0)
		del_count--;
	if(delay_timeBLDC1>0)
		delay_timeBLDC1--; //Отсчет задержки
	if(delay_timeBLDC2>0)
		delay_timeBLDC2--;
	if(CountStates_ticks < 250) // Считаем 250 мс
	{
		CountStates_ticks++;
	}
	else
	{
		CountStates_ticks = 0; // Если досчитали, то отправляем количество пройденых меток
		//USART_SendData(USART2, CountStates_statesBLDC2);// Для ПИД регулятора
		//CountStates_statesBLDC1 = 0;
		CountStates_statesBLDC2=0;
	}*/
}

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
	TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Disable);
	TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Disable);
	TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Disable);
}
