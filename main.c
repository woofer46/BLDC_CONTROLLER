// Svyatoslav Mishin 2014-2017
// Brushless EMF controllers
// Anton Proskunin 2016
// Andrey Pushin 2016

#include <stm32f4xx_conf.h>
#include "hall_control.h"

uint16_t del_count=0;                                // Счетчик для фунции delay_ms
uint16_t SpeedSendTime = 0;							// тестируем измерение скорости
uint8_t LeftSpeed=0;
uint8_t RightSpeed=0;

void SysTick_Handler(void);
void delay_ms(uint16_t del_temp);
void USART2_IRQHandler(void);
void str_to_usart(char* str);


#define SpeedMode 0x00
#define WayMode 0x01
#define AngleMode 0x02
#define BUF_SIZE 1000
#define LOG_SIZE 10000

uint8_t dir_motor1 = 0;//r
uint8_t dir_motor2 = 0;//l

uint8_t command_buffer[6];
uint32_t BufWr=0;
uint32_t BufFill=0;
uint32_t Receive_buf[BUF_SIZE];
uint32_t read_index = 0;
uint8_t log[LOG_SIZE];
uint32_t log_w = 0;
uint32_t log_r = 0;
uint8_t log_start = 0;

int main(void)
{
	SystemInit();
	SysTick_Config(SystemCoreClock/1000);

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


