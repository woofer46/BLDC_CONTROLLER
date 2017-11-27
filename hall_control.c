#include "hall_control.h"
uint8_t hall_state_motor1= 0;
uint8_t state_m1 = 0;
uint8_t prev_state_m1 = 100;

uint8_t hall_state_motor2= 0;
uint8_t state_m2 = 0;
uint8_t prev_state_m2 = 100;

uint16_t LeftHallCounter = 0;
uint16_t RightHallCounter = 0;
void hall_sensor_init(void)
{
	init_h_bridge();
	GPIO_InitTypeDef GPIO_InitStructure; // —тукрутра инициализации GPIO
	//-----------------------------------------------------------------------
	// ¬ход с датчиков ’олла, двигатель 1
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	//-----------------------------------------------------------------------
	// ¬ход с датчиков ’олла, двигатель 2
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	//-----------------------------------------------------------
}
//-------------------------------------------------------------------------------
// ”правление по датчикам холла ƒ¬»√ј“≈Ћ№ 1 Right
void control_hall_motor1(uint8_t dir_motor)//Right motor
{
	hall_state_motor1 = 0;
	hall_state_motor1 = (ReadStateHall1Motor1) | (ReadStateHall2Motor1<<1) | (ReadStateHall3Motor1<<2);
	if(dir_motor == 1)
		hall_state_motor1 = (~hall_state_motor1) & 0x07;

	if(hall_state_motor1 == 0b110) // 011
		state_m1 = 1;
	else if(hall_state_motor1 == 0b010) // 010
		state_m1 = 2;
	else if(hall_state_motor1 == 0b011) // 110
		state_m1 = 3;
	else if(hall_state_motor1 == 0b001) // 100
		state_m1 = 4;
	else if(hall_state_motor1 == 0b101) // 101
		state_m1 = 5;
	else if(hall_state_motor1 == 0b100) // h1 = 0 h2 = 0 h3 = 1
		state_m1 = 6;

	if(state_m1 != prev_state_m1)
	{
		prev_state_m1 = state_m1;
		RightHallCounter++;
		switch(state_m1)
		{
			case 1:
				Disable_Ho_U1;
				Disable_Ho_V1;
				Disable_Lo_W1;
				Disable_Lo_U1;
				Enable_Ho_W1;    // W +
				Enable_Lo_V1;    // V -
				break;
			case 2:
				Disable_Lo_V1;
				Disable_Ho_U1;
				Disable_Ho_V1;
				Disable_Lo_W1;
				Enable_Ho_W1;    // W +
				Enable_Lo_U1;    // U -
				break;
			case 3:
				Disable_Ho_W1;
				Disable_Ho_U1;
				Disable_Lo_W1;
				Disable_Lo_V1;
				Enable_Ho_V1;    // V +
				Enable_Lo_U1;    // U -
				break;
			case 4:
				Disable_Lo_U1;
				Disable_Ho_U1;
				Disable_Ho_W1;
				Disable_Lo_V1;
				Enable_Ho_V1;   // V +
				Enable_Lo_W1;   // W -
				break;
			case 5:
				Disable_Ho_V1;
				Disable_Ho_W1;
				Disable_Lo_U1;
				Disable_Lo_V1;
				Enable_Ho_U1;   // U +
				Enable_Lo_W1;   // W -
				break;
			case 6:
				Disable_Lo_W1;
				Disable_Ho_V1;
				Disable_Ho_W1;
				Disable_Lo_U1;
				Enable_Ho_U1;    // U +
				Enable_Lo_V1;    // V -
				break;
		}
	}
}
//-------------------------------------------------------------------------------
// ”правление по датчикам холла ƒ¬»√ј“≈Ћ№ 2 Left
void control_hall_motor2(uint8_t dir_motor)
{
	hall_state_motor2 = 0;
	hall_state_motor2 = (ReadStateHall1Motor2) | (ReadStateHall2Motor2<<1) | (ReadStateHall3Motor2<<2);
	if(dir_motor == 0)
		hall_state_motor2 = (~hall_state_motor2) & 0x07;

	if(hall_state_motor2 == 0b110) // 011
		state_m2 = 1;
	else if(hall_state_motor2 == 0b010) // 010
		state_m2 = 2;
	else if(hall_state_motor2 == 0b011) // 110
		state_m2 = 3;
	else if(hall_state_motor2 == 0b001) // 100
		state_m2 = 4;
	else if(hall_state_motor2 == 0b101) // 101
		state_m2 = 5;
	else if(hall_state_motor2 == 0b100) // h1 = 0 h2 = 0 h3 = 1
		state_m2 = 6;

	if(state_m2 != prev_state_m2)
	{
		prev_state_m2 = state_m2;
		LeftHallCounter++;
		switch(state_m2)
		{
			case 1:
				Disable_Ho_U2;
				Disable_Ho_V2;
				Disable_Lo_W2;
				Disable_Lo_U2;
				Enable_Ho_W2;    // W +
				Enable_Lo_V2;    // V -
				break;
			case 2:
				Disable_Lo_V2;
				Disable_Ho_U2;
				Disable_Ho_V2;
				Disable_Lo_W2;
				Enable_Ho_W2;    // W +
				Enable_Lo_U2;    // U -
				break;
			case 3:
				Disable_Ho_W2;
				Disable_Ho_U2;
				Disable_Lo_W2;
				Disable_Lo_V2;
				Enable_Ho_V2;    // V +
				Enable_Lo_U2;    // U -
				break;
			case 4:
				Disable_Lo_U2;
				Disable_Ho_U2;
				Disable_Ho_W2;
				Disable_Lo_V2;
				Enable_Ho_V2;   // V +
				Enable_Lo_W2;   // W -
				break;
			case 5:
				Disable_Ho_V2;
				Disable_Ho_W2;
				Disable_Lo_U2;
				Disable_Lo_V2;
				Enable_Ho_U2;   // U +
				Enable_Lo_W2;   // W -
				break;
			case 6:
				Disable_Lo_W2;
				Disable_Ho_V2;
				Disable_Ho_W2;
				Disable_Lo_U2;
				Enable_Ho_U2;    // U +
				Enable_Lo_V2;    // V -
				break;
		}
	}
}
//-------------------------------------------------------------------------------
