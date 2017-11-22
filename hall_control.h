#include "full_h_bridge.h"
#include <stm32f4xx_conf.h>

// Чтение состояний датчиков холла для двигателя 1 LEFT
#define ReadStateHall1Motor1 GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_2)
#define ReadStateHall2Motor1 GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_3)
#define ReadStateHall3Motor1 GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_4)

// Чтение состояний датчиков холла для двигателя 2 RIGHT
#define ReadStateHall1Motor2 GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_7)
#define ReadStateHall2Motor2 GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_10)
#define ReadStateHall3Motor2 GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_11)

void control_hall_motor1(uint8_t dir_motor);
void control_hall_motor2(uint8_t dir_motor);
