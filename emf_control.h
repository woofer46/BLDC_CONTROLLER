#include "full_h_bridge.h"

//Двигатель 1 LEFT
#define ReadPhase_U1 GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_0)
#define ReadPhase_V1 GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_1)
#define ReadPhase_W1 GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_2)
//Двигатель 2 RIGHT
#define ReadPhase_U2 GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_3)
#define ReadPhase_V2 GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_4)
#define ReadPhase_W2 GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_5)
void emf_init(void);
void control_emf_2(void);
void control_emf_1(void);
