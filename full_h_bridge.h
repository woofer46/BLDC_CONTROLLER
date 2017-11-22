#include <stm32f4xx_conf.h>
//Двигатель 1 LEFT
#define Disable_Ho_U1 TIM_CCxCmd(TIM8, TIM_Channel_1, TIM_CCx_Disable);
#define Disable_Ho_V1 TIM_CCxCmd(TIM8, TIM_Channel_2, TIM_CCx_Disable);
#define Disable_Ho_W1 TIM_CCxCmd(TIM8, TIM_Channel_3, TIM_CCx_Disable);

#define Enable_Ho_U1 TIM_CCxCmd(TIM8, TIM_Channel_1, TIM_CCx_Enable);
#define Enable_Ho_V1 TIM_CCxCmd(TIM8, TIM_Channel_2, TIM_CCx_Enable);
#define Enable_Ho_W1 TIM_CCxCmd(TIM8, TIM_Channel_3, TIM_CCx_Enable);

//Двигатель 2 RIGHT
#define Disable_Ho_U2 TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Disable);
#define Disable_Ho_V2 TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Disable);
#define Disable_Ho_W2 TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Disable);

#define Enable_Ho_U2 TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);
#define Enable_Ho_V2 TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);
#define Enable_Ho_W2 TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable);

//Двигатель 1 LEFT
#define Disable_Lo_U1 GPIO_ResetBits(GPIOA, GPIO_Pin_7);
#define Disable_Lo_V1 GPIO_ResetBits(GPIOB, GPIO_Pin_0);
#define Disable_Lo_W1 GPIO_ResetBits(GPIOB, GPIO_Pin_1);

#define Enable_Lo_U1 GPIO_SetBits(GPIOA, GPIO_Pin_7);
#define Enable_Lo_V1 GPIO_SetBits(GPIOB, GPIO_Pin_0);
#define Enable_Lo_W1 GPIO_SetBits(GPIOB, GPIO_Pin_1);

//Двигатель 2 RIGHT
#define Disable_Lo_U2 GPIO_ResetBits(GPIOB, GPIO_Pin_13);
#define Disable_Lo_V2 GPIO_ResetBits(GPIOB, GPIO_Pin_14);
#define Disable_Lo_W2 GPIO_ResetBits(GPIOB, GPIO_Pin_15);

#define Enable_Lo_U2 GPIO_SetBits(GPIOB, GPIO_Pin_13);
#define Enable_Lo_V2 GPIO_SetBits(GPIOB, GPIO_Pin_14);
#define Enable_Lo_W2 GPIO_SetBits(GPIOB, GPIO_Pin_15);

