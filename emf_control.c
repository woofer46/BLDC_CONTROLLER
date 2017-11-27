#include "emf_control.h"

uint8_t emf_delayBLDC1 = 19;                         // ����� ��������� ��������� (��� ���)
uint8_t back_emf_enableBLDC1 = 0;                    // ���� ���������� ���������� �� ���, ����� ���� ��� ������ �������� �����
uint8_t count_step_statesBLDC1 = 0;                  // ���������� ��������� � �������� ������ (��� ���)
uint8_t enable_stateBLDC1 = 0;                       // ���������� �������� ������� ��������� 0, 1, 2, 3, 4, 5
uint16_t delay_timeBLDC1 = 0;                        // ������� �������� �������������� ��������� ��� �������������� ���������� (���� 1)
uint8_t previous_stateBLDC1 = 0;                     // ���������� ���������
uint8_t current_stateBLDC1 = 0;                      // ���� ���� ��� ��������� �� ���������� � ���������� ����
uint8_t CountStates_statesBLDC1 = 0;                 // ������� ���������� ������������ �� ���������� (��� ����������)
uint16_t delay_timeBLDC2 = 0;                        // --//--//-- (���� 2)
uint8_t count_step_statesBLDC2 = 0;
uint8_t enable_stateBLDC2 = 0;
uint8_t previous_stateBLDC2 = 0;
uint8_t CountStates_statesBLDC2 = 0;
uint8_t emf_delayBLDC2 = 19;
uint8_t back_emf_enableBLDC2 = 0;
uint8_t current_stateBLDC2 = 0;
void emf_init(void)
{
	//-----------------------------------------------------------------------
	// ��������� 1, ���� � ������������ P�0, P�1, P�2
	GPIO_InitTypeDef GPIO_InitComparator;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	GPIO_InitComparator.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2;
	GPIO_InitComparator.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitComparator.GPIO_OType = GPIO_OType_PP;
	GPIO_InitComparator.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitComparator.GPIO_PuPd = GPIO_PuPd_UP;//���������
	GPIO_Init(GPIOC, &GPIO_InitComparator);
	//-----------------------------------------------------------------------
	// ��������� 2, ���� � ������������ P�3, P�4, P�5
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	GPIO_InitComparator.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitComparator.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitComparator.GPIO_OType = GPIO_OType_PP;
	GPIO_InitComparator.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitComparator.GPIO_PuPd = GPIO_PuPd_UP;//���������
	GPIO_Init(GPIOC, &GPIO_InitComparator);
}


// ���������� �� �������� ��� ��������� 1
void control_emf(void)
{
		if(delay_timeBLDC1==0) // ���� �������� ��������� ��������� ������� (�� ���� �������� � �������)
		{
			  if(back_emf_enableBLDC1==0) // ������ � ���� (��������� ��� �� �������), ��������� N ��������� �������
				{
					if(count_step_statesBLDC1==6)// ���� 84, �� 6 ���������� * 7 ��� ������� * 2 ������ ��������� = ����� ������������ �� 2 �������
						back_emf_enableBLDC1=1;// ���� ���������� ��������� �� N ���������, ��������� � ����� �� �������� ���
					count_step_statesBLDC1++;
					if(enable_stateBLDC1>0)
						enable_stateBLDC1--;
					if(enable_stateBLDC1==0)
						enable_stateBLDC1=6;
					emf_delayBLDC1=26; // �������� ��� ���������� ��������
				}
				//-------------------------------------------------------------------------------
				// �� �������������������� ������ (���) ���� ��������� �� ���������� � ����������� ����
				if(previous_stateBLDC1 != enable_stateBLDC1)
				{
					previous_stateBLDC1 = enable_stateBLDC1;
					current_stateBLDC1 = 0;
					CountStates_statesBLDC1++;
				}
				//-------------------------------------------------------------------------------
				switch(enable_stateBLDC1) // 6 ��������� �������
				{
					//-------------------------------------------------------------------------------
					// ��������� 1
					case 1:
						if(back_emf_enableBLDC1)// ���� ������������� � ����� �� ���, �� �������� ������� �������
						{
							// ����������� ������� ���� U �� + � -(��������� ������� �����)
							if(ReadPhase_U1)//������ ��������� ���� U, (���������� ���� ��������� ������)
							{
								//delay_ms(emf_delay);
								enable_stateBLDC1=6;  // ��������� ��������� ���������
								delay_timeBLDC1=emf_delayBLDC1; // ��������� �������� �� �������� � ��������� ����������
								break;
							}
						}

						if(current_stateBLDC1==0) // ������������� ���������� ������� ��� �������� ���������
						{
							Disable_Ho_U1;
							Disable_Ho_W1;
							Enable_Ho_V1; // ���� V +
						    //USART_SendData(USART2, 0xF1);
							Disable_Lo_U1;
							Disable_Lo_V1;
							Enable_Lo_W1; // ���� W -
							current_stateBLDC1 = 1; // ������ �� �������������� ��������� ���
						}
					break;
					//-------------------------------------------------------------------------------

					//-------------------------------------------------------------------------------
					// ��������� 2
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

					// ��������� 3
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
					// ��������� 4
					case 4:
						// ��������� ������  E9 ���� U
						//
						if(back_emf_enableBLDC1)
						{
							if(!ReadPhase_U1)//���� ����� ���������� ������� � +
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
					// ��������� 5
					case 5:
						if(back_emf_enableBLDC1)
						{
							// ����������� ������� ���� V �� - � +
							// ����� ������ E7 ���� V
							if(ReadPhase_V1) // � -
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
					// ��������� 6
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

// ��������� 2
void control_emf_2(void)
{
		if(delay_timeBLDC2==0)
		{
			  if(back_emf_enableBLDC2==0)
				{
					if(count_step_statesBLDC2==6)// ���� 84 �� 6 ���������� * 7 ��� ������� * 2 ������ ��������� = ����� ������������ �� 2 �������
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
				// �� �������������� ������ ���� ��������� �� ���������� � ����������� ����
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
					// ��������� 1
					case 1:
						if(back_emf_enableBLDC2)
						{
							// ����������� ������� ���� U �� + � -(��������� ������� �����)
							if(ReadPhase_U2)//���������� ���� ��������� ������
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
					// ��������� 2
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
					// ��������� 3
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
					// ��������� 4
					case 4:
						// ��������� ������  E9 ���� U
						//
						if(back_emf_enableBLDC2)
						{
							if(!ReadPhase_U2)//���� ����� ���������� ������� � +
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
					// ��������� 5
					case 5:
						if(back_emf_enableBLDC2)
						{
							// ����������� ������� ���� V �� - � +
							// ����� ������ E7 ���� V
							if(ReadPhase_V2) // � -
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
					// ��������� 6
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
