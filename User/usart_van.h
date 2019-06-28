#ifndef __VANE_USART__
#define __VANE_USART__

#include "stm32f4xx.h"

//  COM1 Define
/* ��ͬ�Ĵ��ڹ��ص����߲�һ����ʱ��ʹ�ܺ���Ҳ��һ������ֲʱҪע�� 
 * ����1��6��      RCC_APB2PeriphClockCmd
 * ����2/3/4/5/7�� RCC_APB1PeriphClockCmd
 */
#define USART_VAN USART2

#define VAN_USART_CLK RCC_APB1Periph_USART2
#define VAN_USART_APBxClkCmd RCC_APB1PeriphClockCmd
#define VAN_USART_BAUDRATE 115200 //���ڲ�����

#define VAN_USART_RX_GPIO_PORT GPIOA
#define VAN_USART_RX_GPIO_CLK RCC_AHB1Periph_GPIOA
#define VAN_USART_RX_PIN GPIO_Pin_2
#define VAN_USART_RX_AF GPIO_AF_USART2
#define VAN_USART_RX_SOURCE GPIO_PinSource2

#define VAN_USART_TX_GPIO_PORT GPIOA
#define VAN_USART_TX_GPIO_CLK RCC_AHB1Periph_GPIOA
#define VAN_USART_TX_PIN GPIO_Pin_3
#define VAN_USART_TX_AF GPIO_AF_USART2
#define VAN_USART_TX_SOURCE GPIO_PinSource3

#define VAN_USART_IRQ USART2_IRQn
#define VAN_USART_IRQHandler USART2_IRQHandler

//--------------------------------------------------------
#define VAN_BAUDRATE 171
#define VAN_COM_TICK 141
#define VAN_COM_FAIL 151
#define VAN_COM_SUCS 161

void VAN_Init(void);
void VAN_TxCmd(void);
void VAN_Task(void);
void VAN_TransData(void);

void VAN_USART_IRQHandler(void);

#endif

// --------------End of file------------------------
