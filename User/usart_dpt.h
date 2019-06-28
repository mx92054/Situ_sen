#ifndef __DPT_USART__
#define __DPT_USART__

#include "stm32f4xx.h"

//  COM1 Define

/* ��ͬ�Ĵ��ڹ��ص����߲�һ����ʱ��ʹ�ܺ���Ҳ��һ������ֲʱҪע�� 
 * ����1��6��      RCC_APB2PeriphClockCmd
 * ����2/3/4/5/7�� RCC_APB1PeriphClockCmd
 */
#define USART_DPT USART3

#define DPT_USART_CLK RCC_APB1Periph_USART3
#define DPT_USART_APBxClkCmd RCC_APB1PeriphClockCmd
#define DPT_USART_BAUDRATE 115200 //���ڲ�����

#define DPT_USART_RX_GPIO_PORT GPIOB
#define DPT_USART_RX_GPIO_CLK RCC_AHB1Periph_GPIOB
#define DPT_USART_RX_PIN GPIO_Pin_10
#define DPT_USART_RX_AF GPIO_AF_USART3
#define DPT_USART_RX_SOURCE GPIO_PinSource10

#define DPT_USART_TX_GPIO_PORT GPIOB
#define DPT_USART_TX_GPIO_CLK RCC_AHB1Periph_GPIOB
#define DPT_USART_TX_PIN GPIO_Pin_11
#define DPT_USART_TX_AF GPIO_AF_USART3
#define DPT_USART_TX_SOURCE GPIO_PinSource11

#define DPT_USART_IRQ USART3_IRQn
#define DPT_USART_IRQHandler USART3_IRQHandler

//---------------------------------------------------------------------
#define DPT_BAUDRATE 173
#define DPT_COM_TICK 143
#define DPT_COM_FAIL 153
#define DPT_COM_SUCS 163

void DPT_Init(void);
void DPT_Task(void);

void DPT_USART_IRQHandler(void);

#endif

// --------------End of file------------------------
