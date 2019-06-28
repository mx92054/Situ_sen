#ifndef __SPEED_USART__
#define __SPEED_USART__

#include "stm32f4xx.h"

//  COM1 Define

/* ��ͬ�Ĵ��ڹ��ص����߲�һ����ʱ��ʹ�ܺ���Ҳ��һ������ֲʱҪע�� 
 * ����1��6��      RCC_APB2PeriphClockCmd
 * ����2/3/4/5/7�� RCC_APB1PeriphClockCmd
 */
#define USART_SPD UART7

#define SPD_USART_CLK RCC_APB1Periph_UART7
#define SPD_USART_APBxClkCmd RCC_APB1PeriphClockCmd
#define SPD_USART_BAUDRATE 38400 //���ڲ�����

#define SPD_USART_RX_GPIO_PORT GPIOE
#define SPD_USART_RX_GPIO_CLK RCC_AHB1Periph_GPIOE
#define SPD_USART_RX_PIN GPIO_Pin_7
#define SPD_USART_RX_AF GPIO_AF_UART7
#define SPD_USART_RX_SOURCE GPIO_PinSource7

#define SPD_USART_TX_GPIO_PORT GPIOE
#define SPD_USART_TX_GPIO_CLK RCC_AHB1Periph_GPIOE
#define SPD_USART_TX_PIN GPIO_Pin_8
#define SPD_USART_TX_AF GPIO_AF_UART7
#define SPD_USART_TX_SOURCE GPIO_PinSource8

#define SPD_USART_IRQ UART7_IRQn
#define SPD_USART_IRQHandler UART7_IRQHandler

#define SPD_BAUDRATE 172
#define SPD_COM_TICK 142
#define SPD_COM_FAIL 152
#define SPD_COM_SUCS 162
#define SPD_QUEUE_LEN 10

//�ٶ�Ӌ��ֵ���
typedef struct _tag_speed_queue
{
    u16 ptr_head;
    u16 ptr_tail;
    long lSum_ang;
    long lSum_tim;
    short queue_ang[SPD_QUEUE_LEN];
    short queue_tim[SPD_QUEUE_LEN];
} SpeedValueQueue;

void SPD_Init(void);
void SPD_TxCmd(void);
void SPD_Task(void);
void SPD_USART_IRQHandler(void);


//-----------------------------------------------------------------
void SpdQueueInit(SpeedValueQueue *svq);
void SpdQueueIn(SpeedValueQueue *svq, short ang, short tim);
short SpdQueueAvgVal(SpeedValueQueue *svq);


#endif
// --------------End of file------------------------
