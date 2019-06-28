#include "usart_spd.h"
#include "Modbus_svr.h"
#include "SysTick.h"

#define SPD_STATION 1   // SPDE������վ��ַ
#define SPD_START_ADR 0 // SPDE�����������׵�ַ
#define SPD_LENGTH 1    // SPDE��������������
#define SPD_SAVE_ADR 80 // SPDE������������wReg�е���ʼ��ַ

extern u16 wReg[];

uint8_t SPD_frame[8] = {SPD_STATION, 0x03, 0x00, SPD_START_ADR, 0x00, SPD_LENGTH, 0x00, 0x00};
u8 SPD_buffer[256];
u8 SPD_curptr;
u8 SPD_bRecv;
u8 SPD_frame_len = 85;

u32 ulSPDTicks = 0;

SpeedValueQueue qspd;
//-------------------------------------------------------------------------------
//	@brief	�ٶ�ֵӋ����г�ʼ��
//	@param	svq:���ָ�
//	@retval	None
//-------------------------------------------------------------------------------
void SpdQueueInit(SpeedValueQueue *svq)
{
    int i;

    svq->ptr_head = 0;
    svq->ptr_tail = 0;
    svq->lSum_ang = 0;
    svq->lSum_tim = 0;
    for (i = 0; i < SPD_QUEUE_LEN; i++)
    {
        svq->queue_ang[i] = 0;
        svq->queue_tim[i] = 0;
    }
}

//-------------------------------------------------------------------------------
//	@brief	�ٶ�ֵӋ����г�ʼ��
//	@param	svq:���ָ�
//          val:������е�ֵ
//	@retval	None
//-------------------------------------------------------------------------------
void SpdQueueIn(SpeedValueQueue *svq, short ang, short tim)
{
    svq->lSum_ang += ang;
    svq->lSum_ang -= svq->queue_ang[svq->ptr_head];
    svq->lSum_tim += tim;
    svq->lSum_tim -= svq->queue_tim[svq->ptr_head];

    svq->queue_ang[svq->ptr_head] = ang;
    svq->queue_tim[svq->ptr_head] = tim;

    svq->ptr_head++;
    if (svq->ptr_head >= SPD_QUEUE_LEN)
        svq->ptr_head = 0;
}

//-------------------------------------------------------------------------------
//	@brief	�ٶ�ֵӋ����г�ʼ��
//	@param	svq:���ָ�
//	@retval	����б��攵����ƽ��ֵ
//-------------------------------------------------------------------------------
short SpdQueueAvgVal(SpeedValueQueue *svq)
{
    if (svq->lSum_tim == 0)
        return 0;
    return svq->lSum_ang * 1000 / svq->lSum_tim;
}

//-------------------------------------------------------------------------------
//	@brief	�жϳ�ʼ��
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
static void SPD_NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  NVIC_InitStructure.NVIC_IRQChannel = SPD_USART_IRQ;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

//-------------------------------------------------------------------------------
//	@brief	���ڳ�ʼ��
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
static void SPD_Config(int baud)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;

  RCC_AHB1PeriphClockCmd(SPD_USART_RX_GPIO_CLK | SPD_USART_TX_GPIO_CLK, ENABLE);

  /* ʹ�� USART ʱ�� */
  SPD_USART_APBxClkCmd(SPD_USART_CLK, ENABLE);

  /* GPIO��ʼ�� */
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  /* ����Tx����Ϊ���ù���  */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Pin = SPD_USART_TX_PIN;
  GPIO_Init(SPD_USART_TX_GPIO_PORT, &GPIO_InitStructure);

  /* ����Rx����Ϊ���ù��� */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Pin = SPD_USART_RX_PIN;
  GPIO_Init(SPD_USART_RX_GPIO_PORT, &GPIO_InitStructure);

  /* ���� PXx �� USARTx_Tx*/
  GPIO_PinAFConfig(SPD_USART_RX_GPIO_PORT, SPD_USART_RX_SOURCE, SPD_USART_RX_AF);

  /*  ���� PXx �� USARTx__Rx*/
  GPIO_PinAFConfig(SPD_USART_TX_GPIO_PORT, SPD_USART_TX_SOURCE, SPD_USART_TX_AF);

  /* ���ô�SPD_USART ģʽ */
  /* ���������ã�SPD_USART_BAUDRATE */
  USART_InitStructure.USART_BaudRate = baud;
  /* �ֳ�(����λ+У��λ)��8 */
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  /* ֹͣλ��1��ֹͣλ */
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  /* У��λѡ�񣺲�ʹ��У�� */
  USART_InitStructure.USART_Parity = USART_Parity_No;
  /* Ӳ�������ƣ���ʹ��Ӳ���� */
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  /* USARTģʽ���ƣ�ͬʱʹ�ܽ��պͷ��� */
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  /* ���USART��ʼ������ */
  USART_Init(USART_SPD, &USART_InitStructure);

  /* Ƕ�������жϿ�����NVIC���� */
  SPD_NVIC_Configuration();

  /* ʹ�ܴ��ڽ����ж� */
  USART_ITConfig(USART_SPD, USART_IT_RXNE, ENABLE);

  /* ʹ�ܴ��� */
  USART_Cmd(USART_SPD, ENABLE);
}

//-------------------------------------------------------------------------------
//	@brief	CPTͨ�ų�ʼ������
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
void SPD_Init(void)
{
  u16 uCRC;

  SPD_Config(SPD_USART_BAUDRATE);
  uCRC = CRC16(SPD_frame, 6);
  SPD_frame[6] = uCRC & 0x00FF;
  SPD_frame[7] = (uCRC & 0xFF00) >> 8;

  SPD_curptr = 0;
  SPD_bRecv = 0;
  wReg[150] = 0;
  SPD_frame_len = 2 * SPD_LENGTH + 5;

  SpdQueueInit(&qspd);
}

//-------------------------------------------------------------------------------
//	@brief	��������֡
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
void SPD_TxCmd(void)
{
  if (SPD_bRecv == 1) //�����ǰδ��ɽ��գ���ͨ�Ŵ������������
    wReg[SPD_COM_FAIL]++;

  SPD_curptr = 0;
  SPD_bRecv = 1;
  Usart_SendBytes(USART_SPD, SPD_frame, 8);
}

//-------------------------------------------------------------------------------
//	@brief	�������ݴ���
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
void SPD_Task(void)
{
  u32 tick ;
  short delta_a;

  if (SPD_curptr < SPD_frame_len)
    return;

  if (SPD_buffer[0] != SPD_STATION || SPD_buffer[1] != 0x03)
    return;

  if (SPD_buffer[2] != 2 * SPD_LENGTH)
    return;

  wReg[SPD_SAVE_ADR + 1] = wReg[SPD_SAVE_ADR];
  wReg[SPD_SAVE_ADR] = SPD_buffer[3] << 0x08 | SPD_buffer[4];

  delta_a = wReg[SPD_SAVE_ADR] - wReg[SPD_SAVE_ADR + 1];
  if ( wReg[SPD_SAVE_ADR] < 1024  && wReg[SPD_SAVE_ADR + 1] > 3072 )
  {
    delta_a += 4096;
    wReg[SPD_SAVE_ADR + 3]++;
  }
  if ( wReg[SPD_SAVE_ADR] > 3072  && wReg[SPD_SAVE_ADR + 1] < 1024 )
  {
    delta_a -= 4096;
    wReg[SPD_SAVE_ADR + 3]--;
  }

  //�����ٶȡ�λ�ơ�Ȧ��
  tick = GetCurTicks();
  wReg[SPD_COM_TICK] = tick - ulSPDTicks;
  ulSPDTicks = tick;

  SpdQueueIn(&qspd, delta_a, wReg[SPD_COM_TICK]);
  wReg[SPD_SAVE_ADR + 2] = SpdQueueAvgVal(&qspd);

  wReg[SPD_COM_SUCS]++;

  SPD_curptr = 0 ;
  SPD_bRecv = 0;
}

//-------------------------------------------------------------------------------
//	@brief	�����жϷ������
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
void SPD_USART_IRQHandler(void)
{
  u8 ch;

  if (USART_GetITStatus(USART_SPD, USART_IT_RXNE) != RESET) //�ж϶��Ĵ����Ƿ�ǿ�
  {
    ch = USART_ReceiveData(USART_SPD); //�����Ĵ��������ݻ��浽���ջ�������
    SPD_buffer[SPD_curptr++] = ch;
  }

  if (USART_GetITStatus(USART_SPD, USART_IT_TXE) != RESET)
  {
    USART_ITConfig(USART_SPD, USART_IT_TXE, DISABLE);
  }
}

//-----------------------------End of file--------------------------------------------------
