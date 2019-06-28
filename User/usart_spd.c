#include "usart_spd.h"
#include "Modbus_svr.h"
#include "SysTick.h"

#define SPD_STATION 1   // SPDE传感器站地址
#define SPD_START_ADR 0 // SPDE传感器参数首地址
#define SPD_LENGTH 1    // SPDE传感器参数长度
#define SPD_SAVE_ADR 80 // SPDE传感器参数在wReg中的起始地址

extern u16 wReg[];

uint8_t SPD_frame[8] = {SPD_STATION, 0x03, 0x00, SPD_START_ADR, 0x00, SPD_LENGTH, 0x00, 0x00};
u8 SPD_buffer[256];
u8 SPD_curptr;
u8 SPD_bRecv;
u8 SPD_frame_len = 85;

u32 ulSPDTicks = 0;

SpeedValueQueue qspd;
//-------------------------------------------------------------------------------
//	@brief	速度值計算隊列初始化
//	@param	svq:隊列指針
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
//	@brief	速度值計算隊列初始化
//	@param	svq:隊列指針
//          val:插入隊列的值
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
//	@brief	速度值計算隊列初始化
//	@param	svq:隊列指針
//	@retval	隊列中保存數據的平均值
//-------------------------------------------------------------------------------
short SpdQueueAvgVal(SpeedValueQueue *svq)
{
    if (svq->lSum_tim == 0)
        return 0;
    return svq->lSum_ang * 1000 / svq->lSum_tim;
}

//-------------------------------------------------------------------------------
//	@brief	中断初始化
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
//	@brief	串口初始化
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
static void SPD_Config(int baud)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;

  RCC_AHB1PeriphClockCmd(SPD_USART_RX_GPIO_CLK | SPD_USART_TX_GPIO_CLK, ENABLE);

  /* 使能 USART 时钟 */
  SPD_USART_APBxClkCmd(SPD_USART_CLK, ENABLE);

  /* GPIO初始化 */
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  /* 配置Tx引脚为复用功能  */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Pin = SPD_USART_TX_PIN;
  GPIO_Init(SPD_USART_TX_GPIO_PORT, &GPIO_InitStructure);

  /* 配置Rx引脚为复用功能 */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Pin = SPD_USART_RX_PIN;
  GPIO_Init(SPD_USART_RX_GPIO_PORT, &GPIO_InitStructure);

  /* 连接 PXx 到 USARTx_Tx*/
  GPIO_PinAFConfig(SPD_USART_RX_GPIO_PORT, SPD_USART_RX_SOURCE, SPD_USART_RX_AF);

  /*  连接 PXx 到 USARTx__Rx*/
  GPIO_PinAFConfig(SPD_USART_TX_GPIO_PORT, SPD_USART_TX_SOURCE, SPD_USART_TX_AF);

  /* 配置串SPD_USART 模式 */
  /* 波特率设置：SPD_USART_BAUDRATE */
  USART_InitStructure.USART_BaudRate = baud;
  /* 字长(数据位+校验位)：8 */
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  /* 停止位：1个停止位 */
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  /* 校验位选择：不使用校验 */
  USART_InitStructure.USART_Parity = USART_Parity_No;
  /* 硬件流控制：不使用硬件流 */
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  /* USART模式控制：同时使能接收和发送 */
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  /* 完成USART初始化配置 */
  USART_Init(USART_SPD, &USART_InitStructure);

  /* 嵌套向量中断控制器NVIC配置 */
  SPD_NVIC_Configuration();

  /* 使能串口接收中断 */
  USART_ITConfig(USART_SPD, USART_IT_RXNE, ENABLE);

  /* 使能串口 */
  USART_Cmd(USART_SPD, ENABLE);
}

//-------------------------------------------------------------------------------
//	@brief	CPT通信初始化程序
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
//	@brief	发送命令帧
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
void SPD_TxCmd(void)
{
  if (SPD_bRecv == 1) //如果当前未完成接收，则通信错误计数器递增
    wReg[SPD_COM_FAIL]++;

  SPD_curptr = 0;
  SPD_bRecv = 1;
  Usart_SendBytes(USART_SPD, SPD_frame, 8);
}

//-------------------------------------------------------------------------------
//	@brief	接收数据处理
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

  //增加速度、位移、圈数
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
//	@brief	串口中断服务程序
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
void SPD_USART_IRQHandler(void)
{
  u8 ch;

  if (USART_GetITStatus(USART_SPD, USART_IT_RXNE) != RESET) //判断读寄存器是否非空
  {
    ch = USART_ReceiveData(USART_SPD); //将读寄存器的数据缓存到接收缓冲区里
    SPD_buffer[SPD_curptr++] = ch;
  }

  if (USART_GetITStatus(USART_SPD, USART_IT_TXE) != RESET)
  {
    USART_ITConfig(USART_SPD, USART_IT_TXE, DISABLE);
  }
}

//-----------------------------End of file--------------------------------------------------
