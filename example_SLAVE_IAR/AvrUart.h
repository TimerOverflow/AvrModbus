/*********************************************************************************/
/*
 * Author : Jeong Hyun Gu
 * File name : AvrUart.h
*/
/*********************************************************************************/
#ifndef __AVR_UART_H__
#define __AVR_UART_H__
/*********************************************************************************/
#include "SysTypedef.h"
/*********************************************************************************/
#define AVR_UART_REVISION_DATE    20210521
/*********************************************************************************/
/** REVISION HISTORY **/
/*
  2021. 05. 21.          - 485 ENABLE핀 제어에 CRITICAL SECTION 적용.
  Jeong Hyun Gu

  2020. 08. 26.          - 사용자 정의 RS-485 ENABLE핀 제어 함수 추가.
  Jeong Hyun Gu

  2019. 10. 10.          - AvrUartCheckTx(), AvrUartCheckRx() 매크로 함수로 변경.
  Jeong Hyun Gu          - AvrUartControlTxEnd() 추가. TX 종료 후 RX로 전환되기까지
                          지연 시간 추가.
                         - AvrUartControlTxEnd() 함수가 AvrUartFixTxEnableFloating()를
                          대체할 수 있기 때문에 삭제, 하위 호환을 위해 AvrUartFixTxEnableFloating()를
                          호출하면 AvrUartControlTxEnd()가 호출 되도록 매크로 함수 추가.
                         - crc16(20191007) 버전 대응 위해 SysTypedef.h 적용.

  2017. 07. 26.          - AvrUartTxQueueControl() 송신완료 후 TxQue::InPtr과 TxQue::OutPtr이 다를 경우
  Jeong Hyun Gu           버퍼 초기화 실행.

  2017. 03. 22.          - MoveBufPointer() 함수 삭제하고, 매크로 함수로 대체.
  Jeong Hyun Gu

  2017. 02. 24.          - AvrUartViewRxBuf() 함수 삭제.
  Jeong Hyun Gu

  2017. 01. 02.          - AvrUartFixTxEnableFloating() 함수 추가.
  Jeong Hyun Gu

  2016. 12. 07.          - MoveBufPointer() 함수의 인수 'Dir' 삭제.
  Jeong Hyun Gu          - AvrUartGetData() 함수의 인수명 변경 'BufSize' -> 'Length'
                         - AvrUartClearRx() 함수 삭제.
                         - AvrUartClearQueueBuf() 함수 추가.
                         - 주석 추가.

  2016. 11. 17.          - 'AVR_REGISTER' 타입 삭제 -> 'char *' 타입으로 변경.
  Jeong Hyun Gu

  2016. 11. 08.          - 변수명 변경. AvrUartLinkRegister() 함수의 인수 'pTxPort' -> 'pEnablePort'
  Jeong Hyun Gu          - 변수명 변경. AvrUartLinkRegister() 함수의 인수 'TxPin' -> 'EnablePin'
                        - revision valid check 추가.

  2016. 11. 02.          - 변수명 변경. 'ReceivingDelay_us' -> 'ReceivingDelay'
  Jeong Hyun Gu

  2016. 10. 28.          - 초기버전.
  Jeong Hyun Gu
*/
/*********************************************************************************/
/**Define**/

#define  __CRC16_TARGET_UNKNOWN__               0
#define __CRC16_TARGET_IAR_AVR__                1
#define __CRC16_TARGET_MICROCHIP_STUDIO__       2
#define __CRC16_TARGET_COMPILER__               __CRC16_TARGET_IAR_AVR__

#define false       0
#define true        1
#define null        0

/*********************************************************************************/
/**Enum**/


/*********************************************************************************/
/**Struct**/

typedef struct
{
  tU8 *Buf;
  tU8 *InPtr;
  tU8 *OutPtr;
  tU16 Ctr;
  tU16 Size;
}tag_AvrUartRingBuf;

typedef struct
{
  struct
  {
    tU8 InitRegister       :      1;
    tU8 InitBuffer         :      1;
    tU8 InitGeneral        :      1;
    tU8 LinkUserEnPinCtrl  :      1;
    tU8 InitComplete       :      1;

    tU8 DataSend           :      1;
  }Bit;

  tU8 *pUDR;
  tU8 *pUCSRA;
  tU8 *pEnablePort;
  tU8 EnablePin;

  tU32 ReceivingDelay;
  tU32 ReceivingCnt;

  tag_AvrUartRingBuf TxQueue;
  tag_AvrUartRingBuf RxQueue;

  tU16 TxEndDelay;
  tU16 TxEndCnt;

  void (*TurnOnEnPin)(tU8 OnFlag);
}tag_AvrUartCtrl;

/*********************************************************************************/
/**Function**/

tU8 AvrUartLinkRegister(tag_AvrUartCtrl *Com, tU8 *pUDR, tU8 *pUCSRA, tU8 *pEnablePort, tU8 EnablePin);
tU8 AvrUartLinkBuffer(tag_AvrUartCtrl *Com, tU8 *TxBuf, tU16 TxBufSize, tU8 *RxBuf, tU16 RxBufSize);
tU8 AvrUartGeneralInit(tag_AvrUartCtrl *Com);
tU8 AvrUartLinkUserEnPinCtrl(tag_AvrUartCtrl *Com, void (*TurnOnEnPin)(tU8 OnFlag));
#define AvrUartSetTxEndDelay(Com, Delay_us, MainLoopTick_us)              ((Com)->TxEndDelay = Delay_us / MainLoopTick_us)

void AvrUartPutData(tag_AvrUartCtrl *Com, tU8 *Buf, tU16 Length);
void AvrUartPutChar(tag_AvrUartCtrl *Com, tU8 Char);


void AvrUartTxQueueControl(tag_AvrUartCtrl *Com);
void AvrUartRxQueueControl(tag_AvrUartCtrl *Com);

void AvrUartStartTx(tag_AvrUartCtrl *Com);
#define AvrUartCheckTx(Com)            ((Com)->TxQueue.Ctr)
#define AvrUartCheckRx(Com)            ((Com)->RxQueue.Ctr)

void AvrUartGetChar(tag_AvrUartCtrl *Com, tU8 *Char);
void AvrUartGetData(tag_AvrUartCtrl *Com, tU8 *Buf, tU16 Length);

void AvrUartClearQueueBuf(tag_AvrUartRingBuf *Queue);
tU8 AvrUartCheckReceiving(tag_AvrUartCtrl *Com);
void AvrUartControlTxEnd(tag_AvrUartCtrl *Com);

#define AvrUartFixTxEnableFloating(Com)          AvrUartControlTxEnd(Com)

/*********************************************************************************/
#endif //__AVR_UART_H__
