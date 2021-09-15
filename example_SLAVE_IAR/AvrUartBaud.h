/*********************************************************************************/
/*
 * Author : Jeong Hyun Gu
 * File name : AvrUartBaud.h
*/
/*********************************************************************************/
#ifndef __AVR_UART_BAUD_H__
#define __AVR_UART_BAUD_H__
/*********************************************************************************/
#include "SysTypedef.h"
/*********************************************************************************/
#define AVR_UART_BAUD_REVISION_DATE    20200108
/*********************************************************************************/
/** REVISION HISTORY **/
/*
  2020. 01. 08.          - SysTypedef.h 적용.
  Jeong Hyun Gu

  2017. 06. 01.          - AvrUartBaudControlInit() 함수의 인수 pUBRRL, pUBRRH 타입을 아래와 같이 변경.
  Jeong Hyun Gu            'unsigned char volatile __tiny *' -> 'char *'

  2016. 11. 08.          - revision valid check 추가.
  Jeong Hyun Gu

  2016. 10. 28.          - 초기버전.
  Jeong Hyun Gu
*/
/*********************************************************************************/
/**Define**/

#define  true    1
#define  false   0
#define  null    0

#define GET_UBRR(CPU_CLOCK, BAUD, U2X)      ((CPU_CLOCK / (U2X ? 8 : 16) / BAUD) - 1)

/*********************************************************************************/
/**Enum**/

typedef enum
{
  BAUD_9600 = 0,
  BAUD_19200,
  BAUD_38400,
  BAUD_57600,
  BAUD_115200,
  BAUD_INIT,
}Enum_BaudRate;

/*********************************************************************************/
/**Struct**/

typedef struct
{
  struct
  {
    tU8 Init    :    1;                  //초기화

  }Bit;

  tU8 *pUBRRL;
  tU8 *pUBRRH;

  tU32 CpuClock;                         //CPU클록
  Enum_BaudRate Baud;
}tag_UartBaudControl;

/*********************************************************************************/
/**Function**/

tU8 AvrUartBaudControlInit(tag_UartBaudControl *BaudCtrl, tU32 CpuClock, tU8 *pUBRRL, tU8 *pUBRRH);
void AvrUartBaudChange(tag_UartBaudControl *BaudCtrl, Enum_BaudRate Baud);

/*********************************************************************************/
#endif //__AVR_UART_BAUD_H__
