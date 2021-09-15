/*********************************************************************************/
/*
 * Author : Jeong Hyun Gu
 * File name : AvrUartBaud.c
*/
/*********************************************************************************/
#include "AvrUartBaud.h"
/*********************************************************************************/
#if(AVR_UART_BAUD_REVISION_DATE != 20200108)
#error wrong include file. (AvrUartBaud.h)
#endif
/*********************************************************************************/
/** Global variable **/


/*********************************************************************************/
tU8 AvrUartBaudControlInit(tag_UartBaudControl *BaudCtrl, tU32 CpuClock, tU8 *pUBRRL, tU8 *pUBRRH)
{
  /*
    1) Parameter
      - BaudCtrl : 'tag_UartBaudControl'의 인스턴스 주소.
      - CpuClock : Target CPU의 Clock Speed.
      - pUBRRL, pUBRRH : Uart Baud Rate Register의 주소.

    2) Description
      - Baud를 관리할 'tag_UartBaudControl' 인스턴스를 초기화한다.
      - 본 초기화는 한번만 실행하면 된다.
      - 다음 함수들은 반드시 본 초기화를 실행한 후 호출해야 한다.
        AvrUartBaudChange();
  */

  if((pUBRRL) == null || (pUBRRH == null))
  {
    return false;
  }

  BaudCtrl->CpuClock = CpuClock;
  BaudCtrl->Baud = BAUD_INIT;

  BaudCtrl->pUBRRL = pUBRRL;
  BaudCtrl->pUBRRH = pUBRRH;

  BaudCtrl->Bit.Init = true;

  return BaudCtrl->Bit.Init;
}
/*********************************************************************************/
void AvrUartBaudChange(tag_UartBaudControl *BaudCtrl, Enum_BaudRate Baud)
{
  /*
    1) Parameter
      - BaudCtrl : 'tag_UartBaudControl'의 인스턴스 주소.
      - Baud : 변경하고자 하는 baud rate.

    2) Description
      - 'BaudCtrl' 인스턴스의 baud rate를 변경한다.
  */

  if(BaudCtrl->Bit.Init == false)
  {
    return;
    /* error */
  }

  if(BaudCtrl->Baud != Baud)
  {
    BaudCtrl->Baud = Baud;

    switch(Baud)
    {
      default  :
      case  BAUD_9600  :
        *BaudCtrl->pUBRRH = (unsigned char) (GET_UBRR(BaudCtrl->CpuClock, 9600, 0) >> 8);
        *BaudCtrl->pUBRRL = (unsigned char) (GET_UBRR(BaudCtrl->CpuClock, 9600, 0) & 0x00FF);
      break;

      case  BAUD_19200  :
        *BaudCtrl->pUBRRH = (unsigned char) (GET_UBRR(BaudCtrl->CpuClock, 19200, 0) >> 8);
        *BaudCtrl->pUBRRL = (unsigned char) (GET_UBRR(BaudCtrl->CpuClock, 19200, 0) & 0x00FF);
      break;

      case  BAUD_38400  :
        *BaudCtrl->pUBRRH = (unsigned char) (GET_UBRR(BaudCtrl->CpuClock, 38400, 0) >> 8);
        *BaudCtrl->pUBRRL = (unsigned char) (GET_UBRR(BaudCtrl->CpuClock, 38400, 0) & 0x00FF);
      break;

      case  BAUD_57600  :
        *BaudCtrl->pUBRRH = (unsigned char) (GET_UBRR(BaudCtrl->CpuClock, 57600, 0) >> 8);
        *BaudCtrl->pUBRRL = (unsigned char) (GET_UBRR(BaudCtrl->CpuClock, 57600, 0) & 0x00FF);
      break;

      case  BAUD_115200  :
        *BaudCtrl->pUBRRH = (unsigned char) (GET_UBRR(BaudCtrl->CpuClock, 115200, 0) >> 8);
        *BaudCtrl->pUBRRL = (unsigned char) (GET_UBRR(BaudCtrl->CpuClock, 115200, 0) & 0x00FF);
      break;
    }
  }
}
/*********************************************************************************/
