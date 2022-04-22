/*********************************************************************************/
/*
 * Author : Jeong Hyun Gu
 * File name : AvrModbus.c
*/
/*********************************************************************************/
#include <stdlib.h>
#include <string.h>
#include "AvrModbus.h"
#include "crc16.h"
/*********************************************************************************/
#if(AVR_MODBUS_REVISION_DATE != 20220422)
#error wrong include file. (AvrModbus.h)
#endif
/*********************************************************************************/
/** Global variable **/

/*********************************************************************************/
#if(AVR_MODBUS_SLAVE == true)

static tU8 CheckAllOfSlaveInit(tag_AvrModbusSlaveCtrl *Slave)
{
  /*
    1) 인수
      - Slave : tag_AvrModbusSlaveCtrl 인스턴스의 주소.

    2) 반환
      - 0  : 초기화 실패.
      - 1  :  초기화 성공.

    3) 설명
      - 'tag_AvrModbusSlaveCtrl' 인스턴스의 필수 항목 초기화 여부 확인.
  */

  return (Slave->Bit.InitGeneral) ? true : false;
}
/*********************************************************************************/
static void ErrorException(tag_AvrModbusSlaveCtrl *Slave, tU8 ErrCode)
{
  tag_AvrUartRingBuf *TxQue = &Slave->Uart->TxQueue;
  tag_AvrUartRingBuf *RxQue = &Slave->Uart->RxQueue;
  tU16 Crc16;

  /*
    1) 인수
      - Slave : tag_AvrModbusSlaveCtrl 인스턴스의 주소.
      - ErrCode : 에러 코드.

    2) 반환
      - 없음.

    3) 설명
      - Master의 요청에 문제가 있을 때 지정한 에러코드를 응답함.
  */

  if((RxQue->Buf[0] != 0) && (RxQue->Buf[0] != 255))
  {
    AvrUartPutChar(Slave->Uart, RxQue->Buf[0]);
    AvrUartPutChar(Slave->Uart, RxQue->Buf[1] | 0x80);
    AvrUartPutChar(Slave->Uart, ErrCode);

    Crc16 = Crc16Check(TxQue->OutPtr, TxQue->Buf, &TxQue->Buf[TxQue->Size - 1], TxQue->Ctr);

    AvrUartPutChar(Slave->Uart, (Crc16 >> 8));
    AvrUartPutChar(Slave->Uart, (Crc16 & 0x00FF));
    AvrUartStartTx(Slave->Uart);
  }
}
/*********************************************************************************/
static void SlaveReadRegisters(tag_AvrModbusSlaveCtrl *Slave, tag_QueryResponseInfo *QueryResponseInfo)
{
  tag_AvrUartRingBuf *TxQue = &Slave->Uart->TxQueue;
  tag_AvrUartRingBuf *RxQue = &Slave->Uart->RxQueue;
  tU16 StartAddr, NumberOfPoint, Crc16, i, MapStartAddr;
  tU8 *BaseAddr;

  /*
    1) 인수
      - Slave : tag_AvrModbusSlaveCtrl 인스턴스의 주소.

    2) 반환
      - 없음.

    3) 설명
      - Master의 'ReadHolding' 명령에 대한 처리.
  */

  StartAddr = (tU16) (RxQue->Buf[2] << 8) + RxQue->Buf[3];
  NumberOfPoint = (tU16) (RxQue->Buf[4] << 8) + RxQue->Buf[5];
  MapStartAddr = StartAddr == 0 ? 0 : QueryResponseInfo->MapStartAddr;

  if(StartAddr < MapStartAddr)
  {
    ErrorException(Slave, 2);
  }
  else
  {
    StartAddr -= MapStartAddr;
    NumberOfPoint *= 2;
    BaseAddr = (tU8 *) (((tU16 *) QueryResponseInfo->BaseAddr) + StartAddr);
    
    AvrUartPutChar(Slave->Uart, RxQue->Buf[0]);    //Slave Address
    AvrUartPutChar(Slave->Uart, RxQue->Buf[1]);    //Function
    AvrUartPutChar(Slave->Uart, NumberOfPoint);    //Byte Count
  
    for(i = 0; i < NumberOfPoint; i += 2)
    {
      AvrUartPutChar(Slave->Uart, *(BaseAddr + i + 1));
      AvrUartPutChar(Slave->Uart, *(BaseAddr + i));
    }
  
    if(Slave->Bit.InitCustomFrameCheck)
    {
      Crc16 = Slave->CustomFrameCheck(TxQue, TxQue->Ctr);
    }
    else
    {
      Crc16 = Crc16Check(TxQue->OutPtr, TxQue->Buf, &TxQue->Buf[TxQue->Size - 1], TxQue->Ctr);
    }
  
    AvrUartPutChar(Slave->Uart, (Crc16 >> 8));
    AvrUartPutChar(Slave->Uart, (Crc16 & 0x00FF));
    AvrUartStartTx(Slave->Uart);
  }
}
/*********************************************************************************/
static void SlaveReadStatus(tag_AvrModbusSlaveCtrl *Slave, tag_QueryResponseInfo *QueryResponseInfo)
{
  tag_AvrUartRingBuf *TxQue = &Slave->Uart->TxQueue;
  tag_AvrUartRingBuf *RxQue = &Slave->Uart->RxQueue;
  tU16 StartAddr, NumberOfPoint, Crc16, i, MapStartAddr;
  tU8 *BaseAddr;
  
  StartAddr = (tU16) (RxQue->Buf[2] << 8) + RxQue->Buf[3];
  NumberOfPoint = (tU16) (RxQue->Buf[4] << 8) + RxQue->Buf[5];
  MapStartAddr = StartAddr == 0 ? 0 : QueryResponseInfo->MapStartAddr;

  if(StartAddr < MapStartAddr)
  {
    ErrorException(Slave, 2);
  }
  else
  {
    StartAddr -= MapStartAddr;
    NumberOfPoint = (NumberOfPoint / 8) + ((NumberOfPoint % 8) ? 1 : 0);
    BaseAddr = (tU8 *) (((tU16 *) QueryResponseInfo->BaseAddr) + StartAddr);
    
    AvrUartPutChar(Slave->Uart, RxQue->Buf[0]);    //Slave Address
    AvrUartPutChar(Slave->Uart, RxQue->Buf[1]);    //Function
    AvrUartPutChar(Slave->Uart, NumberOfPoint);    //Byte Count
  
    for(i = 0; i < NumberOfPoint; i++)
    {
      AvrUartPutChar(Slave->Uart, BaseAddr[i]);
    }
  
    if(Slave->Bit.InitCustomFrameCheck)
    {
      Crc16 = Slave->CustomFrameCheck(TxQue, TxQue->Ctr);
    }
    else
    {
      Crc16 = Crc16Check(TxQue->OutPtr, TxQue->Buf, &TxQue->Buf[TxQue->Size - 1], TxQue->Ctr);
    }
  
    AvrUartPutChar(Slave->Uart, (Crc16 >> 8));
    AvrUartPutChar(Slave->Uart, (Crc16 & 0x00FF));
    AvrUartStartTx(Slave->Uart);
  }
}
/*********************************************************************************/
static void SlaveForceSingleCoil(tag_AvrModbusSlaveCtrl *Slave, tag_QueryResponseInfo *QueryResponseInfo)
{
  tag_AvrUartRingBuf *TxQue = &Slave->Uart->TxQueue;
  tag_AvrUartRingBuf *RxQue = &Slave->Uart->RxQueue;
  tU16 RegisterAddr, ForceData, Crc16, Move;
  tU8 *BaseAddr;

  RegisterAddr = (tU16) (RxQue->Buf[2] << 8) + RxQue->Buf[3];
  ForceData = (tU16) (RxQue->Buf[4] << 8) + RxQue->Buf[5];

  if(((Slave->Bit.InitCheckOutRange == true) && (Slave->CheckOutRange(RegisterAddr, 1) == true)) || (RegisterAddr < QueryResponseInfo->MapStartAddr))
  {
    ErrorException(Slave, 2);
  }
  else
  {
    Move = RegisterAddr - QueryResponseInfo->MapStartAddr;
    BaseAddr = QueryResponseInfo->BaseAddr + (Move / 8);
    *BaseAddr = ForceData ? *BaseAddr | (1 << (Move % 8)) : *BaseAddr & ~(1 << (Move % 8));
    
    if(Slave->Bit.InitUserException == true)
    {
      Slave->UserException(RegisterAddr, 1);
    }

    if(RxQue->Buf[0] != 0)
    {
      AvrUartPutChar(Slave->Uart, RxQue->Buf[0]);    //Slave Address
      AvrUartPutChar(Slave->Uart, RxQue->Buf[1]);    //Function
      AvrUartPutChar(Slave->Uart, RxQue->Buf[2]);    //Register Address Hi
      AvrUartPutChar(Slave->Uart, RxQue->Buf[3]);    //Register Address Lo
      AvrUartPutChar(Slave->Uart, RxQue->Buf[4]);    //Preset Data Hi
      AvrUartPutChar(Slave->Uart, RxQue->Buf[5]);    //Preset Data Lo

      if(Slave->Bit.InitCustomFrameCheck)
      {
        Crc16 = Slave->CustomFrameCheck(TxQue, TxQue->Ctr);
      }
      else
      {
        Crc16 = Crc16Check(TxQue->OutPtr, TxQue->Buf, &TxQue->Buf[TxQue->Size - 1], TxQue->Ctr);
      }

      AvrUartPutChar(Slave->Uart, (Crc16 >> 8));
      AvrUartPutChar(Slave->Uart, (Crc16 & 0x00FF));
      AvrUartStartTx(Slave->Uart);
    }
  }
}
/*********************************************************************************/
static void SlavePresetSingle(tag_AvrModbusSlaveCtrl *Slave, tag_QueryResponseInfo *QueryResponseInfo)
{
  tag_AvrUartRingBuf *TxQue = &Slave->Uart->TxQueue;
  tag_AvrUartRingBuf *RxQue = &Slave->Uart->RxQueue;
  tU16 RegisterAddr, PresetData, *BaseAddr;
  tU16 Crc16;

  /*
    1) 인수
      - Slave : tag_AvrModbusSlaveCtrl 인스턴스의 주소.

    2) 반환
      - 없음.

    3) 설명
      - Master의 'PresetSingle' 명령에 대한 처리.
  */

  RegisterAddr = (tU16) (RxQue->Buf[2] << 8) + RxQue->Buf[3];
  PresetData = (tU16) (RxQue->Buf[4] << 8) + RxQue->Buf[5];

  if(((Slave->Bit.InitCheckOutRange == true) && (Slave->CheckOutRange(RegisterAddr, 1) == true)) || (RegisterAddr < QueryResponseInfo->MapStartAddr))
  {
    ErrorException(Slave, 2);
  }
  else
  {
    BaseAddr = ((tU16 *) QueryResponseInfo->BaseAddr) + (RegisterAddr - QueryResponseInfo->MapStartAddr);
    *BaseAddr = PresetData;

    if(Slave->Bit.InitUserException == true)
    {
      Slave->UserException(RegisterAddr, 1);
    }

    if(RxQue->Buf[0] != 0)
    {
      AvrUartPutChar(Slave->Uart, RxQue->Buf[0]);    //Slave Address
      AvrUartPutChar(Slave->Uart, RxQue->Buf[1]);    //Function
      AvrUartPutChar(Slave->Uart, RxQue->Buf[2]);    //Register Address Hi
      AvrUartPutChar(Slave->Uart, RxQue->Buf[3]);    //Register Address Lo
      AvrUartPutChar(Slave->Uart, RxQue->Buf[4]);    //Preset Data Hi
      AvrUartPutChar(Slave->Uart, RxQue->Buf[5]);    //Preset Data Lo

      if(Slave->Bit.InitCustomFrameCheck)
      {
        Crc16 = Slave->CustomFrameCheck(TxQue, TxQue->Ctr);
      }
      else
      {
        Crc16 = Crc16Check(TxQue->OutPtr, TxQue->Buf, &TxQue->Buf[TxQue->Size - 1], TxQue->Ctr);
      }

      AvrUartPutChar(Slave->Uart, (Crc16 >> 8));
      AvrUartPutChar(Slave->Uart, (Crc16 & 0x00FF));
      AvrUartStartTx(Slave->Uart);
    }
  }
}
/*********************************************************************************/
static void SlavePresetMultiple(tag_AvrModbusSlaveCtrl *Slave, tag_QueryResponseInfo *QueryResponseInfo)
{
  tag_AvrUartRingBuf *TxQue = &Slave->Uart->TxQueue;
  tag_AvrUartRingBuf *RxQue = &Slave->Uart->RxQueue;
  tU16 StartAddr, NumberOfRegister, Crc16, Length, MapStartAddr, i, j = 7;
  tU8 *BaseAddr;

  /*
    1) 인수
      - Slave : tag_AvrModbusSlaveCtrl 인스턴스의 주소.

    2) 반환
      - 없음.

    3) 설명
      - Master의 'PresetMultiple' 명령에 대한 처리.
  */

  StartAddr = (RxQue->Buf[2] << 8) + RxQue->Buf[3];
  NumberOfRegister = (RxQue->Buf[4] << 8) + RxQue->Buf[5];
  MapStartAddr = StartAddr == 0 ? 0 : QueryResponseInfo->MapStartAddr;

  if(((Slave->Bit.InitCheckOutRange == true) && (Slave->CheckOutRange(StartAddr, NumberOfRegister) == true)) || (StartAddr < MapStartAddr))
  {
    ErrorException(Slave, 2);
  }
  else
  {
    Length = NumberOfRegister * 2;
    Length = (Length > (Slave->Uart->RxQueue.Size - 9)) ? (Slave->Uart->RxQueue.Size - 9) : Length;
    BaseAddr = (tU8 *) (((tU16 *) QueryResponseInfo->BaseAddr) + (StartAddr - MapStartAddr));

    for(i = 0; i < Length; i += 2)
    {
      *(BaseAddr + i + 1) = RxQue->Buf[j++];
      *(BaseAddr + i) = RxQue->Buf[j++];
    }

    if(Slave->Bit.InitUserException == true)
    {
      Slave->UserException(StartAddr, NumberOfRegister);
    }

    if(RxQue->Buf[0] != 0)
    {
      AvrUartPutChar(Slave->Uart, RxQue->Buf[0]);
      AvrUartPutChar(Slave->Uart, RxQue->Buf[1]);
      AvrUartPutChar(Slave->Uart, RxQue->Buf[2]);
      AvrUartPutChar(Slave->Uart, RxQue->Buf[3]);
      AvrUartPutChar(Slave->Uart, RxQue->Buf[4]);
      AvrUartPutChar(Slave->Uart, RxQue->Buf[5]);

      if(Slave->Bit.InitCustomFrameCheck)
      {
        Crc16 = Slave->CustomFrameCheck(TxQue, TxQue->Ctr);
      }
      else
      {
        Crc16 = Crc16Check(TxQue->OutPtr, TxQue->Buf, &TxQue->Buf[TxQue->Size - 1], TxQue->Ctr);
      }

      AvrUartPutChar(Slave->Uart, (Crc16 >> 8));
      AvrUartPutChar(Slave->Uart, (Crc16 & 0x00FF));
      AvrUartStartTx(Slave->Uart);
    }
  }
}
/*********************************************************************************/
static void SlaveReadSerialNumber(tag_AvrModbusSlaveCtrl *Slave)
{
  tag_AvrUartRingBuf *TxQue = &Slave->Uart->TxQueue;
  tag_AvrUartRingBuf *RxQue = &Slave->Uart->RxQueue;
  tU16 StartAddr, NumberOfPoint, Crc16, i;
  tU8 *BaseAddr;

  StartAddr = (tU16) (RxQue->Buf[2] << 8) + RxQue->Buf[3];
  NumberOfPoint = strlen(Slave->SerialNumberAddr) / 2;
  if(strlen(Slave->SerialNumberAddr) % 2) NumberOfPoint += 1;

  if(Slave->Bit.InitSerialNumber == false)
  {
    ErrorException(Slave, 0x0E);
  }
  else if(StartAddr != 0x0F0C)
  {
    ErrorException(Slave, 0x0F);
  }
  else
  {
    NumberOfPoint *= 2;
    BaseAddr = (tU8 *) Slave->SerialNumberAddr;
    
    AvrUartPutChar(Slave->Uart, RxQue->Buf[0]);    //Slave Address
    AvrUartPutChar(Slave->Uart, RxQue->Buf[1]);    //Function
    AvrUartPutChar(Slave->Uart, NumberOfPoint);    //Byte Count
  
    for(i = 0; i < NumberOfPoint; i++)
    {
      AvrUartPutChar(Slave->Uart, *(BaseAddr + i));
    }

    Crc16 = Crc16Check(TxQue->OutPtr, TxQue->Buf, &TxQue->Buf[TxQue->Size - 1], TxQue->Ctr);
  
    AvrUartPutChar(Slave->Uart, (Crc16 >> 8));
    AvrUartPutChar(Slave->Uart, (Crc16 & 0x00FF));
    AvrUartStartTx(Slave->Uart);
  }
}
/*********************************************************************************/
tU8 AvrModbusSlaveGeneralInit(tag_AvrModbusSlaveCtrl *Slave, tag_AvrUartCtrl *Uart, tU8 *BaseAddr, tU32 SlaveProcTick_us)
{
  /*
    1) 인수
      - Slave : tag_AvrModbusSlaveCtrl 인스턴스의 주소.
      - Uart : tag_AvrUartCtrl 인스턴스의 주소.
      - BaseAddr : Slave와 연결되는 데이터의 주소. Master의 명령을 수행할 때 응답 또는 적용의 대상이 되는 데이터.
      - SlaveProcTick_us : AvrModbusSlaveProc() 함수를 실행하는 주기.

    2) 반환
      - 0 : 초기화 실패
      - 1 : 초기화 성공

    3) 설명
      - Slave와 Uart 모듈, 데이터를 연결하는 등의 필수 초기화 실행.
      - 본 함수를 호출하기전 Uart는 선행적으로 초기화가 완료 되어 있어야 함.
  */

  if(Uart->Bit.InitComplete == true)
  {
    Slave->Uart = Uart;
    
    AvrModbusSlaveSetQueryResponseInfo(Slave, AVR_MODBUS_ReadHoldingRegister, BaseAddr, 200);
    AvrModbusSlaveSetQueryResponseInfo(Slave, AVR_MODBUS_ReadInputRegister, BaseAddr, 200);
    AvrModbusSlaveSetQueryResponseInfo(Slave, AVR_MODBUS_ReadCoilStatus, BaseAddr, 200);
    AvrModbusSlaveSetQueryResponseInfo(Slave, AVR_MODBUS_ReadInputStatus, BaseAddr, 200);

    Slave->Uart->ReceivingDelay = AVR_MODBUS_RECEIVING_DELAY_US / SlaveProcTick_us;
    if(Slave->Uart->ReceivingDelay < 2) Slave->Uart->ReceivingDelay = 2;
    AvrUartSetTxEndDelay(Uart, AVR_MODBUS_SLAVE_DEFAULT_TX_END_DELAY, SlaveProcTick_us);

    Slave->Bit.InitGeneral = true;
  }

  Slave->Bit.InitComplete = CheckAllOfSlaveInit(Slave);

  return Slave->Bit.InitGeneral;
}
/*********************************************************************************/
tU8 AvrModbusSlaveLinkCheckRangeFunc(tag_AvrModbusSlaveCtrl *Slave, tU8 (*CheckOutRange)(tU16 StartAddr, tU16 NumberOfRegister))
{
  /*
    1) 인수
      - Slave : tag_AvrModbusSlaveCtrl 인스턴스의 주소.
      - CheckOutRange : 사용자 정의 범위 확인 함수의 주소.

    2) 반환
      - 0 : 초기화 실패
      - 1 : 초기화 성공

    3) 설명
      - 사용자 정의 범위 확인 함수를 Slave에 연결.
      - 본 함수를 실행하여 범위 확인 함수를 Slave에 연결할지 여부는 필수사항이 아닌 선택 사항.
      - 범위 확인 함수는 Master가 허용하지 않은 레지스터에 접근하거나 적용을 명령할 경우 에러코드를 응답할 수 있도록 함.
        함수를 구현할 경우 정상 범위일 때 0, 비정상 범위 일 때에는 1을 응답도록 해야 함.
  */

  if(Slave->Bit.InitComplete == false)
  {
    return false;
  }

  Slave->CheckOutRange = CheckOutRange;
  Slave->Bit.InitCheckOutRange = true;

  return Slave->Bit.InitCheckOutRange;
}
/*********************************************************************************/
tU8 AvrModbusSlaveLinkUserExceptionFunc(tag_AvrModbusSlaveCtrl *Slave, void (*UserException)(tU16 StartAddr, tU16 NumberOfRegister))
{
  /*
    1) 인수
      - Slave : tag_AvrModbusSlaveCtrl 인스턴스의 주소.
      - UserException : 사용자 정의 예외처리 함수의 주소.

    2) 반환
      - 0 : 초기화 실패
      - 1 : 초기화 성공

    3) 설명
      - 사용자 정의 예외처리 함수를 Slave에 연결.
      - 본 함수를 실행하여 예외처리 함수를 Slave에 연결할지 여부는 필수사항이 아닌 선택 사항.
      - 예외처리 함수는 특정 레지스터에 대한 추가 처리를 가능토록 함. 예를 들어 시간 설정 레지스터에 대한 Master의 요청이 있을 경우
        RTC 제어 관련 함수를 호출하거나, 에러해제 시 관련 카운트 변수를 초기화 하는 등의 동작을 구현함.
  */

  if(Slave->Bit.InitComplete == false)
  {
    return false;
  }

  Slave->UserException = UserException;
  Slave->Bit.InitUserException = true;

  return Slave->Bit.InitUserException;
}
/*********************************************************************************/
tU8 AvrModbusSlaveLinkPreUserExceptionFunc(tag_AvrModbusSlaveCtrl *Slave, tU8 (*PreUserException)(struct tag_AvrModbusSlaveCtrl *Slave, tU8 *SlaveId))
{
  /*
    1) 인수
      - Slave : tag_AvrModbusSlaveCtrl 인스턴스의 주소.
      - PreUserException : 사용자 정의 사전 예외처리 함수의 주소.

    2) 반환
      - 0 : 초기화 실패
      - 1 : 초기화 성공

    3) 설명
      - 사용자 정의 예외처리 함수를 Slave에 연결.
      - 본 함수를 실행하여 예외처리 함수를 Slave에 연결할지 여부는 필수사항이 아닌 선택 사항.
      - 이 예외처리 함수는 데이터 수신 후 마스터 요청 처리 전 실행하며, 구현한 예외처리 함수에서 0이 아닌 값을 리턴할 경우
        마스터 요청을 처리 하지 않는다.
  */

  if(Slave->Bit.InitComplete == false)
  {
    return false;
  }

  Slave->PreUserException = PreUserException;
  Slave->Bit.InitPreUserException = true;

  return Slave->Bit.InitPreUserException;
}
/*********************************************************************************/
tU8 AvrModbusSlaveLinkCustomFrameCheck(tag_AvrModbusSlaveCtrl *Slave, tU16  (*CustomFrameCheck)(tag_AvrUartRingBuf *RxQue, tU16 Ctr))
{
  /*
    1) 인수
      - Slave : tag_AvrModbusSlaveCtrl 인스턴스의 주소.
      - CustomFrameCheck : 사용자 정의 프레임 에러 검출 함수 주소.

    2) 반환
      - 0 : 초기화 실패
      - 1 : 초기화 성공

    3) 설명
      - 사용자 정의 프레임 에러 검출 함수 연결.
  */
  
  if(Slave->Bit.InitComplete == false)
  {
    return false;
  }

  Slave->CustomFrameCheck = CustomFrameCheck;
  Slave->Bit.InitCustomFrameCheck = true;

  return Slave->Bit.InitCustomFrameCheck;
}
/*********************************************************************************/
tU8 AvrModbusSlaveLinkSerialNumber(tag_AvrModbusSlaveCtrl *Slave, char *SerialNumberAddr)
{
  /*
    1) 인수
      - Slave : tag_AvrModbusSlaveCtrl 인스턴스의 주소.
      - SerialNumberAddr : 프로그램명 문자열.

    2) 반환
      - 0 : 초기화 실패
      - 1 : 초기화 성공

    3) 설명
      - 본 함수를 호출하여 문자열 연결 후 AVR_MODBUS_ReadSerialNumber(0x73)으로 호출하면 프로그램명 응답.
  */
  
  if(Slave->Bit.InitComplete == false)
  {
    return false;
  }

  Slave->SerialNumberAddr = SerialNumberAddr;
  Slave->Bit.InitSerialNumber = true;

  return Slave->Bit.InitSerialNumber;
}
/*********************************************************************************/
tU8 AvrModbusSlaveSetQueryResponseInfo(tag_AvrModbusSlaveCtrl *Slave, enum_AvrModbusFunction Function, tU8 *BaseAddr, tU16 MapStartAddr)
{
  tag_QueryResponseInfo *pQueryResponseInfo;
  
  switch(Function)
  {
    case  AVR_MODBUS_PresetSingleRegister :
    case  AVR_MODBUS_PresetMultipleRegister :
    case  AVR_MODBUS_ReadHoldingRegister : pQueryResponseInfo = &Slave->QR_ReadHoldingRegister; break;
    case  AVR_MODBUS_ForceSingleCoil :
    case  AVR_MODBUS_ReadCoilStatus : pQueryResponseInfo = &Slave->QR_ReadCoilStatus; break;
    case  AVR_MODBUS_ReadInputStatus : pQueryResponseInfo = &Slave->QR_ReadInputStatus; break;
    case  AVR_MODBUS_ReadInputRegister : pQueryResponseInfo = &Slave->QR_ReadInputRegister; break;
      
    default :
    case  AVR_MODBUS_ReadSerialNumber : return false;
  }
  
  pQueryResponseInfo->BaseAddr = BaseAddr;
  pQueryResponseInfo->MapStartAddr = MapStartAddr;
  
  return true;
}
/*********************************************************************************/
void AvrModbusSlaveProc(tag_AvrModbusSlaveCtrl *Slave, tU8 SlaveId)
{
  tag_AvrUartRingBuf *RxQue = &Slave->Uart->RxQueue;
  tU16 Crc16;
  tU8 PreException = false;

  /*
    1) 인수
      - Slave : tag_AvrModbusSlaveCtrl 인스턴스의 주소.
      - SlaveId : Slave의 ID.

    2) 반환
      - 없음.

    3) 설명
      - Slave 동작을 처리함.
      - AvrModbusSlaveGeneralInit() 함수에서 설정한 'SlaveProcTick_us'과 동일한 주기로 본 함수 실행.
  */

  if(Slave->Bit.InitComplete == false)
  {
    return;
  }

  AvrUartControlTxEnd(Slave->Uart);
  
  if((AvrUartCheckRx(Slave->Uart) >= 1) && (AvrUartCheckReceiving(Slave->Uart) == false))
  {
    if(Slave->Bit.InitPreUserException && Slave->PreUserException(Slave, &SlaveId)) PreException = true;

    if(PreException == false)
    {
      if((RxQue->Buf[0] == SlaveId ) || (RxQue->Buf[0] == 0) || (RxQue->Buf[0] == 255))
      {
        if(Slave->Bit.InitCustomFrameCheck)
        {
          Crc16 = Slave->CustomFrameCheck(RxQue, RxQue->Ctr - 2);
        }
        else
        {
          Crc16 = Crc16Check(RxQue->OutPtr, RxQue->Buf, &RxQue->Buf[RxQue->Size - 1], RxQue->Ctr - 2);
        }

        if((RxQue->Buf[RxQue->Ctr - 2] == (Crc16 >> 8)) && (RxQue->Buf[RxQue->Ctr - 1] == (Crc16 & 0x00FF)))
        {
          switch(RxQue->Buf[1])
          {
            case  AVR_MODBUS_ReadCoilStatus :
              SlaveReadStatus(Slave, &Slave->QR_ReadCoilStatus);
            break;
            
            case  AVR_MODBUS_ReadInputStatus  :
              SlaveReadStatus(Slave, &Slave->QR_ReadInputStatus);
            break;
            
            case  AVR_MODBUS_ReadHoldingRegister  :
              SlaveReadRegisters(Slave, &Slave->QR_ReadHoldingRegister);
            break;
            
            case  AVR_MODBUS_ReadInputRegister  :
              SlaveReadRegisters(Slave, &Slave->QR_ReadInputRegister);
            break;
            
            case  AVR_MODBUS_ForceSingleCoil  :
              SlaveForceSingleCoil(Slave, &Slave->QR_ReadCoilStatus);
            break;

            case  AVR_MODBUS_PresetSingleRegister  :
              SlavePresetSingle(Slave, &Slave->QR_ReadHoldingRegister);
            break;

            case  AVR_MODBUS_PresetMultipleRegister  :
              SlavePresetMultiple(Slave, &Slave->QR_ReadHoldingRegister);
            break;
            
            case  AVR_MODBUS_ReadSerialNumber  :
              SlaveReadSerialNumber(Slave);
            break;

            default  :
              ErrorException(Slave, 1);
            break;
          }
        }
      }
    }
    AvrUartClearQueueBuf(RxQue);
  }
}

#endif
/*********************************************************************************/
#if(AVR_MODBUS_MASTER == true)

static tU8 CheckAllOfMasterInit(tag_AvrModbusMasterCtrl *Master)
{
  /*
    1) 인수
      - Master : tag_AvrModbusMasterCtrl 인스턴스의 주소.

    2) 반환
      - 0  : 초기화 실패.
      - 1  :  초기화 성공.

    3) 설명
      - 'tag_AvrModbusMasterCtrl' 인스턴스의 필수 항목 초기화 여부 확인.
  */

  return (Master->Bit.InitGeneral && (Master->Bit.PollDataAllocFail == false)) ? true : false;
}
/*********************************************************************************/
static tag_AvrModbusMasterSlaveInfo* GetAddedSlaveInfo(tag_AvrModbusMasterCtrl *Master, tag_AvrModbusMasterSlaveInfo *Slave)
{
  tU8 i = 0;
  tag_AvrModbusMasterSlaveInfo *SlaveTemp = Slave;

  /*
    1) 인수
      - Master : tag_AvrModbusMasterCtrl 인스턴스의 주소.
      - Slave : tag_AvrModbusMasterSlaveInfo 인스턴스의 주소.

    2) 반환
      - 인수로 받은 Slave와 인접한 다음 Slave의 주소.

    3) 설명
      - 본 함수는 Master에 추가 되어 있는 Slave를 순회하기 위해 구현함.
      - 인수로 받은 Slave에 다음에 위치한 Slave 주소를 반환함. 예를 들어 총 5개의 Slave가 추가 되어 있을 때 인수로 받은 Slave가
        3번째라면 4번째 Slave의 주소를 반환함.
  */

  do
  {
    if(Slave == &Master->SlaveArray[Master->MaxSlave - 1])
    {
      Slave = Master->SlaveArray;
    }
    else
    {
      Slave++;
    }

    if(++i > Master->MaxSlave)
    {
      Slave = SlaveTemp;
      break;
    }
  }while(Slave->Id == 0);

  return Slave;
}
/*********************************************************************************/
static void MasterPolling(tag_AvrModbusMasterCtrl *Master)
{
  tag_AvrUartRingBuf *TxQue = &Master->Uart->TxQueue;
  tag_AvrModbusMasterSlavePollData *PollData;
  tU16 Crc16;

  /*
    1) 인수
      - Master : tag_AvrModbusMasterCtrl 인스턴스의 주소.

    2) 반환
      - 없음.

    3) 설명
      - Slave Polling을 수행함.
  */

  if(++Master->SlavePoll->PollDataIndex >= Master->SlavePoll->PollDataMax)
  {
    Master->SlavePoll->PollDataIndex = 0;
    Master->SlavePoll = GetAddedSlaveInfo(Master, Master->SlavePoll);
  }

  if(Master->SlavePoll->NoResponseCnt < Master->SlavePoll->NoResponseLimit)
  {
    Master->SlavePoll->NoResponseCnt++;
  }

  Master->IsPollingSend = true;
  PollData = &Master->SlavePoll->PollData[Master->SlavePoll->PollDataIndex];

  AvrUartPutChar(Master->Uart, Master->SlavePoll->Id);
  AvrUartPutChar(Master->Uart, PollData->PollFunction);
  AvrUartPutChar(Master->Uart, (PollData->StartAddr >> 8));
  AvrUartPutChar(Master->Uart, (PollData->StartAddr & 0x00FF));
  AvrUartPutChar(Master->Uart, (PollData->NumberOfRegister >> 8));
  AvrUartPutChar(Master->Uart, (PollData->NumberOfRegister & 0x00FF));

  Crc16 = Crc16Check(TxQue->OutPtr, TxQue->Buf, &TxQue->Buf[TxQue->Size - 1], TxQue->Ctr);

  AvrUartPutChar(Master->Uart, (Crc16 >> 8));
  AvrUartPutChar(Master->Uart, (Crc16 & 0x00FF));
  AvrUartStartTx(Master->Uart);
}
/*********************************************************************************/
static void MasterReceiveRegister(tag_AvrModbusMasterSlavePollData *PollData, tag_AvrUartRingBuf *RxQue)
{
  tU16 Length, i, j = 3;
  
  Length = PollData->NumberOfRegister * 2;
  for(i = 0; i < Length; i += 2)
  {
    *(PollData->BaseAddr + i + 1) = RxQue->Buf[j++];
    *(PollData->BaseAddr + i) = RxQue->Buf[j++];
  }
}
/*********************************************************************************/
static void MasterReceiveStatus(tag_AvrModbusMasterSlavePollData *PollData, tag_AvrUartRingBuf *RxQue)
{
  tU8 ByteCount;
  
  ByteCount = RxQue->Buf[2];
  if(ByteCount > (RxQue->Size - 4)) ByteCount = RxQue->Size - 4;
  memcpy(PollData->BaseAddr, &RxQue->Buf[3], ByteCount);
}
/*********************************************************************************/
static tU16 GetExpectRxCnt(tag_AvrModbusMasterSlavePollData *PollData)
{
  const tU8 HeaderSize = 5; //slave address / function / byte count / crc / crc
  tU16 Cnt;
  
  switch(PollData->PollFunction)
  {
    case  AVR_MODBUS_ReadInputStatus  :
    case  AVR_MODBUS_ReadCoilStatus  :  
      Cnt = PollData->NumberOfRegister / 8;
      if(PollData->NumberOfRegister % 8) Cnt++;
      Cnt += HeaderSize;
    break;
    
    default  :
    case  AVR_MODBUS_ReadInputRegister  :
    case  AVR_MODBUS_ReadHoldingRegister  : 
      Cnt = (PollData->NumberOfRegister * 2) + HeaderSize;
    break;
  }
  
  return Cnt;
}
/*********************************************************************************/
static void MasterReceive(tag_AvrModbusMasterCtrl *Master)
{
  tU16 Crc16;
  tag_AvrUartRingBuf *RxQue = &Master->Uart->RxQueue;
  tag_AvrModbusMasterSlavePollData *PollData;
  tag_AvrModbusMasterSlaveInfo *Slave;

  /*
    1) 인수
      - Master : tag_AvrModbusMasterCtrl 인스턴스의 주소.

    2) 반환
      - 없음.

    3) 설명
      - Slave의 응답을 처리함.
  */

  Slave = Master->SlavePoll;
  PollData = &Slave->PollData[Slave->PollDataIndex];

  if((Slave != null) && (Master->IsPollingSend == true) && (RxQue->Buf[1] == PollData->PollFunction) && (GetExpectRxCnt(PollData) == RxQue->Ctr))
  {
    Crc16 = Crc16Check(RxQue->OutPtr, RxQue->Buf, &RxQue->Buf[RxQue->Size - 1], RxQue->Ctr - 2);

    if((RxQue->Buf[RxQue->Ctr - 2] == (Crc16 >> 8)) && (RxQue->Buf[RxQue->Ctr - 1] == (Crc16 & 0x00FF)))
    {
      switch(PollData->PollFunction)
      {
        case  AVR_MODBUS_ReadInputStatus  :
        case  AVR_MODBUS_ReadCoilStatus  :  MasterReceiveStatus(PollData, RxQue); break;
        
        default  :
        case  AVR_MODBUS_ReadInputRegister  :  
        case  AVR_MODBUS_ReadHoldingRegister  :  MasterReceiveRegister(PollData, RxQue); break;
      }
      
      Slave->NoResponseCnt = 0;
      Master->IsPollingSend = false;
      if(Master->Bit.InitRxUserException == true)
      {
        Master->UserException(Slave->Id);
      }
    }
  }
}
/*********************************************************************************/
tU8 AvrModbusMasterGeneralInit(tag_AvrModbusMasterCtrl *Master, tag_AvrUartCtrl *Uart, tU8 MaxSlave, tU32 MasterProcTick_us)
{
  tU8 i;
  /*
    1) 인수
      - Master : tag_AvrModbusMasterCtrl 인스턴스의 주소.
      - Uart : tag_AvrUartCtrl 인스턴스의 주소.
      - MaxSlave : 추가할 수 있는 최대 Slave의 수.
      - MasterProcTick_us : AvrModbusMasterProc() 함수를 실행하는 주기.

    2) 반환
      - 0 : 초기화 실패
      - 1 : 초기화 성공

    3) 설명
      - Master와 관련된 필수 정보들을 초기화 함.
      - 본 함수를 호출하기전 Uart는 선행적으로 초기화가 완료 되어 있어야 함.
  */
  
  if((Uart == null) || (Uart->Bit.InitComplete != true))
  {
    return false;
  }
  //Uart 초기화 여부 검사

  Master->SlaveArray = (tag_AvrModbusMasterSlaveInfo *) calloc(MaxSlave, sizeof(tag_AvrModbusMasterSlaveInfo));
  if(Master->SlaveArray == null)
  {
    return false;
  }
  //tag_AvrModbusMasterSlaveInfo 동적할당
  
  for(i = 0; i < MaxSlave; i++)
  {
    Master->SlaveArray[i].PollData = (tag_AvrModbusMasterSlavePollData *) calloc(1, sizeof(tag_AvrModbusMasterSlavePollData));
    if(Master->SlaveArray[i].PollData == null)
    {
      return false;
    }
  }
  //tag_AvrModbusMasterSlavePollData 동적 할당
  
  Master->IsWriteCmdPending = false;
  Master->Uart = Uart;
  Master->MaxSlave = MaxSlave;
  Master->Tick_us = MasterProcTick_us;
  Master->Uart->ReceivingDelay = AVR_MODBUS_RECEIVING_DELAY_US / Master->Tick_us;
  Master->PollDelay = AVR_MODBUS_DEFAULT_POLLING_DELAY_US / Master->Tick_us;
  if(Master->Uart->ReceivingDelay < 2) Master->Uart->ReceivingDelay = 2;

  Master->Bit.InitGeneral = true;
  Master->Bit.InitComplete = CheckAllOfMasterInit(Master);

  return Master->Bit.InitGeneral;
}
/*********************************************************************************/
tU8 AvrModbusMasterSetPollingDelay(tag_AvrModbusMasterCtrl *Master, tU32 PollDelay_us)
{
  /*
    1) 인수
      - Master : tag_AvrModbusMasterCtrl 인스턴스의 주소.
      - PollDelay_us : Master가 Slave를 Polling할 주기.

    2) 반환
      - 0 : 초기화 실패
      - 1 : 초기화 성공

    3) 설명
      - Master의 Slave Polling 주기 설정.
  */

  if(Master->Bit.InitComplete == false)
  {
    return false;
  }

  Master->PollDelay = PollDelay_us / Master->Tick_us;
  return true;
}
/*********************************************************************************/
tU8 AvrModbusMasterAddSlave(tag_AvrModbusMasterCtrl *Master, tU8 Id, enum_AvrModbusFunction PollFunction, tU16 StartAddr, tU16 NumberOfRegister, tU8 *BaseAddr)
{
  tU8 i;
  tag_AvrModbusMasterSlaveInfo *Slave = Master->SlaveArray;

  /*
    1) 인수
      - Master : tag_AvrModbusMasterCtrl 인스턴스의 주소.
      - Id : 추가할 Slave의 ID
      - StartAddr : 추가할 Slave의 StartAddr
      - NumberOfRegister : 추가할 Slave의 레지스터 갯수.
      - BaseAddr : 추가할 Slave와 연결할 Data의 주소. Slave가 ReadHolding에 대한 요청을 응답하면 해당 데이터를 BaseAddr에 대입.

    2) 반환
      - 0 : 초기화 실패
      - 1 : 초기화 성공

    3) 설명
      - Master가 관리할 Slave 추가.
  */

  if((Master->Bit.InitComplete == false) || (Master->AddedSlave >= Master->MaxSlave))
  {
    return false;
  }

  for(i = 0; i < Master->MaxSlave; i++)
  {
    if(Slave->Id == Id)
    {
      return false;
    }
    Slave = GetAddedSlaveInfo(Master, Slave);
  }

  for(i = 0; i < Master->MaxSlave; i++)
  {
    if(Master->SlaveArray[i].Id == 0)
    {
      Master->SlaveArray[i].Id = Id;
      Master->SlaveArray[i].NoResponseCnt = 0;
      Master->SlaveArray[i].NoResponseLimit = AVR_MODBUS_DEFAULT_SLAVE_NO_RESPONSE;
      Master->SlaveArray[i].PollDataMax = 1;
      Master->SlaveArray[i].PollData[0].PollFunction = PollFunction;
      Master->SlaveArray[i].PollData[0].StartAddr = StartAddr;
      Master->SlaveArray[i].PollData[0].NumberOfRegister = NumberOfRegister;
      Master->SlaveArray[i].PollData[0].BaseAddr = BaseAddr;
      Master->SlavePoll = &Master->SlaveArray[i];
      Master->AddedSlave++;
      return true;
    }
  }

  Master->Bit.InitComplete = CheckAllOfMasterInit(Master);
  return false;
}
/*********************************************************************************/
tU8 AvrModbusMasterAddSlavePollData(tag_AvrModbusMasterCtrl *Master, tU8 Id, enum_AvrModbusFunction PollFunction, tU16 StartAddr, tU16 NumberOfRegister, tU8 *BaseAddr)
{
  tU8 i;
  tag_AvrModbusMasterSlaveInfo *Slave = Master->SlaveArray;
  
  /*
    1) 인수
      - Master : tag_AvrModbusMasterCtrl 인스턴스의 주소.
      - Id : 추가할 Slave의 ID
      - StartAddr : 추가할 PollData의 StartAddr
      - NumberOfRegister : 추가할 PollData의 레지스터 갯수.
      - BaseAddr : 추가할 PollData와 연결할 Data의 주소. Slave가 ReadHolding에 대한 요청을 응답하면 해당 데이터를 BaseAddr에 대입.

    2) 반환
      - 0 : 추가 실패
      - 1 : 추가 성공

    3) 설명
      - 이미 추가 되어 있는 Slave에 PollData 추가.
      - PollData는 슬레이브 데이터를 비선형적으로 호출할 필요가 있을 때 추가. 예를 들어 200~210번지, 430~450번지와 같이
        필요한 데이터의 주소가 인접해 있지 않은 경우 PollData를 추가하여 순차 호출.
      - 추가 하려는 StartAddr가 이미 추가 되어 있는 값과 같다면 중복이므로 처리 하지 않음.
      - recommend.
        Slave1 Add.
        Slave1 PollData Add.
        Slave2 Add.
        Slave2 PollData Add.
      - not recommend.
        Slave1 Add.
        Slave2 Add.
        Slave1 PollData Add.
        Slave2 PollData Add.
  */

  if((Master->Bit.InitComplete == false) || (Master->AddedSlave == 0))
  {
    return false;
  }

  Slave = AvrModbusMasterFindSlaveById(Master, Id);
  if((Slave == null) || (Slave->PollDataMax == 0))
  {
    return false;
  }
  
  for(i = 0; i < Slave->PollDataMax; i++)
  {
    if((Slave->PollData[i].StartAddr == StartAddr) && (Slave->PollData[i].PollFunction == PollFunction)) return false;
  }
  
  Slave->PollData = (tag_AvrModbusMasterSlavePollData *) realloc(Slave->PollData, (Slave->PollDataMax + 1) * sizeof(tag_AvrModbusMasterSlavePollData));
  if(Slave->PollData == null)
  {
    Master->Bit.PollDataAllocFail = true;
    Master->Bit.InitComplete = CheckAllOfMasterInit(Master);
    return false;
  }
  
  Slave->PollDataMax++;
  Slave->PollData[Slave->PollDataMax - 1].StartAddr = StartAddr;
  Slave->PollData[Slave->PollDataMax - 1].NumberOfRegister = NumberOfRegister;
  Slave->PollData[Slave->PollDataMax - 1].BaseAddr = BaseAddr;
  Slave->PollData[Slave->PollDataMax - 1].PollFunction = PollFunction;
  
  return true;
}
/*********************************************************************************/
void AvrModbusMasterRemoveSlave(tag_AvrModbusMasterCtrl *Master, tU8 Id)
{
  tag_AvrModbusMasterSlaveInfo *Slave;
  tag_AvrModbusMasterSlavePollData *BackupPollData;

  /*
    1) 인수
      - Master : tag_AvrModbusMasterCtrl 인스턴스의 주소.
      - Id : 삭제할 Slave의 ID

    2) 반환
      - 없음.

    3) 설명
      - 인수로 받은 ID와 동일한 Slave를 검색하여 일치하는 Slave 삭제.
  */

  if((Master->Bit.InitComplete == false) || (Master->AddedSlave == 0))
  {
    return;
  }

  Slave = AvrModbusMasterFindSlaveById(Master, Id);

  if(Slave != null)
  {
    BackupPollData = (tag_AvrModbusMasterSlavePollData *) realloc(Slave->PollData, sizeof(tag_AvrModbusMasterSlavePollData));
    memset(Slave, 0, sizeof(tag_AvrModbusMasterSlaveInfo));
    Slave->PollData = BackupPollData;
    Master->AddedSlave--;
  }
}
/*********************************************************************************/
void AvrModbusMasterSetSlaveNoResponse(tag_AvrModbusMasterCtrl *Master, tU8 Id, tU8 NoResponseLimit)
{
  tag_AvrModbusMasterSlaveInfo *Slave = null;

  /*
    1) 인수
      - Master : tag_AvrModbusMasterCtrl 인스턴스의 주소.
      - Id : Slave의 ID.
      - NoResponseLimit : 무응답 횟수.

    2) 반환
      - 없음.

    3) 설명
      - 인수로 받은 ID와 동일한 Slave를 검색하여 일치하는 Slave의 무응답 횟수 설정.
  */

  if((Master->Bit.InitComplete == false) || (Master->AddedSlave == 0))
  {
    return;
  }

  Slave = AvrModbusMasterFindSlaveById(Master, Id);

  if(Slave != null)
  {
    Slave->NoResponseLimit = NoResponseLimit;
    Slave->NoResponseCnt = 0;
  }
}
/*********************************************************************************/
tU8 AvrModbusMasterLinkUserException(tag_AvrModbusMasterCtrl *Master, void (*UserException)(tU8 Id))
{
  /*
    1) 인수
      - Master : tag_AvrModbusMasterCtrl 인스턴스의 주소.
      - UserException : 사용자 정의 함수의 주소.

    2) 반환
      - 0 : 연결 실패
      - 1 : 연결 성공

    3) 설명
      - 사용자 정의 함수 연결.
      - Slave가 Master 호출에 정상적으로 응답했을 때 데이터 수신 처리 후 본 함수 호출.
  */

  if(Master->Bit.InitComplete == false)
  {
    return false;
  }

  Master->UserException = UserException;
  Master->Bit.InitRxUserException = true;

  return Master->Bit.InitRxUserException;
}
/*********************************************************************************/
void AvrModbusMasterProc(tag_AvrModbusMasterCtrl *Master)
{
  /*
    1) 인수
      - Master : tag_AvrModbusMasterCtrl 인스턴스의 주소.

    2) 반환
      - 없음.

    3) 설명
      - Master 동작 처리.
  */

  if((Master->Bit.InitComplete == false) || (Master->AddedSlave == 0))
  {
    return;
  }

  if((AvrUartCheckRx(Master->Uart) >= 1) && (AvrUartCheckReceiving(Master->Uart) == false))
  {
    MasterReceive(Master);
    AvrUartClearQueueBuf(&Master->Uart->RxQueue);
  }
  else
  {
    if(Master->PollCnt)
    {
      Master->PollCnt--;
    }
    else
    {
      Master->PollCnt = Master->PollDelay;
      if(Master->IsWriteCmdPending)
      {
        Master->IsWriteCmdPending = false;
        AvrUartStartTx(Master->Uart);
      }
      else
      {
        MasterPolling(Master);
      }
    }
  }
}
/*********************************************************************************/
tU8 AvrModbusMasterWriteSingle(tag_AvrModbusMasterCtrl *Master, tU8 SlaveId, enum_AvrModbusFunction Func, tU16 RegAddr, tU16 PresetData)
{
  tU16 Crc16;
  tag_AvrUartRingBuf *TxQue = &Master->Uart->TxQueue;

  /*
    1) 인수
      - Master : tag_AvrModbusMasterCtrl 인스턴스의 주소.
      - SlaveId : 명령을 전달할 Slave의 ID
      - Func  :명령 펑션타입
      - RegAddr : write 레지스터의 주소.
      - PresetData : write 데이터.

    2) 반환
      - 0 : 요청 실패
      - 1 : 요청 성공

    3) 설명
      - 인수로 받은 SlaveId에 write single 명령을 보냄.
      - 현재 이 함수는 AVR_MODBUS_ForceSingleCoil, AVR_MODBUS_PresetSingleRegister 펑션만 지원한다.
  */

  if((Master->Bit.InitComplete == false) || (Master->AddedSlave == 0) || (Master->IsWriteCmdPending == true) || ((Func != AVR_MODBUS_ForceSingleCoil) && (Func != AVR_MODBUS_PresetSingleRegister)))
  {
    return false;
  }
  
  Master->IsWriteCmdPending = true;
  if(Func == AVR_MODBUS_ForceSingleCoil){ PresetData = PresetData ? 0xFF00 : 0x0000; }

  AvrUartPutChar(Master->Uart, SlaveId);
  AvrUartPutChar(Master->Uart, Func);
  AvrUartPutChar(Master->Uart, RegAddr >> 8);
  AvrUartPutChar(Master->Uart, RegAddr & 0x00FF);
  AvrUartPutChar(Master->Uart, PresetData >> 8);
  AvrUartPutChar(Master->Uart, PresetData & 0x00FF);

  Crc16 = Crc16Check(TxQue->OutPtr, TxQue->Buf, &TxQue->Buf[TxQue->Size - 1], TxQue->Ctr);

  AvrUartPutChar(Master->Uart, (Crc16 >> 8));
  AvrUartPutChar(Master->Uart, (Crc16 & 0x00FF));

  return true;
}
/*********************************************************************************/
tU8 AvrModbusMasterPresetSingle(tag_AvrModbusMasterCtrl *Master, tU8 SlaveId, tU16 RegAddr, tU16 PresetData)
{
  /*
    1) 인수
      - Master : tag_AvrModbusMasterCtrl 인스턴스의 주소.
      - SlaveId : 명령을 전달할 Slave의 ID
      - RegAddr : write 레지스터의 주소.
      - PresetData : write 데이터.

    2) 반환
      - 0 : 요청 실패
      - 1 : 요청 성공

    3) 설명
      - 인수로 받은 SlaveId에 PresetSingle 명령을 보냄.
  */
  
  return AvrModbusMasterWriteSingle(Master, SlaveId, AVR_MODBUS_PresetSingleRegister, RegAddr, PresetData);
}
/*********************************************************************************/
tU8 AvrModbusMasterPresetMultiple(tag_AvrModbusMasterCtrl *Master, tU8 SlaveId, tU16 StartAddr, tU16 NumberOfRegister, tU8 *BaseAddr)
{
  tU16 Crc16, i;
  tag_AvrUartRingBuf *TxQue = &Master->Uart->TxQueue;

  /*
    1) 인수
      - Master : tag_AvrModbusMasterCtrl 인스턴스의 주소.
      - SlaveId : 명령을 전달할 Slave의 ID
      - StartAddr : 시작 주소.
      - NumberOfRegister : 레지스터의 갯수.
      - BaseAddr : PresetMultiple 요청할 데이터 버퍼의 주소.

    2) 반환
      - 0 : 요청 실패
      - 1 : 요청 성공

    3) 설명
      - 인수로 받은 SlaveId에 PresetMultiple 명령을 보냄.
  */

  if((Master->Bit.InitComplete == false) || (Master->AddedSlave == 0) || (Master->IsWriteCmdPending == true))
  {
    return false;
  }
  
  Master->IsWriteCmdPending = true;

  AvrUartPutChar(Master->Uart, SlaveId);
  AvrUartPutChar(Master->Uart, AVR_MODBUS_PresetMultipleRegister);
  AvrUartPutChar(Master->Uart, StartAddr >> 8);
  AvrUartPutChar(Master->Uart, StartAddr & 0x00FF);
  AvrUartPutChar(Master->Uart, NumberOfRegister >> 8);
  AvrUartPutChar(Master->Uart, NumberOfRegister & 0x00FF);

  NumberOfRegister *= 2;
  AvrUartPutChar(Master->Uart, NumberOfRegister);

  for(i = 0; i < NumberOfRegister; i += 2)
  {
    AvrUartPutChar(Master->Uart, *(BaseAddr + i + 1));
    AvrUartPutChar(Master->Uart, *(BaseAddr + i));
  }

  Crc16 = Crc16Check(TxQue->OutPtr, TxQue->Buf, &TxQue->Buf[TxQue->Size - 1], TxQue->Ctr);

  AvrUartPutChar(Master->Uart, (Crc16 >> 8));
  AvrUartPutChar(Master->Uart, (Crc16 & 0x00FF));

  return true;
}
/*********************************************************************************/
tU8 AvrModbusMasterCheckSlaveNoResponse(tag_AvrModbusMasterCtrl *Master, tU8 Id)
{
  tag_AvrModbusMasterSlaveInfo *Slave;

  /*
    1) 인수
      - Master : tag_AvrModbusMasterCtrl 인스턴스의 주소.
      - Id : 무응답 여부를 확인할 Slave의 ID

    2) 반환
      - 0 : Slave 응답 정상
      - 1 : Slave 무응답

    3) 설명
      - 인수로 받은 Id와 동일한 Slave 응답 여부.
  */

  if((Master->Bit.InitComplete == false) || (Master->AddedSlave == 0))
  {
    return false;
  }

  Slave = AvrModbusMasterFindSlaveById(Master, Id);

  if((Slave != null) && (Slave->NoResponseCnt >= Slave->NoResponseLimit))
  {
    return true;
  }
  else
  {
    return false;
  }
}
/*********************************************************************************/
tag_AvrModbusMasterSlaveInfo* AvrModbusMasterFindSlaveById(tag_AvrModbusMasterCtrl *Master, tU8 Id)
{
  tU8 i, Find = false;
  tag_AvrModbusMasterSlaveInfo *Slave = Master->SlavePoll;

  /*
    1) 인수
      - Master : tag_AvrModbusMasterCtrl 인스턴스의 주소.
      - Id : 찾고자 하는 Slave의 ID

    2) 반환
      - 인수로 받은 Id에 해당하는 Slave의 주소.

    3) 설명
      - 추가 되어 있는 Slave 중 인수로 받은 ID와 동일한 Slave의 주소를 찾아 반환함.
  */

  if((Master->Bit.InitComplete == false) || (Master->AddedSlave == 0))
  {
    return null;
  }

  for(i = 0; i < Master->MaxSlave; i++)
  {
    if(Slave->Id == Id)
    {
      Find = true;
      break;
    }
    Slave = GetAddedSlaveInfo(Master, Slave);
  }

  return Find ? Slave : null;
}
/*********************************************************************************/
#endif
