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
#if(AVR_MODBUS_REVISION_DATE != 20180906)
#error wrong include file. (AvrModbus.h)
#endif
/*********************************************************************************/
/** Global variable **/


/*********************************************************************************/
#if(AVR_MODBUS_SLAVE == true)

static char CheckAllOfSlaveInit(tag_AvrModbusSlaveCtrl *Slave)
{
	/*
		1) 인수
			- Slave : tag_AvrModbusSlaveCtrl 인스턴스의 주소.

		2) 반환
			- 0	: 초기화 실패.
			- 1	:	초기화 성공.

		3) 설명
			- 'tag_AvrModbusSlaveCtrl' 인스턴스의 필수 항목 초기화 여부 확인.
	*/

	return (Slave->Bit.InitGeneral) ? true : false;
}
/*********************************************************************************/
static void ErrorException(tag_AvrModbusSlaveCtrl *Slave, char ErrCode)
{
	tag_AvrUartRingBuf *TxQue = &Slave->Uart->TxQueue;
	tag_AvrUartRingBuf *RxQue = &Slave->Uart->RxQueue;
	unsigned int Crc16;

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
static void SlaveReadHolding(tag_AvrModbusSlaveCtrl *Slave)
{
	tag_AvrUartRingBuf *TxQue = &Slave->Uart->TxQueue;
	tag_AvrUartRingBuf *RxQue = &Slave->Uart->RxQueue;
	unsigned int StartAddr, NumberOfPoint, Crc16, i;
	char *BaseAddr;

	/*
		1) 인수
			- Slave : tag_AvrModbusSlaveCtrl 인스턴스의 주소.

		2) 반환
			- 없음.

		3) 설명
			- Master의 'ReadHolding' 명령에 대한 처리.
	*/

	StartAddr = (int) (RxQue->Buf[2] << 8) + RxQue->Buf[3];
	NumberOfPoint = (int) (RxQue->Buf[4] << 8) + RxQue->Buf[5];

	StartAddr = (StartAddr < 200) ? 0 : StartAddr - 200;
	NumberOfPoint *= 2;
	BaseAddr = (char *) (((int *) Slave->BaseAddr) + StartAddr);

	AvrUartPutChar(Slave->Uart, RxQue->Buf[0]);		//Slave Address
	AvrUartPutChar(Slave->Uart, RxQue->Buf[1]);		//Function
	AvrUartPutChar(Slave->Uart, NumberOfPoint);		//Byte Count

	for(i = 0; i < NumberOfPoint; i += 2)
	{
		AvrUartPutChar(Slave->Uart, *(BaseAddr + i + 1));
		AvrUartPutChar(Slave->Uart, *(BaseAddr + i));
	}

	Crc16 = Crc16Check(TxQue->OutPtr, TxQue->Buf, &TxQue->Buf[TxQue->Size - 1], TxQue->Ctr);

	AvrUartPutChar(Slave->Uart, (Crc16 >> 8));
	AvrUartPutChar(Slave->Uart, (Crc16 & 0x00FF));
	AvrUartStartTx(Slave->Uart);
}
/*********************************************************************************/
static void SlavePresetSingle(tag_AvrModbusSlaveCtrl *Slave)
{
	tag_AvrUartRingBuf *TxQue = &Slave->Uart->TxQueue;
	tag_AvrUartRingBuf *RxQue = &Slave->Uart->RxQueue;
	int RegisterAddr, PresetData, *BaseAddr;
	unsigned int Crc16;

	/*
		1) 인수
			- Slave : tag_AvrModbusSlaveCtrl 인스턴스의 주소.

		2) 반환
			- 없음.

		3) 설명
			- Master의 'PresetSingle' 명령에 대한 처리.
	*/

	RegisterAddr = (int) (RxQue->Buf[2] << 8) + RxQue->Buf[3];
	PresetData = (int) (RxQue->Buf[4] << 8) + RxQue->Buf[5];

	if((Slave->Bit.InitCheckOutRange == true) && (Slave->CheckOutRange(RegisterAddr, 1) == true))
	{
		ErrorException(Slave, 2);
	}
	else
	{
		BaseAddr = ((int *) Slave->BaseAddr) + ((RegisterAddr < 200) ? 0 : RegisterAddr - 200);
		*BaseAddr = PresetData;

		if(Slave->Bit.InitUserException == true)
		{
			Slave->UserException(RegisterAddr, 1);
		}

		if(RxQue->Buf[0] != 0)
		{
			AvrUartPutChar(Slave->Uart, RxQue->Buf[0]);		//Slave Address
			AvrUartPutChar(Slave->Uart, RxQue->Buf[1]);		//Function
			AvrUartPutChar(Slave->Uart, RxQue->Buf[2]);		//Register Address Hi
			AvrUartPutChar(Slave->Uart, RxQue->Buf[3]);		//Register Address Lo
			AvrUartPutChar(Slave->Uart, RxQue->Buf[4]);		//Preset Data Hi
			AvrUartPutChar(Slave->Uart, RxQue->Buf[5]);		//Preset Data Lo

			Crc16 = Crc16Check(TxQue->OutPtr, TxQue->Buf, &TxQue->Buf[TxQue->Size - 1], TxQue->Ctr);

			AvrUartPutChar(Slave->Uart, (Crc16 >> 8));
			AvrUartPutChar(Slave->Uart, (Crc16 & 0x00FF));
			AvrUartStartTx(Slave->Uart);
		}
	}
}
/*********************************************************************************/
static void SlavePresetMultiple(tag_AvrModbusSlaveCtrl *Slave)
{
	tag_AvrUartRingBuf *TxQue = &Slave->Uart->TxQueue;
	tag_AvrUartRingBuf *RxQue = &Slave->Uart->RxQueue;
	unsigned int StartAddr, NumberOfRegister, Crc16, Length, i, j = 7;
	char *BaseAddr;

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

	if((Slave->Bit.InitCheckOutRange == true) && (Slave->CheckOutRange(StartAddr, NumberOfRegister) == true))
	{
		ErrorException(Slave, 2);
	}
	else
	{
		Length = NumberOfRegister * 2;
		Length = (Length > (Slave->Uart->RxQueue.Size - 9)) ? (Slave->Uart->RxQueue.Size - 9) : Length;
		BaseAddr = (char *) (((int *) Slave->BaseAddr) + ((StartAddr < 200) ? 0 : StartAddr - 200));

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

			Crc16 = Crc16Check(TxQue->OutPtr, TxQue->Buf, &TxQue->Buf[TxQue->Size - 1], TxQue->Ctr);

			AvrUartPutChar(Slave->Uart, (Crc16 >> 8));
			AvrUartPutChar(Slave->Uart, (Crc16 & 0x00FF));
			AvrUartStartTx(Slave->Uart);
		}
	}
}
/*********************************************************************************/
char AvrModbusSlaveGeneralInit(tag_AvrModbusSlaveCtrl *Slave, tag_AvrUartCtrl *Uart, char *BaseAddr, long SlaveProcTick_us)
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
		Slave->BaseAddr = BaseAddr;
		Slave->Uart->ReceivingDelay = AVR_MODBUS_RECEIVING_DELAY_US / SlaveProcTick_us;
		if(Slave->Uart->ReceivingDelay < 2) Slave->Uart->ReceivingDelay = 2;

		Slave->Bit.InitGeneral = true;
	}

	Slave->Bit.InitComplete = CheckAllOfSlaveInit(Slave);

	return Slave->Bit.InitGeneral;
}
/*********************************************************************************/
char AvrModbusSlaveLinkCheckRangeFunc(tag_AvrModbusSlaveCtrl *Slave, char (*CheckOutRange)(int StartAddr, int NumberOfRegister))
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
char AvrModbusSlaveLinkUserExceptionFunc(tag_AvrModbusSlaveCtrl *Slave, void (*UserException)(int StartAddr, int NumberOfRegister))
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
void AvrModbusSlaveProc(tag_AvrModbusSlaveCtrl *Slave, unsigned char SlaveAddr)
{
	tag_AvrUartRingBuf *RxQue = &Slave->Uart->RxQueue;
	unsigned int Crc16;

	/*
		1) 인수
			- Slave : tag_AvrModbusSlaveCtrl 인스턴스의 주소.
			- SlaveAddr : Slave의 ID.

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

	AvrUartFixTxEnableFloating(Slave->Uart);

	if((AvrUartCheckRx(Slave->Uart) >= 1) && (AvrUartCheckReceiving(Slave->Uart) == false))
	{
		if((RxQue->Buf[0] == SlaveAddr ) || (RxQue->Buf[0] == 0) || (RxQue->Buf[0] == 255))
		{
			Crc16 = Crc16Check(RxQue->OutPtr, RxQue->Buf, &RxQue->Buf[RxQue->Size - 1], RxQue->Ctr - 2);

			if((RxQue->Buf[RxQue->Ctr - 2] == (Crc16 >> 8)) && (RxQue->Buf[RxQue->Ctr - 1] == (Crc16 & 0x00FF)))
			{
				switch(RxQue->Buf[1])
				{
					case	AVR_MODBUS_ReadHolding	:
						SlaveReadHolding(Slave);
					break;

					case	AVR_MODBUS_PresetSingle	:
						SlavePresetSingle(Slave);
					break;

					case	AVR_MODBUS_PresetMultiple	:
						SlavePresetMultiple(Slave);
					break;

					default	:
						ErrorException(Slave, 1);
					break;
				}
			}
			else
			{
				ErrorException(Slave, 3);
			}
		}
		AvrUartClearQueueBuf(&Slave->Uart->RxQueue);
	}
}

#endif
/*********************************************************************************/
#if(AVR_MODBUS_MASTER == true)

static char CheckAllOfMasterInit(tag_AvrModbusMasterCtrl *Master)
{
	/*
		1) 인수
			- Master : tag_AvrModbusMasterCtrl 인스턴스의 주소.

		2) 반환
			- 0	: 초기화 실패.
			- 1	:	초기화 성공.

		3) 설명
			- 'tag_AvrModbusMasterCtrl' 인스턴스의 필수 항목 초기화 여부 확인.
	*/

	return (Master->Bit.InitGeneral) ? true : false;
}
/*********************************************************************************/
static tag_AvrModbusMasterSlaveInfo* GetAddedSlaveInfo(tag_AvrModbusMasterCtrl *Master, tag_AvrModbusMasterSlaveInfo *Slave)
{
	char i = 0;
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
static tag_AvrModbusMasterSlaveInfo* FindSlaveById(tag_AvrModbusMasterCtrl *Master, unsigned char Id)
{
	unsigned char i, Find = false;
	tag_AvrModbusMasterSlaveInfo *Slave = Master->SlaveArray;

	/*
		1) 인수
			- Master : tag_AvrModbusMasterCtrl 인스턴스의 주소.
			- Id : 찾고자 하는 Slave의 ID

		2) 반환
			- 인수로 받은 Id에 해당하는 Slave의 주소.

		3) 설명
			- 추가 되어 있는 Slave 중 인수로 받은 ID와 동일한 Slave의 주소를 찾아 반환함.
	*/

	for(i = 0; i < Master->AddedSlave; i++)
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
static void MasterPolling(tag_AvrModbusMasterCtrl *Master)
{
	tag_AvrUartRingBuf *TxQue = &Master->Uart->TxQueue;
	unsigned int Crc16;

	/*
		1) 인수
			- Master : tag_AvrModbusMasterCtrl 인스턴스의 주소.

		2) 반환
			- 없음.

		3) 설명
			- Slave Polling을 수행함.
	*/

	Master->SlavePoll = GetAddedSlaveInfo(Master, Master->SlavePoll);

	if(Master->SlavePoll->NoResponseCnt < Master->SlavePoll->NoResponseLimit)
	{
		Master->SlavePoll->NoResponseCnt++;
	}

	AvrUartPutChar(Master->Uart, Master->SlavePoll->Id);
	AvrUartPutChar(Master->Uart, AVR_MODBUS_ReadHolding);
	AvrUartPutChar(Master->Uart, (Master->SlavePoll->StartAddr >> 8));
	AvrUartPutChar(Master->Uart, (Master->SlavePoll->StartAddr & 0x00FF));
	AvrUartPutChar(Master->Uart, (Master->SlavePoll->NumberOfRegister >> 8));
	AvrUartPutChar(Master->Uart, (Master->SlavePoll->NumberOfRegister & 0x00FF));

	Crc16 = Crc16Check(TxQue->OutPtr, TxQue->Buf, &TxQue->Buf[TxQue->Size - 1], TxQue->Ctr);

	AvrUartPutChar(Master->Uart, (Crc16 >> 8));
	AvrUartPutChar(Master->Uart, (Crc16 & 0x00FF));
	AvrUartStartTx(Master->Uart);
}
/*********************************************************************************/
static void MasterReceive(tag_AvrModbusMasterCtrl *Master)
{
	unsigned int Crc16, Length, i, j = 3;
	tag_AvrUartRingBuf *RxQue = &Master->Uart->RxQueue;
	tag_AvrModbusMasterSlaveInfo *Slave;

	/*
		1) 인수
			- Master : tag_AvrModbusMasterCtrl 인스턴스의 주소.

		2) 반환
			- 없음.

		3) 설명
			- Slave의 ReadHolding 응답을 처리함.
	*/

	Slave = FindSlaveById(Master, RxQue->Buf[0]);

	if((Slave != null) && (RxQue->Buf[1] == AVR_MODBUS_ReadHolding))
	{
		Crc16 = Crc16Check(RxQue->OutPtr, RxQue->Buf, &RxQue->Buf[RxQue->Size - 1], RxQue->Ctr - 2);

		if((RxQue->Buf[RxQue->Ctr - 2] == (Crc16 >> 8)) && (RxQue->Buf[RxQue->Ctr - 1] == (Crc16 & 0x00FF)))
		{
			Slave->NoResponseCnt = 0;

			Length = Slave->NumberOfRegister * 2;
			for(i = 0; i < Length; i += 2)
			{
				*(Slave->BaseAddr + i + 1) = RxQue->Buf[j++];
				*(Slave->BaseAddr + i) = RxQue->Buf[j++];
			}

			if(Master->Bit.InitRxUserException == true)
			{
				Master->UserException(Slave->Id);
			}
		}
	}
}
/*********************************************************************************/
char AvrModbusMasterGeneralInit(tag_AvrModbusMasterCtrl *Master, tag_AvrUartCtrl *Uart, char MaxSlave, long MasterProcTick_us)
{
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

	if(Uart->Bit.InitComplete == true)
	{
		Master->Uart = Uart;
	}

	Master->SlaveArray = (tag_AvrModbusMasterSlaveInfo *) calloc(MaxSlave, sizeof(tag_AvrModbusMasterSlaveInfo));
	Master->MaxSlave = MaxSlave;
	Master->Status = AVR_MODBUS_ReadHolding;
	Master->Tick_us = MasterProcTick_us;
	Master->Uart->ReceivingDelay = AVR_MODBUS_RECEIVING_DELAY_US / Master->Tick_us;
	Master->PollDelay = AVR_MODBUS_DEFAULT_POLLING_DELAY_US / Master->Tick_us;
	if(Master->Uart->ReceivingDelay < 2) Master->Uart->ReceivingDelay = 2;

	Master->Bit.InitGeneral = ((Master->Uart == null) || (Master->SlaveArray == null)) ? false : true;
	Master->Bit.InitComplete = CheckAllOfMasterInit(Master);

	return Master->Bit.InitGeneral;
}
/*********************************************************************************/
char AvrModbusMasterSetPollingDelay(tag_AvrModbusMasterCtrl *Master, long PollDelay_us)
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
char AvrModbusMasterAddSlave(tag_AvrModbusMasterCtrl *Master, unsigned char Id, int StartAddr, int NumberOfRegister, char *BaseAddr)
{
	char i;
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

	for(i = 0; i < Master->AddedSlave; i++)
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
			Master->SlaveArray[i].StartAddr = StartAddr;
			Master->SlaveArray[i].NumberOfRegister = NumberOfRegister;
			Master->SlaveArray[i].BaseAddr = BaseAddr;
			Master->SlaveArray[i].NoResponseCnt = 0;
			Master->SlaveArray[i].NoResponseLimit = AVR_MODBUS_DEFAULT_SLAVE_NO_RESPONSE;

			Master->SlaveReceive = Master->SlavePoll = &Master->SlaveArray[i];
			Master->AddedSlave++;
			return true;
		}
	}

	return false;
}
/*********************************************************************************/
void AvrModbusMasterRemoveSlave(tag_AvrModbusMasterCtrl *Master, unsigned char Id)
{
	tag_AvrModbusMasterSlaveInfo *Slave;

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

	Slave = FindSlaveById(Master, Id);

	if(Slave != null)
	{
		memset(Slave, 0, sizeof(tag_AvrModbusMasterSlaveInfo));
		Master->AddedSlave--;
	}
}
/*********************************************************************************/
void AvrModbusMasterSetSlaveNoResponse(tag_AvrModbusMasterCtrl *Master, unsigned char Id, unsigned char NoResponseLimit)
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

	Slave = FindSlaveById(Master, Id);

	if(Slave != null)
	{
		Slave->NoResponseLimit = NoResponseLimit;
		Slave->NoResponseCnt = 0;
	}
}
/*********************************************************************************/
char AvrModbusMasterLinkUserException(tag_AvrModbusMasterCtrl *Master, void (*UserException)(unsigned char Id))
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
			switch(Master->Status)
			{
				case	AVR_MODBUS_PresetSingle	:
				case	AVR_MODBUS_PresetMultiple	:
					Master->Status = AVR_MODBUS_ReadHolding;
					AvrUartStartTx(Master->Uart);
				break;

				default	:
				case	AVR_MODBUS_ReadHolding	:
					MasterPolling(Master);
				break;
			}
		}
	}
}
/*********************************************************************************/
char AvrModbusMasterPresetSingle(tag_AvrModbusMasterCtrl *Master, unsigned char SlaveId, int RegAddr, int PresetData)
{
	unsigned int Crc16;
	tag_AvrUartRingBuf *TxQue = &Master->Uart->TxQueue;

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

	if((Master->Bit.InitComplete == false) || (Master->AddedSlave == 0) || (Master->Status != AVR_MODBUS_ReadHolding))
	{
		return false;
	}

	Master->Status = AVR_MODBUS_PresetSingle;

	AvrUartPutChar(Master->Uart, SlaveId);
	AvrUartPutChar(Master->Uart, AVR_MODBUS_PresetSingle);
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
char AvrModbusMasterPresetMultiple(tag_AvrModbusMasterCtrl *Master, unsigned char SlaveId, int StartAddr, int NumberOfRegister, char *BaseAddr)
{
	unsigned int Crc16, i;
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

	if((Master->Bit.InitComplete == false) || (Master->AddedSlave == 0) || (Master->Status != AVR_MODBUS_ReadHolding))
	{
		return false;
	}

	Master->Status = AVR_MODBUS_PresetMultiple;

	AvrUartPutChar(Master->Uart, SlaveId);
	AvrUartPutChar(Master->Uart, AVR_MODBUS_PresetMultiple);
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
char AvrModbusMasterCheckSlaveNoResponse(tag_AvrModbusMasterCtrl *Master, unsigned char Id)
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

	Slave = FindSlaveById(Master, Id);

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
#endif
