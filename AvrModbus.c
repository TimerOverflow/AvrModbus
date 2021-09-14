/*********************************************************************************/
/*
 * Author : Jung Hyun Gu
 * File name : AvrModbus.c
*/
/*********************************************************************************/
#include <stdlib.h>
#include <string.h>
#include "include/AvrModbus.h"
#include "include/crc16.h"
/*********************************************************************************/
/** Global variable **/


/*********************************************************************************/
#if(AVR_MODBUS_SLAVE == true)

static char CheckAllOfSlaveInit(tag_AvrModbusSlaveCtrl *Slave)
{
	return (Slave->Bit.InitGeneral) ? true : false;
}
/*********************************************************************************/
static void ErrorException(tag_AvrModbusSlaveCtrl *Slave, char ErrCode)
{
	tag_AvrUartRingBuf *TxQue = &Slave->Uart->TxQueue;
	unsigned int Crc16;
	
	if(AvrUartViewRxBuf(Slave->Uart, 0, AVR_UART_FORWARD) != 0)
	{
		AvrUartPutChar(Slave->Uart, AvrUartViewRxBuf(Slave->Uart, 0, AVR_UART_FORWARD));
		AvrUartPutChar(Slave->Uart, AvrUartViewRxBuf(Slave->Uart, 1, AVR_UART_FORWARD) | 0x80);
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
	unsigned int StartAddr, NumberOfPoint, Crc16, i;
	char *TargetData;
	
	StartAddr = (int) (AvrUartViewRxBuf(Slave->Uart, 2, AVR_UART_FORWARD) << 8) + AvrUartViewRxBuf(Slave->Uart, 3, AVR_UART_FORWARD);
	NumberOfPoint = (int) (AvrUartViewRxBuf(Slave->Uart, 4, AVR_UART_FORWARD) << 8) + AvrUartViewRxBuf(Slave->Uart, 5, AVR_UART_FORWARD);
	
	StartAddr = (StartAddr < 200) ? 0 : StartAddr - 200;
	NumberOfPoint *= 2;
	TargetData = (char *) (((int *) Slave->TargetData) + StartAddr);
	
	AvrUartPutChar(Slave->Uart, AvrUartViewRxBuf(Slave->Uart, 0, AVR_UART_FORWARD));		//Slave Address
	AvrUartPutChar(Slave->Uart, AvrUartViewRxBuf(Slave->Uart, 1, AVR_UART_FORWARD));		//Function
	AvrUartPutChar(Slave->Uart, NumberOfPoint);																					//Byte Count
	
	for(i = 0; i < NumberOfPoint; i += 2)
	{
		AvrUartPutChar(Slave->Uart, *(TargetData + i + 1));
		AvrUartPutChar(Slave->Uart, *(TargetData + i));
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
	int RegisterAddr, PresetData, *TargetData;
	unsigned int Crc16;
	
	RegisterAddr = (int) (AvrUartViewRxBuf(Slave->Uart, 2, AVR_UART_FORWARD) << 8) + AvrUartViewRxBuf(Slave->Uart, 3, AVR_UART_FORWARD);
	PresetData = (int) (AvrUartViewRxBuf(Slave->Uart, 4, AVR_UART_FORWARD) << 8) + AvrUartViewRxBuf(Slave->Uart, 5, AVR_UART_FORWARD);
	
	if((Slave->Bit.InitCheckOutRange == true) && (Slave->CheckOutRange(RegisterAddr, 1) == true))
	{
		ErrorException(Slave, 2);
	}
	else
	{
		TargetData = ((int *) Slave->TargetData) + ((RegisterAddr < 200) ? 0 : RegisterAddr - 200);
		*TargetData = PresetData;
		
		if(Slave->Bit.InitUserException == true)
		{
			Slave->UserException(RegisterAddr, 1);
		}
	
		if(AvrUartViewRxBuf(Slave->Uart, 0, AVR_UART_FORWARD) != 0)
		{
			AvrUartPutChar(Slave->Uart, AvrUartViewRxBuf(Slave->Uart, 0, AVR_UART_FORWARD));		//Slave Address
			AvrUartPutChar(Slave->Uart, AvrUartViewRxBuf(Slave->Uart, 1, AVR_UART_FORWARD));		//Function
			AvrUartPutChar(Slave->Uart, AvrUartViewRxBuf(Slave->Uart, 2, AVR_UART_FORWARD));		//Register Address Hi
			AvrUartPutChar(Slave->Uart, AvrUartViewRxBuf(Slave->Uart, 3, AVR_UART_FORWARD));		//Register Address Lo
			AvrUartPutChar(Slave->Uart, AvrUartViewRxBuf(Slave->Uart, 4, AVR_UART_FORWARD));		//Preset Data Hi
			AvrUartPutChar(Slave->Uart, AvrUartViewRxBuf(Slave->Uart, 5, AVR_UART_FORWARD));		//Preset Data Lo
			
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
	unsigned int StartAddr, NumberOfRegister, Crc16, Length, i, j = 7;
	char *TargetData;
	
	StartAddr = (AvrUartViewRxBuf(Slave->Uart, 2, AVR_UART_FORWARD) << 8) + AvrUartViewRxBuf(Slave->Uart, 3, AVR_UART_FORWARD);
	NumberOfRegister = (AvrUartViewRxBuf(Slave->Uart, 4, AVR_UART_FORWARD) << 8) + AvrUartViewRxBuf(Slave->Uart, 5, AVR_UART_FORWARD);
	
	if((Slave->Bit.InitCheckOutRange == true) && (Slave->CheckOutRange(StartAddr, NumberOfRegister) == true))
	{
		ErrorException(Slave, 2);
	}
	else
	{
		Length = NumberOfRegister *= 2;
		Length = (Length > (Slave->Uart->RxQueue.Size - 9)) ? (Slave->Uart->RxQueue.Size - 9) : Length;
		TargetData = (char *) (((int *) Slave->TargetData) + ((StartAddr < 200) ? 0 : StartAddr - 200));
		
		for(i = 0; i < Length; i += 2)
		{
			*(TargetData + i + 1) = AvrUartViewRxBuf(Slave->Uart, j++, AVR_UART_FORWARD);
			*(TargetData + i) = AvrUartViewRxBuf(Slave->Uart, j++, AVR_UART_FORWARD);
		}
		
		if(Slave->Bit.InitUserException == true)
		{
			Slave->UserException(StartAddr, NumberOfRegister);
		}
		
		if(AvrUartViewRxBuf(Slave->Uart, 0, AVR_UART_FORWARD) != 0)
		{
			AvrUartPutChar(Slave->Uart, AvrUartViewRxBuf(Slave->Uart, 0, AVR_UART_FORWARD));
			AvrUartPutChar(Slave->Uart, AvrUartViewRxBuf(Slave->Uart, 1, AVR_UART_FORWARD));
			AvrUartPutChar(Slave->Uart, AvrUartViewRxBuf(Slave->Uart, 2, AVR_UART_FORWARD));
			AvrUartPutChar(Slave->Uart, AvrUartViewRxBuf(Slave->Uart, 3, AVR_UART_FORWARD));
			AvrUartPutChar(Slave->Uart, AvrUartViewRxBuf(Slave->Uart, 4, AVR_UART_FORWARD));
			AvrUartPutChar(Slave->Uart, AvrUartViewRxBuf(Slave->Uart, 5, AVR_UART_FORWARD));
			
			Crc16 = Crc16Check(TxQue->OutPtr, TxQue->Buf, &TxQue->Buf[TxQue->Size - 1], TxQue->Ctr);
			
			AvrUartPutChar(Slave->Uart, (Crc16 >> 8));
			AvrUartPutChar(Slave->Uart, (Crc16 & 0x00FF));
			AvrUartStartTx(Slave->Uart);
		}
	}
}
/*********************************************************************************/
char AvrModbusSlaveGeneralInit(tag_AvrModbusSlaveCtrl *Slave, tag_AvrUartCtrl *Uart, char *TargetData, long SlaveProcTick_us)
{
	if(Uart->Bit.InitComplete == true)
	{
		Slave->Uart = Uart;
		Slave->TargetData = TargetData;
		Slave->Uart->ReceivingDelay = AVR_MODBUS_RECEIVING_DELAY_US / SlaveProcTick_us;
	
		Slave->Bit.InitGeneral = true;
	}

	Slave->Bit.InitComplete = CheckAllOfSlaveInit(Slave);
	
	return Slave->Bit.InitGeneral;
}
/*********************************************************************************/
char AvrModbusSlaveLinkCheckRangeFunc(tag_AvrModbusSlaveCtrl *Slave, char (*CheckOutRange)(int StartAddr, int NumberOfRegister))
{
	Slave->CheckOutRange = CheckOutRange;
	Slave->Bit.InitCheckOutRange = true;
	
	return Slave->Bit.InitCheckOutRange;
}
/*********************************************************************************/
char AvrModbusSlaveLinkUserExceptionFunc(tag_AvrModbusSlaveCtrl *Slave, void (*UserException)(int StartAddr, int NumberOfRegister))
{
	Slave->UserException = UserException;
	Slave->Bit.InitUserException = true;
	
	return Slave->Bit.InitUserException;
}
/*********************************************************************************/
void AvrModbusSlaveProc(tag_AvrModbusSlaveCtrl *Slave, unsigned char SlaveAddr)
{
	tag_AvrUartRingBuf *RxQue = &Slave->Uart->RxQueue;
	unsigned int Crc16;
	
	
	if((AvrUartCheckRx(Slave->Uart) >= 1) && (AvrUartCheckReceiving(Slave->Uart) == false))
	{
		if((AvrUartViewRxBuf(Slave->Uart, 0, AVR_UART_FORWARD) == SlaveAddr ) || (AvrUartViewRxBuf(Slave->Uart, 0, AVR_UART_FORWARD) == 0) || (AvrUartViewRxBuf(Slave->Uart, 0, AVR_UART_FORWARD) == 255))
		{
			Crc16 = Crc16Check(RxQue->OutPtr, RxQue->Buf, &RxQue->Buf[RxQue->Size - 1], RxQue->Ctr - 2);
			
			if((AvrUartViewRxBuf(Slave->Uart, RxQue->Ctr - 2, AVR_UART_FORWARD) == (Crc16 >> 8)) && (AvrUartViewRxBuf(Slave->Uart, RxQue->Ctr - 1, AVR_UART_FORWARD) == (Crc16 & 0x00FF)))
			{
				switch(AvrUartViewRxBuf(Slave->Uart, 1, AVR_UART_FORWARD))
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
		AvrUartClearRx(Slave->Uart);
	}
}

#endif
/*********************************************************************************/
#if(AVR_MODBUS_MASTER == true)

static char CheckAllOfMasterInit(tag_AvrModbusMasterCtrl *Master)
{
	return (Master->Bit.InitGeneral && Master->Bit.InitPollDelay) ? true : false;
}
/*********************************************************************************/
static tag_AvrModbusMasterSlaveInfo* GetAddedSlaveInfo(tag_AvrModbusMasterCtrl *Master, tag_AvrModbusMasterSlaveInfo *Slave, enum_AvrModbusDirection Direction)
{
	char i = 0;
	tag_AvrModbusMasterSlaveInfo *SlaveTemp = Slave;
	
	do
	{
		switch(Direction)
		{
			default	:
			case	AVR_MODBUS_Next	:
				if(Slave == &Master->SlaveArray[Master->MaxSlave - 1])
				{
					Slave = Master->SlaveArray;
				}
				else
				{
					Slave++;
				}
			break;
			
			case	AVR_MODBUS_Prev	:
				if(Slave == Master->SlaveArray)
				{
					Slave = &Master->SlaveArray[Master->MaxSlave - 1];
				}
				else
				{
					Slave--;
				}
			break;
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
static char CheckSlaveId(tag_AvrModbusMasterCtrl *Master, char SlaveId)
{
	char i;
	
	for(i = 0; i < Master->AddedSlave; i++)
	{
		if(SlaveId == Master->SlaveReceive->Id)
		{
			return true;
		}
		
		Master->SlaveReceive = GetAddedSlaveInfo(Master, Master->SlaveReceive, AVR_MODBUS_Next);
	}
	
	return false;
}
/*********************************************************************************/
static void MasterPolling(tag_AvrModbusMasterCtrl *Master)
{
	tag_AvrUartRingBuf *TxQue = &Master->Uart->TxQueue;
	unsigned int Crc16;
	
	Master->SlavePoll = GetAddedSlaveInfo(Master, Master->SlavePoll, AVR_MODBUS_Next);
	if(Master->SlavePoll->NoResponseCnt < AVR_MODBUS_SLAVE_NO_RESPONSE)
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
	
	if(CheckSlaveId(Master, AvrUartViewRxBuf(Master->Uart, 0, AVR_UART_FORWARD)) == true)
	{
		Crc16 = Crc16Check(RxQue->OutPtr, RxQue->Buf, &RxQue->Buf[RxQue->Size - 1], RxQue->Ctr - 2);
		
		if((AvrUartViewRxBuf(Master->Uart, RxQue->Ctr - 2, AVR_UART_FORWARD) == (Crc16 >> 8)) && (AvrUartViewRxBuf(Master->Uart, RxQue->Ctr - 1, AVR_UART_FORWARD) == (Crc16 & 0x00FF)))
		{
			Master->SlaveReceive->NoResponseCnt = 0;
			Length = Master->SlaveReceive->NumberOfRegister * 2;
			
			for(i = 0; i < Length; i += 2)
			{
				*(Master->SlaveReceive->TargetData + i + 1) = AvrUartViewRxBuf(Master->Uart, j++, AVR_UART_FORWARD);
				*(Master->SlaveReceive->TargetData + i) = AvrUartViewRxBuf(Master->Uart, j++, AVR_UART_FORWARD);
			}
		}
	}
}
/*********************************************************************************/
static tag_AvrModbusMasterSlaveInfo* FindSlaveById(tag_AvrModbusMasterCtrl *Master, unsigned char Id)
{
	unsigned char i, Find = false;
	tag_AvrModbusMasterSlaveInfo *Slave = Master->SlaveArray;
	
	for(i = 0; i < Master->AddedSlave; i++)
	{
		if(Slave->Id == Id)
		{
			Find = true;
			break;
		}
		Slave = GetAddedSlaveInfo(Master, Slave, AVR_MODBUS_Next);
	}
	
	return Find ? Slave : null;
}
/*********************************************************************************/
char AvrModbusMasterGeneralInit(tag_AvrModbusMasterCtrl *Master, tag_AvrUartCtrl *Uart, char MaxSlave, long MasterProcTick_us)
{
	if(Uart->Bit.InitComplete == true)
	{
		Master->Uart = Uart;
	}
	
	Master->SlaveArray = (tag_AvrModbusMasterSlaveInfo *) calloc(MaxSlave, sizeof(tag_AvrModbusMasterSlaveInfo));
	Master->MaxSlave = MaxSlave;
	Master->Status = AVR_MODBUS_ReadHolding;
	Master->Uart->ReceivingDelay = AVR_MODBUS_RECEIVING_DELAY_US / MasterProcTick_us;
	if(Master->Uart->ReceivingDelay == 0) Master->Uart->ReceivingDelay = 2;
	
	Master->Bit.InitGeneral = ((Master->Uart == null) || (Master->SlaveArray == null)) ? false : true;
	Master->Bit.InitComplete = CheckAllOfMasterInit(Master);
	
	return Master->Bit.InitGeneral;
}
/*********************************************************************************/
char AvrModbusMasterSetPollingDelay(tag_AvrModbusMasterCtrl *Master, long PollDelay_us, long MasterProcTick_us)
{
	Master->PollDelay = PollDelay_us / MasterProcTick_us;
	
	Master->Bit.InitPollDelay = true;
	Master->Bit.InitComplete = CheckAllOfMasterInit(Master);
	
	return Master->Bit.InitPollDelay;
}
/*********************************************************************************/
char AvrModbusMasterAddSlave(tag_AvrModbusMasterCtrl *Master, unsigned char Id, int StartAddr, int NumberOfRegister, char *TargetData)
{
	char i;
	tag_AvrModbusMasterSlaveInfo *Slave = Master->SlaveArray;
	
	if(Master->Bit.InitComplete == false)
	{
		return false;
	}
	
	if(Master->AddedSlave >= Master->MaxSlave)
	{
		return false;
	}
	
	for(i = 0; i < Master->AddedSlave; i++)
	{
		if(Slave->Id == Id)
		{
			return false;
		}
		Slave = GetAddedSlaveInfo(Master, Slave, AVR_MODBUS_Next);
	}
	
	for(i = 0; i < Master->MaxSlave; i++)
	{
		if(Master->SlaveArray[i].Id == 0)
		{
			Master->SlaveArray[i].Id = Id;
			Master->SlaveArray[i].StartAddr = StartAddr;
			Master->SlaveArray[i].NumberOfRegister = NumberOfRegister;	
			Master->SlaveArray[i].TargetData = TargetData;
			
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
void AvrModbusMasterProc(tag_AvrModbusMasterCtrl *Master)
{
	if((Master->Bit.InitComplete == false) || (Master->AddedSlave == 0))
	{
		return;
	}
	
	if((AvrUartCheckRx(Master->Uart) >= 1) && (AvrUartCheckReceiving(Master->Uart) == false))
	{
		MasterReceive(Master);
		AvrUartClearRx(Master->Uart);
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
	
	if((Master->Bit.InitComplete == false) || (Master->AddedSlave == 0))
	{
		return false;
	}
	
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
char AvrModbusMasterPresetMultiple(tag_AvrModbusMasterCtrl *Master, unsigned char SlaveId, int StartAddr, int NumberOfRegister, char *TargetData)
{
	unsigned int Crc16, i;
	tag_AvrUartRingBuf *TxQue = &Master->Uart->TxQueue;
	
	if((Master->Bit.InitComplete == false) || (Master->AddedSlave == 0))
	{
		return false;
	}
	
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
		AvrUartPutChar(Master->Uart, *(TargetData + i + 1));
		AvrUartPutChar(Master->Uart, *(TargetData + i));
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
	
	if((Master->Bit.InitComplete == false) || (Master->AddedSlave == 0))
	{
		return false;
	}
	
	Slave = FindSlaveById(Master, Id);
	
	if((Slave != null) && (Slave->NoResponseCnt >= AVR_MODBUS_SLAVE_NO_RESPONSE))
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










