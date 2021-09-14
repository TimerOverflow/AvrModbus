/*********************************************************************************/
/*
 * Author : Jung Hyun Gu
 * File name : AvrModbus.h
*/
/*********************************************************************************/
#ifndef __AVR_MODBUS_H__
#define	__AVR_MODBUS_H__
/*********************************************************************************/
#include "AvrUart.h"
/*********************************************************************************/
/** REVISION HISTORY **/
/*
	2016. 11. 04.					- Master파트 CheckSlaveId() 함수 삭제.
	Jung Hyun Gu					- 하위호한 가능.

	2016. 11. 02.					- AvrModbusMasterGeneralInit() 함수 일부 수정. (초기화 및 Receiving 최소 지연 추가)
	Jung Hyun Gu					- 마스터 변수명 변경. 'PollDelay_us' -> 'PollDelay'
												- UART 모듈 변수명 변경. 'Uart->ReceivingDelay_us' -> 'Uart->ReceivingDelay'
												- 전처리 값 변경 'AVR_MODBUS_RECEIVING_DELAY_US' 20000 -> 50000

	2016. 10. 28.					- 초기버전.
	Jung Hyun Gu
*/
/*********************************************************************************/
/**Define**/

#define true		1
#define	false		0
#define	null		0

#define	AVR_MODBUS_MASTER			true
#define	AVR_MODBUS_SLAVE			false

#define AVR_MODBUS_RECEIVING_DELAY_US			50000
#define AVR_MODBUS_SLAVE_NO_RESPONSE			20

/*********************************************************************************/
/**Enum**/

typedef enum
{
	AVR_MODBUS_ReadHolding = 0x03,
	AVR_MODBUS_PresetSingle = 0x06,
	AVR_MODBUS_PresetMultiple = 0x10,
	
}enum_AvrModbusFunction;

typedef enum
{
	AVR_MODBUS_Next = 0,
	AVR_MODBUS_Prev,
	
}enum_AvrModbusDirection;

/*********************************************************************************/
/**Struct**/


#if(AVR_MODBUS_SLAVE == true)

typedef struct
{
	struct
	{
		char InitGeneral					:		1;
		char InitCheckOutRange		:		1;
		char InitUserException		:		1;
		char InitComplete					:		1;
		
	}Bit;
	
	tag_AvrUartCtrl *Uart;
	char (*CheckOutRange)(int StartAddr, int NumberOfRegister);
	void (*UserException)(int StartAddr, int NumberOfRegister);
	char *TargetData;
	
}tag_AvrModbusSlaveCtrl;

#endif

#if(AVR_MODBUS_MASTER == true)

typedef struct
{
	unsigned char Id;
	int StartAddr;
	int NumberOfRegister;
	char *TargetData;
	char NoResponseCnt;
}tag_AvrModbusMasterSlaveInfo;
	
typedef struct
{
	struct
	{
		char InitGeneral				:		1;
		char InitPollDelay			:		1;
		char InitComplete				:		1;
		
	}Bit;
	
	tag_AvrUartCtrl *Uart;
	long PollDelay;
	long PollCnt;
	
	tag_AvrModbusMasterSlaveInfo *SlaveArray;
	tag_AvrModbusMasterSlaveInfo *SlavePoll;
	tag_AvrModbusMasterSlaveInfo *SlaveReceive;
	char MaxSlave;
	char AddedSlave;
	
	enum_AvrModbusFunction Status;
	
}tag_AvrModbusMasterCtrl;

#endif

/*********************************************************************************/
/**Function**/

#if(AVR_MODBUS_SLAVE == true)

char AvrModbusSlaveGeneralInit(tag_AvrModbusSlaveCtrl *Slave, tag_AvrUartCtrl *Uart, char *TargetData, long SlaveProcTick_us);
char AvrModbusSlaveLinkCheckRangeFunc(tag_AvrModbusSlaveCtrl *Slave, char (*CheckRange)(int StartAddr, int NumberOfRegister));
char AvrModbusSlaveLinkUserExceptionFunc(tag_AvrModbusSlaveCtrl *Slave, void (*UserException)(int StartAddr, int NumberOfRegister));
void AvrModbusSlaveProc(tag_AvrModbusSlaveCtrl *Slave, unsigned char SlaveAddr);

#endif


#if(AVR_MODBUS_MASTER == true)

char AvrModbusMasterGeneralInit(tag_AvrModbusMasterCtrl *Master, tag_AvrUartCtrl *Uart, char MaxSlave, long MasterProcTick_us);
char AvrModbusMasterSetPollingDelay(tag_AvrModbusMasterCtrl *Master, long PollDelay_us, long MasterProcTick_us);
char AvrModbusMasterAddSlave(tag_AvrModbusMasterCtrl *Master, unsigned char Id, int StartAddr, int NumberOfRegister, char *TargetData);
void AvrModbusMasterRemoveSlave(tag_AvrModbusMasterCtrl *Master, unsigned char Id);

void AvrModbusMasterProc(tag_AvrModbusMasterCtrl *Master);
char AvrModbusMasterPresetSingle(tag_AvrModbusMasterCtrl *Master, unsigned char SlaveId, int RegAddr, int PresetData);
char AvrModbusMasterPresetMultiple(tag_AvrModbusMasterCtrl *Master, unsigned char SlaveId, int StartAddr, int NumberOfRegister, char *TargetData);
char AvrModbusMasterCheckSlaveNoResponse(tag_AvrModbusMasterCtrl *Master, unsigned char Id);
	
#endif

/*********************************************************************************/
#endif //__AVR_MODBUS_H__











