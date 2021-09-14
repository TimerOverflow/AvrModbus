/*********************************************************************************/
/*
 * Author : Jeong Hyun Gu
 * File name : AvrModbus.h
*/
/*********************************************************************************/
#ifndef __AVR_MODBUS_H__
#define	__AVR_MODBUS_H__
/*********************************************************************************/
#include "AvrUart.h"
/*********************************************************************************/
#define AVR_MODBUS_REVISION_DATE				20190507
/*********************************************************************************/
/** REVISION HISTORY **/
/*
	2019. 05. 07.					- Master파트 슬레이브 호출 펑션 AVR_MODBUS_ReadInput(0x04) 지원 추가.
	Jeong Hyun Gu					- AvrModbusMasterSetSlavePollFunction() 추가.

	2019. 04. 16.					- Slave파트 AvrModbusSlaveProc()함수에서 CRC에러 코드 응답할 때 유효한 펑션코드가 수신 되었는지 검사 추가.
	Jeong Hyun Gu						(프레임 인식에 오류 발견, A슬레이브 응답 중 B슬레이브가 ID패킷을 수신하지 못 하고 우연히 B슬레이브를 호출하는 것으로 인식하여
													동작. 결과적으로 B슬레이브가 CRC에러 응답을 하여 정상응답 패킷과 프레임 충돌 발생. 근본적인 원인 파악이 필요하나 시간관계상
													위와 같은 임시조치 추가. 이러한 이슈는 본 모듈을 사용한 슬레이브가 2대 이상일 경우 발생 가능성 있음. 추후 확인 필요.)
												- AVR_MODBUS_RECEIVING_DELAY_US 값 '50000' -> '20000' 변경.

	2019. 03. 19.					-	tag_AvrModbusMasterCtrl::SlaveReceive 삭제.
	Jeong Hyun Gu					- AvrModbusMasterAddSlavePollData() 추가.
													이제 마스터는 슬레이브 데이터를 비선형적으로 호출 가능. 예를 들어 200~210번지, 260~280번지등
													PollData를 추가하여 순차 호출.
												- tag_AvrModbusMasterCtrl::Bit.PollDataAllocFail를 추가하여 동적 메모리 할당 실패 여부 확인.
													AvrModbusMasterAddSlave(), AvrModbusMasterAddSlavePollData()호출 후 위 비트 값이 1이면 에러, 0이면 정상.

	2019. 01. 10.					- AvrModbusSlaveProc함수의 인수 SlaveAddr -> SlaveId 변수명 변경.
	Jeong Hyun Gu					- Slave파트 PreUserException 추가.
												- Master파트 FindSlaveById함수를 AvrModbusMasterFindSlaveById()로 변경하고
													public으로 공개.

	2018. 10. 23.					- Master파트 AvrModbusMasterAddSlave()함수에서 중복 ID 검색수량
	Jeong Hyun Gu						tag_AvrModbusMasterCtrl::AddedSlave -> tag_AvrModbusMasterCtrl::MaxSlave 변경.
													tag_AvrModbusMasterCtrl::SlaveArray에서 추가된 Slave보다 앞쪽 배열이 비어 있는 경우
													중복 추가 되는 현상 수정.
												- Master파트 FindSlaveById()함수에서 지역변수 Slave의 초기 값
													tag_AvrModbusMasterCtrl::SlaveArray -> tag_AvrModbusMasterCtrl::SlavePoll 변경.
													tag_AvrModbusMasterCtrl::SlaveArray에서 제거하고자 하는 Slave가 tag_AvrModbusMasterCtrl::AddedSlave
													보다 뒤쪽에 위치할 경우 제거 되지 않던 현상 수정.

	2018. 09. 06.					- Slave파트 AVR_MODBUS_PresetMultiple 커맨드 처리에서
	Jeong Hyun Gu						tag_AvrModbusSlaveCtrl :: UserException() 함수의 NumberOfRegister인자가
													2배 큰 값으로 전달 되던 현상 수정.

	2017. 12. 27.					- ErrorException() Id가 0xFF일 경우 에러를 응답하지 않음.
	Jeong Hyun Gu

	2017. 07. 26.					- Uart::ReceivingDelay 최소 값 제한.
	Jeong Hyun Gu					- AvrModbusSlaveProc() 함수에 AvrUartFixTxEnableFloating() 호출 추가.

	2017. 03. 22.					- Slave파트 일부 public 함수에 초기화 확인 부분 추가.
	Jeong Hyun Gu					- Master 파트 AvrModbusMasterPresetSingle(), AvrModbusMasterPresetMultiple()는 폴링 상태일 때에만 처리함.
													(커맨드 처리전 중복 호출할 경우 정상처리 하지 못 함.)


	2017. 02. 24.					- Master파트 MasterReceive() 함수에 function code 확인 추가.
	Jeong Hyun Gu					- AvrModbusMasterSetSlaveNoResponse() 함수 추가.
												-	AvrModbusMasterSetPollingDelay() 함수의 'MasterProcTick_us' 인수 삭제.
													폴링 딜레이 함수는 더 이상 필수 초기화가 아니며, AvrModbusMasterGeneralInit()에서 디폴트 설정.
												- AvrUartViewRxBuf() 함수를 사용하지 않고 수신 버퍼에 직접 접근.
												- Master 파트 UserException() 함수 추가.

	2017. 01. 02.					- 2016. 12. 26.에 추가 되었던 내용 삭제. (AvrUart 모듈로 이동)
	Jeong Hyun Gu

	2016. 12. 26.					- Slave파트 AvrModbusSlaveProc()함수에 수신 대기 중 TX Enable 비활성화 추가.
	Jeong Hyun Gu

	2016. 12. 07.					- 'AvrUart' 모듈의 'enum_AvrUartMoveDirection' 타입 삭제와 관련된 부분 수정.
	Jeong Hyun Gu					- 'AvrUart' 모듈의 'AvrUartClearRx' 함수 삭제와 관련된 부분 수정.
												- 'enum_AvrModbusDirection' 타입 삭제.
												- 주석 추가.

	2016. 11. 21.					- Master파트 AvrModbusMasterPresetSingle(), AvrModbusMasterPresetMultiple() 함수에서
	Jeong Hyun Gu						Master->Status를 핸들링 하지 않던 부분 수정.

	2016. 11. 08.					- revision valid check 추가.
	Jeong Hyun Gu

	2016. 11. 04.					- Master파트 CheckSlaveId() 함수 삭제.
	Jeong Hyun Gu					- 하위호환 가능.

	2016. 11. 02.					- AvrModbusMasterGeneralInit() 함수 일부 수정. (초기화 및 Receiving 최소 지연 추가)
	Jeong Hyun Gu					- 마스터 변수명 변경. 'PollDelay_us' -> 'PollDelay'
												- UART 모듈 변수명 변경. 'Uart->ReceivingDelay_us' -> 'Uart->ReceivingDelay'
												- 전처리 값 변경 'AVR_MODBUS_RECEIVING_DELAY_US' 20000 -> 50000

	2016. 10. 28.					- 초기버전.
	Jeong Hyun Gu
*/
/*********************************************************************************/
/**Define**/

#define true		1
#define	false		0
#define	null		0

#define	AVR_MODBUS_MASTER			true
#define	AVR_MODBUS_SLAVE			true

#define AVR_MODBUS_RECEIVING_DELAY_US							20000
#define AVR_MODBUS_DEFAULT_POLLING_DELAY_US				500000
#define AVR_MODBUS_DEFAULT_SLAVE_NO_RESPONSE			20

/*********************************************************************************/
/**Enum**/

typedef enum
{
	AVR_MODBUS_ReadHolding = 0x03,
	AVR_MODBUS_ReadInput = 0x04,
	AVR_MODBUS_PresetSingle = 0x06,
	AVR_MODBUS_PresetMultiple = 0x10,
}enum_AvrModbusFunction;

/*********************************************************************************/
/**Struct**/


#if(AVR_MODBUS_SLAVE == true)

typedef struct tag_AvrModbusSlaveCtrl
{
	struct
	{
		char InitGeneral					:		1;
		char InitCheckOutRange		:		1;
		char InitUserException		:		1;
		char InitPreUserException	:		1;

		char InitComplete					:		1;
	}Bit;

	tag_AvrUartCtrl *Uart;
	char (*CheckOutRange)(int StartAddr, int NumberOfRegister);
	void (*UserException)(int StartAddr, int NumberOfRegister);
	char (*PreUserException)(struct tag_AvrModbusSlaveCtrl *Slave, unsigned char *SlaveId);
	char *BaseAddr;
}tag_AvrModbusSlaveCtrl;

#endif

#if(AVR_MODBUS_MASTER == true)

typedef struct
{
	int StartAddr;
	int NumberOfRegister;
	char *BaseAddr;
}tag_AvrModbusMasterSlavePollData;

typedef struct
{
	unsigned char Id;
	unsigned char NoResponseCnt;
	unsigned char NoResponseLimit;
	unsigned char PollDataIndex;
	unsigned char PollDataMax;
	enum_AvrModbusFunction PollFunction;
	tag_AvrModbusMasterSlavePollData *PollData;
}tag_AvrModbusMasterSlaveInfo;

typedef struct
{
	struct
	{
		char InitGeneral				:		1;
		char InitRxUserException:		1;
		char InitComplete				:		1;
		char PollDataAllocFail	:		1;
	}Bit;

	tag_AvrUartCtrl *Uart;

	long Tick_us;
	long PollDelay;
	long PollCnt;

	tag_AvrModbusMasterSlaveInfo *SlaveArray;
	tag_AvrModbusMasterSlaveInfo *SlavePoll;

	char MaxSlave;
	char AddedSlave;

	enum_AvrModbusFunction Status;
	void (*UserException)(unsigned char Id);

}tag_AvrModbusMasterCtrl;

#endif

/*********************************************************************************/
/**Function**/

#if(AVR_MODBUS_SLAVE == true)

char AvrModbusSlaveGeneralInit(tag_AvrModbusSlaveCtrl *Slave, tag_AvrUartCtrl *Uart, char *BaseAddr, long SlaveProcTick_us);
char AvrModbusSlaveLinkCheckRangeFunc(tag_AvrModbusSlaveCtrl *Slave, char (*CheckRange)(int StartAddr, int NumberOfRegister));
char AvrModbusSlaveLinkUserExceptionFunc(tag_AvrModbusSlaveCtrl *Slave, void (*UserException)(int StartAddr, int NumberOfRegister));
char AvrModbusSlaveLinkPreUserExceptionFunc(tag_AvrModbusSlaveCtrl *Slave, char (*PreUserException)(tag_AvrModbusSlaveCtrl *Slave, unsigned char *SlaveId));
void AvrModbusSlaveProc(tag_AvrModbusSlaveCtrl *Slave, unsigned char SlaveId);

#endif


#if(AVR_MODBUS_MASTER == true)

char AvrModbusMasterGeneralInit(tag_AvrModbusMasterCtrl *Master, tag_AvrUartCtrl *Uart, char MaxSlave, long MasterProcTick_us);
char AvrModbusMasterSetPollingDelay(tag_AvrModbusMasterCtrl *Master, long PollDelay_us);
char AvrModbusMasterAddSlave(tag_AvrModbusMasterCtrl *Master, unsigned char Id, int StartAddr, int NumberOfRegister, char *BaseAddr);
char AvrModbusMasterAddSlavePollData(tag_AvrModbusMasterCtrl *Master, unsigned char Id, int StartAddr, int NumberOfRegister, char *BaseAddr);
void AvrModbusMasterRemoveSlave(tag_AvrModbusMasterCtrl *Master, unsigned char Id);
void AvrModbusMasterSetSlaveNoResponse(tag_AvrModbusMasterCtrl *Master, unsigned char Id, unsigned char NoResponseLimit);
void AvrModbusMasterSetSlavePollFunction(tag_AvrModbusMasterCtrl *Master, unsigned char Id, enum_AvrModbusFunction PollFunction);
char AvrModbusMasterLinkUserException(tag_AvrModbusMasterCtrl *Master, void (*UserException)(unsigned char Id));

void AvrModbusMasterProc(tag_AvrModbusMasterCtrl *Master);
char AvrModbusMasterPresetSingle(tag_AvrModbusMasterCtrl *Master, unsigned char SlaveId, int RegAddr, int PresetData);
char AvrModbusMasterPresetMultiple(tag_AvrModbusMasterCtrl *Master, unsigned char SlaveId, int StartAddr, int NumberOfRegister, char *BaseAddr);
char AvrModbusMasterCheckSlaveNoResponse(tag_AvrModbusMasterCtrl *Master, unsigned char Id);
tag_AvrModbusMasterSlaveInfo* AvrModbusMasterFindSlaveById(tag_AvrModbusMasterCtrl *Master, unsigned char Id);

#endif

/*********************************************************************************/
#endif //__AVR_MODBUS_H__
