#define ENABLE_BIT_DEFINITIONS
#include <iom64.h>
#include <ina90.h>
#include "AvrModbus.h"
#include "AvrUart.h"
#include "AvrUartBaud.h"
#include "AvrTimer.h"

#define __CPU_CLK__                   14745600L   //14.7456Mhz
#define T_SYSTEM_TICK                 0.01        //10ms

#define GPIO_485_ENABLE_PORT          PORTD
#define GPIO_485_ENABLE_PIN           PIND
#define GPIO_485_ENABLE               5

typedef struct
{
  tU16 Reg0;        //Addr: 0
  tU16 Reg1;        //Addr: 1
  tU16 Reg2;        //Addr: 2
  tU16 Reg3;        //Addr: 3
  
  tU16 Reg10;       //Addr: 10
  tU16 Reg11;       //Addr: 11
  tU16 Reg12;       //Addr: 12
}tag_SlaveStruct;
tag_SlaveStruct SlaveData;

tag_AvrUartCtrl Uart0;
tag_UartBaudControl Uart0Baud;
tag_AvrModbusMasterCtrl AvrModbusMaster;

tU8 SysTick10ms;
tU8 IsSlaveNoRespone;

tU8 Uart0_TxBuf[256];
tU8 Uart0_RxBuf[256];

#pragma vector = USART0_RXC_vect
__interrupt void USART0_RXC_ISR(void)
{
  AvrUartRxQueueControl(&Uart0);
}

#pragma vector = USART0_TXC_vect
__interrupt void USART0_TXC_ISR(void)
{
  AvrUartTxQueueControl(&Uart0);
}

#pragma vector = TIMER0_OVF_vect
__interrupt void TIMER0_OVF_ISR(void)
{
  //10ms
  TCNT0 = (tU8) TIMER_TCNT_8BIT(T_SYSTEM_TICK, __CPU_CLK__, 1024);
  SysTick10ms = true;
}

void ModbusMasterUserException(tU8 Id)
{
  //do something for user exception. this function call after receive slave respone.
}

void main( void )
{
  tU8 MaxSlave = 1;
  tU32 MasterProcTick_us = 10000;
  tU32 PollDelay_us = 500000;
  tU8 SlaveId = 1;
  tU16 SlaveStartAddr = 0;
  tU16 SlaveNumberOfRegister = 4;
  tU16 SlaveStartAddr2 = 10;
  tU16 SlaveNumberOfRegister2 = 2;
  tU16 SlaveWriteAddr = 3;
  tU16 SlaveWriteData = 0;
  tU16 SendDelay;
  
  DDRD |= (1 << DDD5);
  //GPIO direction.
  
  UCSR0B = (1 << RXCIE0) | (1 << TXCIE0) | (1 << RXEN0) | (1 << TXEN0);
  //uart enable.
  
  TCCR0 = (1 << CS02) | (1 << CS01) | (1 << CS00);
  TCNT0 = 0xFF;
  //timer0 set.
  
  TIMSK = (1 << TOIE0);
  //timer interrupt enable.
  
  AvrUartLinkRegister(&Uart0, (tU8 *) &UDR0, (tU8 *) &UCSR0A, (tU8 *) &GPIO_485_ENABLE_PORT, GPIO_485_ENABLE);
  AvrUartLinkBuffer(&Uart0, Uart0_TxBuf, sizeof(Uart0_TxBuf), Uart0_RxBuf, sizeof(Uart0_RxBuf));
  AvrUartGeneralInit(&Uart0);
  //uart0 init.
  
  AvrUartBaudControlInit(&Uart0Baud, __CPU_CLK__, (tU8 *) &UBRR0L, (tU8 *) &UBRR0H);
  //uart0baud init.
  
  AvrUartBaudChange(&Uart0Baud, BAUD_9600);
  //setting baudrate
  
  AvrModbusMasterGeneralInit(&AvrModbusMaster, &Uart0, MaxSlave, MasterProcTick_us);
  AvrModbusMasterSetPollingDelay(&AvrModbusMaster, PollDelay_us);
  AvrModbusMasterAddSlave(&AvrModbusMaster, SlaveId, AVR_MODBUS_ReadHoldingRegister, SlaveStartAddr, SlaveNumberOfRegister, (tU8 *) &SlaveData.Reg0);
  AvrModbusMasterAddSlavePollData(&AvrModbusMaster, SlaveId, AVR_MODBUS_ReadHoldingRegister, SlaveStartAddr2, SlaveNumberOfRegister2, (tU8 *) &SlaveData.Reg10);
  AvrModbusMasterLinkUserException(&AvrModbusMaster, ModbusMasterUserException);
  /*
    "MasterProcTick_us" must be set to the same period as AvrModbusMasterProc() call.
  */
  
  __enable_interrupt();
  
  while(1)
  {
    if(SysTick10ms == true)
    {
      SysTick10ms = false;
      
      AvrModbusMasterProc(&AvrModbusMaster);
      IsSlaveNoRespone = AvrModbusMasterCheckSlaveNoResponse(&AvrModbusMaster, SlaveId);
      
      if(SendDelay == 0)
      {
        if(IsSlaveNoRespone == false)
        {
          if(SlaveData.Reg3 > 10){ AvrModbusMasterWriteSingle(&AvrModbusMaster, SlaveId, AVR_MODBUS_PresetSingleRegister, SlaveWriteAddr, SlaveWriteData); SendDelay = 500; }
        }
      }
      else
      {
        SendDelay--;
      }
    }
  }
}
