/*********************************************************************************/
/*
 * Author : Jeong Hyun Gu
 * File name : crc16.h
*/
/*********************************************************************************/
#ifndef __CRC16_H__
#define __CRC16_H__
/*********************************************************************************/
#include "SysTypedef.h"
/*********************************************************************************/
#define AVR_CRC16_REVISION_DATE    20210604
/*********************************************************************************/
/** REVISION HISTORY **/
/*
  2021. 06. 04.          - __CRC16_TARGET_MICROCHIP_STUDIO__ 추가.
  Jeong Hyun Gu

  2020. 09. 07.          - __CRC16_TARGET_IAR_AVR__ 추가. AVR 타겟일 경우 crc테이블을 플래쉬 메모리에 할당.
  Jeong Hyun Gu

  2019. 10. 07.          - 'SysTypedef.h' 적용.
  Jeong Hyun Gu

  2018. 05. 02.          - CRC16 테이블 변수 타입 변경.
  Jeong Hyun Gu

  2016. 11. 08.          - revision valid check 추가.
  Jeong Hyun Gu

  2016. 10. 28.          - 초기버전.
  Jeong Hyun Gu
*/
/*********************************************************************************/
/**Define**/

#define true    1
#define false   0
#define null    0

#define  __CRC16_TARGET_UNKNOWN__             0
#define __CRC16_TARGET_IAR_AVR__              1
#define __CRC16_TARGET_MICROCHIP_STUDIO__     2
#define __CRC16_TARGET_COMPILER__             __CRC16_TARGET_IAR_AVR__

/*********************************************************************************/
/**Enum**/


/*********************************************************************************/
/**Struct**/




/*********************************************************************************/
/**Function**/

tU16 Crc16Check(tU8 *BufCurPos, tU8 *BufOffSet, tU8 *BufEnd, tU16 Length);

/*********************************************************************************/
#endif //__CRC16_H__