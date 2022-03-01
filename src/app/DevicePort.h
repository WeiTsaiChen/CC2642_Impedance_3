/**  
 * @file       DevicePort.h (ADICUP3029Port.h)
 * @brief      AD5940 library. This file contains all AD5940 library functions. 
 * @author     ADI
 * @date       March 2019
 * @par Revision History:
 * 
 * Copyright (c) 2017-2019 Analog Devices, Inc. All Rights Reserved.
 * 
 * This software is proprietary to Analog Devices, Inc. and its licensors.
 * By using this software you agree to the terms of the associated
 * Analog Devices Software License Agreement.
**/

/*********************************************************************
 * INCLUDES
 */

#ifndef _DEVICEPORT_H_
#define _DEVICEPORT_H_
#define _DEVICEPORT_C_
// #include "math.h"
// #include "string.h"
// #include "stdio.h"

/*********************************************************************
 * GLOBAL VARIABLES
 */

typedef struct
{
  char          SPIMessageChar[6];
  int           SPIRWtimes;
}SPITransferMessage_Type;

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * MARCOS
 */

/*********************************************************************
 * EXTERN FUNCTIONS
 */

void SetucInterrupted(uint32_t value);
void AD5940_ReadWriteNBytes(unsigned char *pSendBuffer,unsigned char *pRecvBuff,unsigned long length);
void AD5940_Delay10us(uint32_t time);
void SetucInterrupted(uint32_t value);
uint32_t AD5940_GetMCUIntFlag(void);
uint32_t AD5940_ClrMCUIntFlag(void);
uint32_t AD5940_MCUResourceInit(void *pCfg);
void Ext_Int0_Handler();

void CC2642_AD5941_SPI_Attribute(int datasize);

uint8_t CC2642_AD5941_SPI_8BReadWriteData(uint8_t *pWriteData, uint8_t *pReadData, int datalen); // SPI test only

#endif

