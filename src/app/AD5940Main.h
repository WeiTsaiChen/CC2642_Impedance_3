/**  
 * @file       AD5940Main.h
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

#ifndef _AD5940MAIN_H_
#define _AD5940MAIN_H_
#define _AD5940MAIN_C_
#include "string.h"
#include "stdio.h"
#include "stdbool.h"

/*********************************************************************
*  EXTERNAL VARIABLES
*/

typedef struct{
  float         Freq[130];
  float         RzMag[130];
  float         RzPhase[130];
  // float         tVolt[250];
  // float         tmAmpere[250];
  int           RTIAgear;
  // float         tRzX[100];
  // float         tRzY[100];
  float         ADCvolt[130];
  int           GPIOBTNFlag;
}MeasurementData_Type;

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * MARCOS
 */

/*********************************************************************
 * FUNCTIONS
 */

int32_t ImpedanceShowResult(uint32_t *pData, uint32_t DataCount);
static int32_t AD5940PlatformCfg(void);
void AD5940ImpedanceStructInit(void);
void AD5940_Main(void);

void AD5940_measurement(int MeasureNum);

#endif