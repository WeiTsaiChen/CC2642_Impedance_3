/******************************************************************************

 @file  simple_peripheral.h

 @brief This file contains the Simple Peripheral sample application
        definitions and prototypes.

 Group: WCS, BTS
 Target Device: cc13x2_26x2

 ******************************************************************************
 
 Copyright (c) 2013-2019, Texas Instruments Incorporated
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

 *  Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

 *  Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

 *  Neither the name of Texas Instruments Incorporated nor the names of
    its contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 ******************************************************************************
 
 
 *****************************************************************************/

#ifndef SIMPLEPERIPHERAL_H
#define SIMPLEPERIPHERAL_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include <menu/two_btn_menu.h>
  
// PIN Drivers file
#include <ti/drivers/pin/PINCC26XX.h>
  
// Board file
#include "Board.h"
   
#include <ti/sysbios/knl/Clock.h>
#include <ti/drivers/GPIO.h>
#include <ti/drivers/PIN.h>
  
#include "gatt.h"
#include "att.h"

/*********************************************************************
*  EXTERNAL VARIABLES
*/

/* The running time of motor */
typedef struct{
  int           tTimeFan;
  int           tTimeMotorBlow;
  int           tTimeMotorStir;
}TimeDuration;

/* BLE Send/Receive data */
typedef struct
{
  int           TransferdataInt_list[3];
  float         Transferdatafloat_list[3];
  uint8_t       TransferdataU8_list[12];
  uint16_t      BLEMessageReceive;
  uint8_t       BLEValue_list[2];
  int           BLEValue_listpos;
}BLEData_Type;
  
/*********************************************************************
 * CONSTANTS
 */

#define Board_DIO11                             IOID_11

//*****************************************************************************
//
// BLE SENDING MODE IDENTIFY CODE
//
//*****************************************************************************

#define IMPEDANCE_MODE                          1
#define CV_MODE                                 2
#define MESSAGERETURN_MODE                      3
// #define SWV_MODE                                4
// #define DPV_MODE                                5
   
//*****************************************************************************
//
// BLE CHAR EVENT IDENTIFY CODE
//
//*****************************************************************************

#define CHARTEST_EVT                            0x10
#define CHARTEST_BRANCHCODE1                    0x01
#define CHARTEST_BRANCHCODE2                    0x02

//*****************************************************************************
//
// BLE MOTOR CONTROL SEPERATED EVENT IDENTIFY CODE
//
//*****************************************************************************

#define MOTORCONTROL_EVT                        0x20
#define MOTORCONTROL_DIO0ENABLECODE             0x01
#define MOTORCONTROL_DIO0DISABLECODE            0x02
#define MOTORCONTROL_DIO11ENABLECODE            0x03
#define MOTORCONTROL_DIO11DISABLECODE           0x04
#define MOTORCONTROL_DIO22ENABLECODE            0x05
#define MOTORCONTROL_DIO22DISABLECODE           0x06

//*****************************************************************************
//
// BLE MOTOR RUNNING TIME MODIFY EVENT IDENTIFY CODE
//
//*****************************************************************************

#define MOTOR_TIMEMODIFY_EVT                    0x50
#define DIO0_0MIN                               0x10
#define DIO0_6MIN                               0x16
#define DIO11_0MIN                              0x20
#define DIO11_6MIN                              0x26
#define DIO22_0MIN                              0x30
#define DIO22_6MIN                              0x36

//*****************************************************************************
//
// BLE MOTOR COMBINE WITH MEASUREMENT EVENT IDENTIFY CODE
//
//*****************************************************************************

#define MOTOR_COMBINE_MEASUREMENT_EVT           0x60
#define MOTOR_COMBINE_MEASUREMENT_ENABLECODE    0x01

//*****************************************************************************
//
// PIN CONFIGURATIONS
//
//*****************************************************************************

static PIN_State        PINstate;      // GPIOstate
static PIN_Handle       PINhandle;     // GPIOhandle

static PIN_Config PinsCfg[] = {
  // Low initially
  // Use static to avoid the changing of instructment
  Board_DIO0  | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,      // Fan, relay using
  Board_DIO22  | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,      // Motor pump-1, relay using  
  Board_DIO12 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MIN,      // AREF, should be 1.82V
  Board_DIO15 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,     // SPI-CS
  Board_DIO21 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,      // Reset
  IOID_11  | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,     // Motor pump-2, relay using 
  // IOID_9 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,           // Pin test, works
  
  PIN_TERMINATE
};

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * FUNCTIONS
 */

/*
 * Task creation function for the Simple Peripheral.
 */
extern void SimplePeripheral_createTask(void);

/*
 * Functions for menu action
 */
/* Actions for Menu: Choose connection to work with */
bool SimplePeripheral_doSelectConn(uint8 index);

/* Action for Menu: AutoConnect */
bool SimplePeripheral_doAutoConnect(uint8_t index);

/* Actions for Menu: Set PHY - Select */
bool SimplePeripheral_doSetConnPhy(uint8 index);

/* CS-Reset function */
void AD5940_CsClr(void);
void AD5940_CsSet(void);
void AD5940_RstSet(void);
void AD5940_RstClr(void);

/* BLE data exchange function */
void AD5941_TX_data_transmit(float Freq, float RzMag, float RzPhase, float Volt, float Ampere, uint16_t Notificationcode, int Txtrg);

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* SIMPLEPERIPHERAL_H */
