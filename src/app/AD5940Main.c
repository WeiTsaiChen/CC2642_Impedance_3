/*!
 *****************************************************************************
 @file:    AD5940Main.c
 @author:  Neo Xu
 @brief:   Standard 4-wire or 2-wire impedance measurement example.
 -----------------------------------------------------------------------------

Copyright (c) 2017-2019 Analog Devices, Inc. All Rights Reserved.

This software is proprietary to Analog Devices, Inc. and its licensors.
By using this software you agree to the terms of the associated
Analog Devices Software License Agreement.
 
*****************************************************************************/
#include "Impedance.h"

#include "AD5940Main.h"
#include "simple_peripheral.h"
#include "math.h"
#include "ad5940.h"

#include <ti/sysbios/knl/Task.h>

/**
   User could configure following parameters
**/

// Final results Monitoring 
MeasurementData_Type            PMData;

#define APPBUFF_SIZE 512
uint32_t AppBuff[APPBUFF_SIZE];

int counter = 0;

bool RTIAFlag = 0;

/*****************************************************************************/

int32_t ImpedanceShowResult(uint32_t *pData, uint32_t DataCount)
{
  float freq;
  uint32_t rd;

  fImpPol_Type *pImp = (fImpPol_Type*)pData;
  AppIMPCtrl(IMPCTRL_GETFREQ, &freq);
  
  PMData.Freq[counter] = freq;
  
  /* Process data */
  for(int i=0;i<DataCount;i++)
  {
    // printf("RzMag: %f Ohm , RzPhase: %f \n",pImp[i].Magnitude,pImp[i].Phase*180/MATH_PI);
    // MData.tRzMag = pImp[i].Magnitude;
    // MData.tRzPhase = pImp[i].Phase*180/MATH_PI;
    
    PMData.RzMag[counter] = pImp[i].Magnitude;
    PMData.RzPhase[counter] = (-1)*pImp[i].Phase*180/MATH_PI;
    // MData.tRzX[counter] = (pImp[i].Magnitude)*cos(MData.tRzPhase[counter]);
    // MData.tRzY[counter] = (pImp[i].Magnitude)*sin(MData.tRzPhase[counter]);
    
    AD5941_TX_data_transmit(freq, pImp[i].Magnitude, (-1)*pImp[i].Phase*180/MATH_PI, 0, 0, 0, IMPEDANCE_MODE);
        
    rd = AD5940_ReadAfeResult(AFERESULT_SINC2);
    float diff_volt = AD5940_ADCCode2Volt(rd, ADCPGA_1P5, 1.82);
        
    PMData.ADCvolt[counter] = diff_volt+1.11;
        
    // printf("ADC Code:%d, diff-volt: %.4f, volt:%.4f\n",rd, (-1)*diff_volt, diff_volt+1.11);
    // MData.tADCvolt[counter] = diff_volt;
        
  }
 
  return 0;
  
}

static int32_t AD5940PlatformCfg(void)
{
  CLKCfg_Type clk_cfg;
  FIFOCfg_Type fifo_cfg;
  AGPIOCfg_Type gpio_cfg;

  /* Use hardware reset */
  AD5940_HWReset();
  AD5940_Initialize();
  /* Platform configuration */
  /* Step1. Configure clock */
  clk_cfg.ADCClkDiv = ADCCLKDIV_1;
  clk_cfg.ADCCLkSrc = ADCCLKSRC_HFOSC;
  clk_cfg.SysClkDiv = SYSCLKDIV_1;
  clk_cfg.SysClkSrc = SYSCLKSRC_HFOSC;
  clk_cfg.HfOSC32MHzMode = bFALSE;
  clk_cfg.HFOSCEn = bTRUE;
  clk_cfg.HFXTALEn = bFALSE;
  clk_cfg.LFOSCEn = bTRUE;
  AD5940_CLKCfg(&clk_cfg);
  /* Step2. Configure FIFO and Sequencer*/
  fifo_cfg.FIFOEn = bFALSE;
  fifo_cfg.FIFOMode = FIFOMODE_FIFO;
  fifo_cfg.FIFOSize = FIFOSIZE_4KB;                       /* 4kB for FIFO, The reset 2kB for sequencer */
  fifo_cfg.FIFOSrc = FIFOSRC_DFT;
  fifo_cfg.FIFOThresh = 4;//AppIMPCfg.FifoThresh;        /* DFT result. One pair for RCAL, another for Rz. One DFT result have real part and imaginary part */
  AD5940_FIFOCfg(&fifo_cfg);
  fifo_cfg.FIFOEn = bTRUE;
  AD5940_FIFOCfg(&fifo_cfg);
  
  /* Step3. Interrupt controller */
  AD5940_INTCCfg(AFEINTC_1, AFEINTSRC_ALLINT, bTRUE);   /* Enable all interrupt in INTC1, so we can check INTC flags */
  AD5940_INTCClrFlag(AFEINTSRC_ALLINT);
  AD5940_INTCCfg(AFEINTC_0, AFEINTSRC_DATAFIFOTHRESH, bTRUE); 
  AD5940_INTCClrFlag(AFEINTSRC_ALLINT);
  /* Step4: Reconfigure GPIO */
  gpio_cfg.FuncSet = GP0_INT|GP1_SLEEP|GP2_SYNC;
  gpio_cfg.InputEnSet = 0;
  gpio_cfg.OutputEnSet = AGPIO_Pin0|AGPIO_Pin1|AGPIO_Pin2;
  gpio_cfg.OutVal = 0;
  gpio_cfg.PullEnSet = 0;
  AD5940_AGPIOCfg(&gpio_cfg);
  AD5940_SleepKeyCtrlS(SLPKEY_UNLOCK);  /* Allow AFE to enter sleep mode. */
  return 0;
}

void AD5940ImpedanceStructInit(void)
{
  AppIMPCfg_Type *pImpedanceCfg;
  
  AppIMPGetCfg(&pImpedanceCfg);
  /* Step1: configure initialization sequence Info */
  pImpedanceCfg->SeqStartAddr = 0;
  pImpedanceCfg->MaxSeqLen = 512; /* @todo add checker in function */

  pImpedanceCfg->RcalVal = 10000.0; // Standard: 10000.0
  pImpedanceCfg->SinFreq = 10000.0;
  pImpedanceCfg->FifoThresh = 4;
	
  /* Set switch matrix to onboard(EVAL-AD5940ELECZ) dummy sensor. */
  /* Note the RCAL0 resistor is 10kOhm. */
  pImpedanceCfg->DswitchSel = SWD_CE0;
  pImpedanceCfg->PswitchSel = SWP_RE0;
  pImpedanceCfg->NswitchSel = SWN_SE0;  // Note, original: SWT_SE0
  pImpedanceCfg->TswitchSel = SWT_SE0LOAD; // Note, original: SWT_SE0LOAD
  /* The dummy sensor is as low as 5kOhm. We need to make sure RTIA is small enough that HSTIA won't be saturated. */
  pImpedanceCfg->HstiaRtiaSel = HSTIARTIA_10K; // Can use number to replace	
  
  // #define HSTIARTIA_200               0     /**< HSTIA Internal RTIA resistor 200  */
  // #define HSTIARTIA_1K                1     /**< HSTIA Internal RTIA resistor 1K   */
  // #define HSTIARTIA_5K                2     /**< HSTIA Internal RTIA resistor 5K   */
  // #define HSTIARTIA_10K               3     /**< HSTIA Internal RTIA resistor 10K  */
  // #define HSTIARTIA_20K               4     /**< HSTIA Internal RTIA resistor 20K  */
  // #define HSTIARTIA_40K               5     /**< HSTIA Internal RTIA resistor 40K  */
  // #define HSTIARTIA_80K               6     /**< HSTIA Internal RTIA resistor 80K  */
  // #define HSTIARTIA_160K              7     /**< HSTIA Internal RTIA resistor 160K */
  // #define HSTIARTIA_OPEN              8     /**< Open internal resistor */
  
  /* Configure the sweep function. */
  pImpedanceCfg->SweepCfg.SweepEn = bTRUE;
  pImpedanceCfg->SweepCfg.SweepStart = 10000.0;	/* Start from 1kHz */
  pImpedanceCfg->SweepCfg.SweepStop = 1.0;		/* Stop at 100kHz */
  // 10000Hz - 1000Hz is the best region of measurement.
  // Using the parameters below 1000Hz may cause incorrect data output spontinously.
  
  pImpedanceCfg->SweepCfg.SweepPoints = 41;		/* Points is 101 */
  pImpedanceCfg->SweepCfg.SweepLog = bTRUE;
  /* Configure Power Mode. Use HP mode if frequency is higher than 80kHz. */
  pImpedanceCfg->PwrMod = AFEPWR_LP;
  /* Configure filters if necessary */
  // pImpedanceCfg->ADCSinc3Osr = ADCSINC3OSR_4;		/* Sample rate is 800kSPS/2 = 400kSPS */
  pImpedanceCfg->ADCSinc2Osr = ADCSINC2OSR_44;
  
  pImpedanceCfg->DftNum = DFTNUM_16384;
  // pImpedanceCfg->DftSrc = DFTSRC_SINC3;                 /* High speed mode */
  pImpedanceCfg->DftSrc = DFTSRC_SINC2NOTCH;            /* Low speed mode */
  
  // pImpedanceCfg->DftSrc = DFTSRC_ADCRAW;
  
  pImpedanceCfg->AdcPgaGain = ADCPGA_1P5;               /* PGA gain select 1 (Original setting)*/
  
  pImpedanceCfg->ExcitBufGain = EXCITBUFGAIN_0P25;
  // pImpedanceCfg->HsDacGain = HSDACGAIN_1;
  pImpedanceCfg->HsDacGain = HSDACGAIN_0P2;
  // HSDACGAIN_1
  pImpedanceCfg->DacVoltPP = 200; // DC, min = 200mV
  pImpedanceCfg->BiasVolt = -18.3; 
  // BiasVolt = -18.3 mV
  /* Configure Excitation Waveform 
	*
  *	 Output waveform = DacVoltPP * ExcitBufGain * HsDacGain 
	* 	
	*		= 300 * 0.25 * 0.2 = 15mV pk-pk ac
	*
	*/
  
}

void AD5940_measurement(int MeasureNum)
{
  uint32_t temp;
  AppIMPCtrl(IMPCTRL_START, 0);
    
  while(MeasureNum > counter)
  {
    // LED_control(1);
    
    // uint32_t rd;
    
    // ttttFlag.Data = GPIO_read(Board_GPIO_BTN1);
    PMData.GPIOBTNFlag = GPIO_read(Board_GPIO_BTN1);
    
    if(AD5940_GetMCUIntFlag())
    {    
      AD5940_ClrMCUIntFlag(); // clear the flag
      temp = APPBUFF_SIZE;
      AppIMPISR(AppBuff, &temp); // Data start from this section
     
      ImpedanceShowResult(AppBuff, temp);
      
      // rd = AD5940_ReadAfeResult(AFERESULT_SINC2);
      // float diff_volt = AD5940_ADCCode2Volt(rd, ADCPGA_1P5, 1.82);
      // printf("ADC Code:%d, diff-volt: %.4f, volt:%.4f\n",rd, diff_volt, diff_volt+1.11);
      
      // rd = AD5940_ReadAfeResult(AFERESULT_SINC2);
      // float diff_volt = AD5940_ADCCode2Volt(rd, ADCPGA_1P5, 1.82);
      
      // printf("ADC Code:%d, diff-volt: %.4f, volt:%.4f\n",rd, (-1)*diff_volt, diff_volt+1.11);
      // MData.tADCvolt[counter] = diff_volt;
      
      counter++;
    }
  }
  
  if(MeasureNum == counter)
  {
    counter = 0;
    AppIMPCtrl(IMPCTRL_STOPNOW, 0);
    Task_sleep(10);
  }
  
}

void AD5940_Main(void)
{
  // uint32_t temp;  
  
  AD5940PlatformCfg();
  AD5940ImpedanceStructInit(); // Initialize system, also calibrate manual

  AppIMPInit(AppBuff, APPBUFF_SIZE);    /* Initialize IMP application. Provide a buffer, which is used to store sequencer commands */
}

// void AD5940_Main(void)
// {
  // uint32_t temp;
  // AD5940PlatformCfg();
  // AD5940ImpedanceStructInit();

  // AppIMPInit(AppBuff, APPBUFF_SIZE);    /* Initialize IMP application. Provide a buffer, which is used to store sequencer commands */
  // AppIMPCtrl(IMPCTRL_START, 0);          /* Control IMP measurement to start. Second parameter has no meaning with this command. */

  // while(1)
  // {
    // if(AD5940_GetMCUIntFlag())
    // {
      // AD5940_ClrMCUIntFlag();
      // temp = APPBUFF_SIZE;
      // AppIMPISR(AppBuff, &temp);
      // ImpedanceShowResult(AppBuff, temp);
    // }
  // }
// }