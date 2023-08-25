
// // 阻抗扫频分析
// /*!
//  *****************************************************************************
//  @file:    AD5940Main.c
//  @author:  Neo Xu
//  @brief:   Used to control specific application and further process data.
//  -----------------------------------------------------------------------------

// Copyright (c) 2017-2019 Analog Devices, Inc. All Rights Reserved.

// This software is proprietary to Analog Devices, Inc. and its licensors.
// By using this software you agree to the terms of the associated
// Analog Devices Software License Agreement.

// *****************************************************************************/
// /**
//  * @addtogroup AD5940_System_Examples
//  * @{
//  *  @defgroup BioElec_Example
//  *  @{
//  */
// #include <Arduino.h>
// #include "AD5940.h"
// #include <stdio.h>
// #include "string.h"
// #include "math.h"
// #include "BIOZ-2Wire.h"

// #define APPBUFF_SIZE 64
// uint32_t AppBuff[APPBUFF_SIZE];

// /* It's your choice here how to do with the data. Here is just an example to print them to UART */
// int32_t BIOZShowResult(uint32_t *pData, uint32_t DataCount)
// {
//   // float freq;

//   // fImpCar_Type *pImp = (fImpCar_Type *)pData;
//   // AppBIOZCtrl(BIOZCTRL_GETFREQ, &freq);

//   // /*Process data*/
//   // for (int i = 0; i < DataCount; i++) {
//   //   printf("Freq:%.2f\n", freq/10000);
//   //   //printf("RzMag: %f Ohm , RzPhase: %f \n",AD5940_ComplexMag(&pImp[i]), AD5940_ComplexPhase(&pImp[i])*180/MATH_PI);
//   //   // printf("Impedance:(Real,Image) = (%f,%f)\n", pImp[i].Real, pImp[i].Image);

//   //   printf("Real:%f\n", pImp[i].Real/100000);

//   //   printf("Image:%f)\n", pImp[i].Image/100000);

//   //   printf("test:%f)\n", pImp[i].Image/pImp[i].Real);

//   // }
//   // return 0;

//   float freq;

//   fImpCar_Type *pImp = (fImpCar_Type *)pData;
//   AppBIOZCtrl(BIOZCTRL_GETFREQ, &freq);

//   /*Process data*/
//   for (int i = 0; i < DataCount; i++)
//   {
//     printf("Freq:%.2f ", freq);
//     //   if (isnanf(AD5940_ComplexMag(&pImp[i])) == 1) // 判断电阻值是否是nan
//     printf("RzMag: %f Ohm , RzPhase: %f \n", AD5940_ComplexMag(&pImp[i]), AD5940_ComplexPhase(&pImp[i]) * 180 / MATH_PI);
//     // printf("Impedance:(Real,Image) = (%f,%f)\n", pImp[i].Real, pImp[i].Image);
//     // printf("test:%f)\n", AD5940_ComplexMag(&pImp[i]));
//   }
//   return 0;
// }

// /* Initialize AD5940 basic blocks like clock */
// static int32_t AD5940PlatformCfg(void)
// {
//   CLKCfg_Type clk_cfg;
//   FIFOCfg_Type fifo_cfg;
//   AGPIOCfg_Type gpio_cfg;

//   /*重要！！！需要初始化esp32的spi，否则esp32的spi会报错*/
//   AD5940_MCUResourceInit(NULL);
//   printf("MCU Initialised\n");
//   /* Use hardware reset */
//   AD5940_HWReset();
//   /* Platform configuration */
//   AD5940_Initialize();
//   printf("AD5941 Initialised\n");
//   /* Step1. Configure clock */
//   clk_cfg.ADCClkDiv = ADCCLKDIV_1;
//   clk_cfg.ADCCLkSrc = ADCCLKSRC_XTAL;
//   clk_cfg.SysClkDiv = SYSCLKDIV_1;
//   clk_cfg.SysClkSrc = SYSCLKSRC_XTAL;
//   clk_cfg.HfOSC32MHzMode = bFALSE;
//   clk_cfg.HFOSCEn = bFALSE;
//   clk_cfg.HFXTALEn = bTRUE;
//   clk_cfg.LFOSCEn = bTRUE;
//   AD5940_CLKCfg(&clk_cfg);
//   printf("1\n");

//   /* Step2. Configure FIFO and Sequencer*/
//   fifo_cfg.FIFOEn = bFALSE;
//   fifo_cfg.FIFOMode = FIFOMODE_FIFO;
//   fifo_cfg.FIFOSize = FIFOSIZE_4KB; /* 4kB for FIFO, The reset 2kB for sequencer */
//   fifo_cfg.FIFOSrc = FIFOSRC_DFT;
//   fifo_cfg.FIFOThresh = 4;
//   AD5940_FIFOCfg(&fifo_cfg); /* Disable to reset FIFO. */
//   fifo_cfg.FIFOEn = bTRUE;
//   AD5940_FIFOCfg(&fifo_cfg); /* Enable FIFO here */

//   /* Step3. Interrupt controller */
//   AD5940_INTCCfg(AFEINTC_1, AFEINTSRC_ALLINT, bTRUE);         /* Enable all interrupt in Interrupt Controller 1, so we can check INTC flags */
//   AD5940_INTCCfg(AFEINTC_0, AFEINTSRC_DATAFIFOTHRESH, bTRUE); /* Interrupt Controller 0 will control GP0 to generate interrupt to MCU */
//   AD5940_INTCClrFlag(AFEINTSRC_ALLINT);
//   /* Step4: Reconfigure GPIO */
//   gpio_cfg.FuncSet = GP6_SYNC | GP5_SYNC | GP4_SYNC | GP2_TRIG | GP1_SYNC | GP0_INT;
//   gpio_cfg.InputEnSet = AGPIO_Pin2;
//   gpio_cfg.OutputEnSet = AGPIO_Pin0 | AGPIO_Pin1 | AGPIO_Pin4 | AGPIO_Pin5 | AGPIO_Pin6;
//   gpio_cfg.OutVal = 0;
//   gpio_cfg.PullEnSet = 0;

//   AD5940_AGPIOCfg(&gpio_cfg);
//   AD5940_SleepKeyCtrlS(SLPKEY_UNLOCK); /* Allow AFE to enter sleep mode. */

//   printf("AD5940PlatformCfg end!!!\n");
//   return 0;
// }

// /* !!Change the application parameters here if you want to change it to none-default value */
// void AD5940BIOZStructInit(void)
// {
//   AppBIOZCfg_Type *pBIOZCfg;
//   AppBIOZGetCfg(&pBIOZCfg);

//   pBIOZCfg->SeqStartAddr = 0;
//   pBIOZCfg->MaxSeqLen = 512;

//   pBIOZCfg->SinFreq = 20e3;   /* 20kHz. This value is ignored if SweepEn = bTRUE */
//   pBIOZCfg->RcalVal = 1000.0; /* Value of RCAl on the evaluaiton board */
//   pBIOZCfg->HstiaRtiaSel = HSTIARTIA_200;

//   /* Configure Switch matrix */
//   pBIOZCfg->DswitchSel = SWD_CE0;
//   pBIOZCfg->PswitchSel = SWP_CE0;
//   pBIOZCfg->NswitchSel = SWN_AIN1;
//   pBIOZCfg->TswitchSel = SWN_AIN1;

//   /* Configure Sweep Parameters */
//   pBIOZCfg->SweepCfg.SweepEn = bFALSE;
//   pBIOZCfg->SweepCfg.SweepStart = 10000;
//   pBIOZCfg->SweepCfg.SweepStop = 200000.0;
//   pBIOZCfg->SweepCfg.SweepPoints = 40; /* Maximum is 100 */
//   pBIOZCfg->SweepCfg.SweepLog = bFALSE;

//   pBIOZCfg->BIOZODR = 10;   /* ODR(Sample Rate) 5Hz */
//   pBIOZCfg->NumOfData = -1; /* Never stop until you stop it manually by AppBIOZCtrl() function */
// }

// void setup()
// {

//   AD5940PlatformCfg();

//   AD5940BIOZStructInit(); /* Configure your parameters in this function */
//   printf("AD5940BIOZStructInit Initialised\n");

//   AppBIOZInit(AppBuff, APPBUFF_SIZE); /* Initialize BIOZ application. Provide a buffer, which is used to store sequencer commands */
//   AppBIOZCtrl(BIOZCTRL_START, 0);     /* Control BIOZ measurement to start. Second parameter has no meaning with this command. */
// }

// void loop()
// {
//   uint32_t temp;
//   /* Check if interrupt flag which will be set when interrupt occurred. */
//   if (AD5940_GetMCUIntFlag())
//   {
//     AD5940_ClrMCUIntFlag(); /* Clear this flag */
//     temp = APPBUFF_SIZE;
//     AppBIOZISR(AppBuff, &temp);    /* Deal with it and provide a buffer to store data we got */
//     BIOZShowResult(AppBuff, temp); /* Show the results to UART */
//   }
//   delay(10);
// }

/**
 * @}
 * @}
 * */

// 单独测阻抗值
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
#include <Arduino.h>
#include "Impedance.h"

/**
   User could configure following parameters
**/

#define APPBUFF_SIZE 64
uint32_t AppBuff[APPBUFF_SIZE];

int32_t ImpedanceShowResult(uint32_t *pData, uint32_t DataCount)
{
  float freq;

  fImpPol_Type *pImp_Pol = (fImpPol_Type *)pData;
  fImpCar_Type *pImp_Car = (fImpCar_Type *)pData;
  AppIMPCtrl(IMPCTRL_GETFREQ, &freq);

  printf("Freq:%.2f ", freq);
  printf("RzMag: %.2f Ohm , RzPhase: %.2f \n", AD5940_ComplexMag(&pImp_Car[0])-2073, AD5940_ComplexPhase(&pImp_Car[0]) * 180 / MATH_PI);
  // /*Process data*/
  // for (int i = 0; i < DataCount; i++)
  // {
  //   printf("RzMag: %f Ohm , RzPhase: %f \n", pImp_Pol[i].Magnitude, pImp_Pol[i].Phase * 180 / MATH_PI);
  // }
  return 0;
}

static int32_t AD5940PlatformCfg(void)
{
  CLKCfg_Type clk_cfg;
  FIFOCfg_Type fifo_cfg;
  AGPIOCfg_Type gpio_cfg;
  /*重要！！！需要初始化esp32的spi，否则esp32的spi会报错*/
  AD5940_MCUResourceInit(NULL);
  printf("MCU Initialised\n");
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
  fifo_cfg.FIFOSize = FIFOSIZE_4KB; /* 4kB for FIFO, The reset 2kB for sequencer */
  fifo_cfg.FIFOSrc = FIFOSRC_DFT;
  fifo_cfg.FIFOThresh = 4; // AppIMPCfg.FifoThresh;        /* DFT result. One pair for RCAL, another for Rz. One DFT result have real part and imaginary part */
  AD5940_FIFOCfg(&fifo_cfg);
  fifo_cfg.FIFOEn = bTRUE;
  AD5940_FIFOCfg(&fifo_cfg);

  /* Step3. Interrupt controller */
  AD5940_INTCCfg(AFEINTC_1, AFEINTSRC_ALLINT, bTRUE); /* Enable all interrupt in INTC1, so we can check INTC flags */
  AD5940_INTCClrFlag(AFEINTSRC_ALLINT);
  AD5940_INTCCfg(AFEINTC_0, AFEINTSRC_DATAFIFOTHRESH, bTRUE);
  AD5940_INTCClrFlag(AFEINTSRC_ALLINT);
  /* Step4: Reconfigure GPIO */
  gpio_cfg.FuncSet = GP0_INT | GP1_SLEEP | GP2_SYNC;
  gpio_cfg.InputEnSet = 0;
  gpio_cfg.OutputEnSet = AGPIO_Pin0 | AGPIO_Pin1 | AGPIO_Pin2;
  gpio_cfg.OutVal = 0;
  gpio_cfg.PullEnSet = 0;
  AD5940_AGPIOCfg(&gpio_cfg);
  AD5940_SleepKeyCtrlS(SLPKEY_UNLOCK); /* Allow AFE to enter sleep mode. */
  return 0;
}

void AD5940ImpedanceStructInit(void)
{
  AppIMPCfg_Type *pImpedanceCfg;

  AppIMPGetCfg(&pImpedanceCfg);
  /* Step1: configure initialization sequence Info */
  pImpedanceCfg->SeqStartAddr = 0;
  pImpedanceCfg->MaxSeqLen = 512; /* @todo add checker in function */

  pImpedanceCfg->RcalVal = 1000.0;
  pImpedanceCfg->SinFreq = 20000.0;
  pImpedanceCfg->FifoThresh = 4;

  /* Set switch matrix to onboard(EVAL-AD5940ELECZ) dummy sensor. */
  /* Note the RCAL0 resistor is 10kOhm. */
  pImpedanceCfg->DswitchSel = SWD_CE0;
  pImpedanceCfg->PswitchSel = SWP_CE0;
  pImpedanceCfg->NswitchSel = SWN_AIN1;
  pImpedanceCfg->TswitchSel = SWT_AIN1;
  /* The dummy sensor is as low as 5kOhm. We need to make sure RTIA is small enough that HSTIA won't be saturated. */
  pImpedanceCfg->HstiaRtiaSel = HSTIARTIA_200;

  /* Configure the sweep function. */
  pImpedanceCfg->SweepCfg.SweepEn = bFALSE;
  pImpedanceCfg->SweepCfg.SweepStart = 1e3f;  /* Start from 1kHz */
  pImpedanceCfg->SweepCfg.SweepStop = 100e3f; /* Stop at 100kHz */
  pImpedanceCfg->SweepCfg.SweepPoints = 101;  /* Points is 101 */
  pImpedanceCfg->SweepCfg.SweepLog = bFALSE;
  /* Configure Power Mode. Use HP mode if frequency is higher than 80kHz. */
  pImpedanceCfg->PwrMod = AFEPWR_LP;
  /* Configure filters if necessary */
  pImpedanceCfg->ADCSinc3Osr = ADCSINC3OSR_2; /* Sample rate is 800kSPS/2 = 400kSPS */
  pImpedanceCfg->DftNum = DFTNUM_16384;
  pImpedanceCfg->DftSrc = DFTSRC_SINC3;
}

void setup()
{

  AD5940PlatformCfg();
  AD5940ImpedanceStructInit();

  AppIMPInit(AppBuff, APPBUFF_SIZE); /* Initialize IMP application. Provide a buffer, which is used to store sequencer commands */
  AppIMPCtrl(IMPCTRL_START, 0);      /* Control IMP measurement to start. Second parameter has no meaning with this command. */
}
void loop()
{
  uint32_t temp;
  if (AD5940_GetMCUIntFlag())
  {
    AD5940_ClrMCUIntFlag();
    temp = APPBUFF_SIZE;
    AppIMPISR(AppBuff, &temp);
    ImpedanceShowResult(AppBuff, temp);
  }
  delay(500);
}