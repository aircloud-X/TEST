#include "Impedance.h"
#include "diy.h"

int32_t ImpedanceShowResult(uint32_t *pData, uint32_t DataCount)
{
  float freq;

  fImpPol_Type *pImp_Pol = (fImpPol_Type *)pData;
  fImpCar_Type *pImp_Car = (fImpCar_Type *)pData;
  AppIMPCtrl(IMPCTRL_GETFREQ, &freq);

  printf("Freq:%.2f ", freq);
  printf("RzMag: %.2f Ohm , RzPhase: %.2f \n", AD5940_ComplexMag(&pImp_Car[0]) - 2073, AD5940_ComplexPhase(&pImp_Car[0]) * 180 / MATH_PI);
  // /*Process data*/
  // for (int i = 0; i < DataCount; i++)
  // {
  //   printf("RzMag: %f Ohm , RzPhase: %f \n", pImp_Pol[i].Magnitude, pImp_Pol[i].Phase * 180 / MATH_PI);
  // }
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

void show()
{
  uint32_t temp;
  // if (AD5940_GetMCUIntFlag())
  // {
  // AD5940_ClrMCUIntFlag();
  temp = APPBUFF_SIZE;
  AppIMPISR(AppBuff, &temp);
  ImpedanceShowResult(AppBuff, temp);
}