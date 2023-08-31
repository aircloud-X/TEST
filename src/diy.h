#ifdef __cplusplus
extern "C"
{
#endif

#ifndef _DIY_H_
#define _DIY_H_

#define APPBUFF_SIZE 64
    uint32_t AppBuff[APPBUFF_SIZE];
    int32_t ImpedanceShowResult(uint32_t *pData, uint32_t DataCount);
    void AD5940ImpedanceStructInit(void);
    void show();

#endif

#ifdef __cplusplus
}
#endif