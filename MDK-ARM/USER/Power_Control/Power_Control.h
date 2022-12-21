#ifndef __Power_CONTROL_H
#define __Power_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include"main.h"
#include"dac_util.h"
#include"adc_util.h"
#include"pid_alg.h" 


void Power_Control_SendMessage_Can(void);

void Power_Control_SendMessage_Uart(void);

extern float Cap_Max_Voltage;
#endif

