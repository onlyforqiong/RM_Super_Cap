#ifndef MTTP_ALG_H
#define MTTP_ALG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"
 #define float32 float
 #define int16 int16_t
 
//*********** Structure Definition ********//
typedef struct {
	float32  Ipv;
	float32  Vpv;
	float32  IpvH;
	float32  IpvL;
	float32  VpvH;
	float32  VpvL;
	float32  MaxVolt;
	float32  MinVolt;
	float32  Stepsize;
	float32  VmppOut;
	// internal variables
	float32  Cond;
	float32  IncCond;
	float32  DeltaV;
	float32  DeltaI;
	float32  VpvOld;
	float32  IpvOld;
	float32  StepFirst;
	int16 mppt_enable;
	int16 mppt_first;
} MPPT_INCC_F;
 
typedef MPPT_INCC_F mppt_incc_typedef;
extern mppt_incc_typedef mppt_incc_handle;
 
 #endif