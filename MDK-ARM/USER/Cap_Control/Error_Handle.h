/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ERROR_HANDLE_H
#define __ERROR_HANDLE_H

#ifdef __cplusplus
extern "C"
{
#endif
#include "filter_alg.h"
#include "fdcan_util.h"
#include "Cap_Control.h"


void RxError_Handler(void);
void Short_Circuit_Error_Handle(void);
void Motor_Error_Handle(void);
void Communicate_Offline_Error_Handle();


#endif
