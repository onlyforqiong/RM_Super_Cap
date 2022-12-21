#ifndef __CHA_CHASSIS_CTR_H
#define __CHA_CHASSIS_CTR_H

#ifdef __cplusplus
extern "C" {
#endif

#include"can_util.h"
#include"motor_Periph.h"
#include"remote_Periph.h"

void Chassis_Task(void const * argument);
void Cha_ChassisInit(void);
void Cha_ChassisControl(void);
void Cha_ChassisSendMessage(void);
void Cha_McNhamWheelDecoder(void);



#endif

