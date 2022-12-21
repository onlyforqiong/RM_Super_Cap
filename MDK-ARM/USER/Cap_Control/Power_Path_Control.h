#ifndef __POWER_PATH_H
#define __POWER_PATH_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define CAP_POWERPATH_PORT        		GPIOC
#define CAP_POWERPATH_PIN         		GPIO_PIN_15
#define JDUGE_POWERPATH_PORT      		GPIOC
#define JDUGE_POWERPATH_PIN       		GPIO_PIN_14

#define BOOST_POWERPATH_PORT     		GPIOB
#define BOOST_POWERPATH_PIN       		GPIO_PIN_5
#define BACKFLUSH_POWERPATH_PORT        GPIOB
#define BACKFLUSH_POWERPATH_PIN         GPIO_PIN_6

void Cap_Control_Stop_Cap_PowerPath(void) ;
void Cap_Control_Stop_Out_PowerPath(void) ;
void Cap_Control_Change_Cap_PowerPath(void);
void Cap_Control_Change_Jduge_PowerPath(void);
void Cap_Control_Change_Boost_PowerPath(void);
void Cap_Control_Change_Backflush_PowerPath(void);


#endif



