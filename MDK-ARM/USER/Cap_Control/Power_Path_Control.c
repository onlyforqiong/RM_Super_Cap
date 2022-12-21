#include"Power_Path_Control.h"
#include "Cap_Control.h"

void Cap_Control_Change_Cap_PowerPath(void){

	HAL_GPIO_WritePin(CAP_POWERPATH_PORT,CAP_POWERPATH_PIN,GPIO_PIN_SET);
	HAL_GPIO_WritePin(JDUGE_POWERPATH_PORT,JDUGE_POWERPATH_PIN,GPIO_PIN_RESET);
	Cap_Control_State.Cap_Switch_State = CapSwitch_State;
}


void Cap_Control_Change_Jduge_PowerPath(void){

	HAL_GPIO_WritePin(CAP_POWERPATH_PORT,CAP_POWERPATH_PIN,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(JDUGE_POWERPATH_PORT,JDUGE_POWERPATH_PIN,GPIO_PIN_SET);
	Cap_Control_State.Cap_Switch_State = Judge_State;
}

void Cap_Control_Change_Boost_PowerPath(void){

	HAL_GPIO_WritePin(BOOST_POWERPATH_PORT,BOOST_POWERPATH_PIN,GPIO_PIN_SET);
	HAL_GPIO_WritePin(BACKFLUSH_POWERPATH_PORT,BACKFLUSH_POWERPATH_PIN,GPIO_PIN_RESET);
	Cap_Control_State.Back_Flush_Switch_State = Boost_State;
}


void Cap_Control_Change_Backflush_PowerPath(void){

	HAL_GPIO_WritePin(BOOST_POWERPATH_PORT,BOOST_POWERPATH_PIN,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(BACKFLUSH_POWERPATH_PORT,BACKFLUSH_POWERPATH_PIN,GPIO_PIN_SET);
	Cap_Control_State.Back_Flush_Switch_State =  Back_Flush_State;
}

void Cap_Control_Stop_Cap_PowerPath(void) {
	
	HAL_GPIO_WritePin(CAP_POWERPATH_PORT,CAP_POWERPATH_PIN,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(JDUGE_POWERPATH_PORT,JDUGE_POWERPATH_PIN,GPIO_PIN_RESET);
}

void Cap_Control_Stop_Out_PowerPath(void) {
	
	HAL_GPIO_WritePin(BOOST_POWERPATH_PORT,BOOST_POWERPATH_PIN,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(BACKFLUSH_POWERPATH_PORT,BACKFLUSH_POWERPATH_PIN,GPIO_PIN_RESET);
}


