#ifndef __CAP_CONTROL_H
#define __CAP_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include"main.h"
#include"dac_util.h"
#include"adc_util.h"
#include"pid_alg.h" 
#include "Decoder.h"
#include "Power_Path_Control.h"
//#include "iwdg.h"
#include "configue.h"


#define CAP_BUCK_SWITCH_PORT      		GPIOB
#define CAP_BUCK_SWITCH_PIN       		GPIO_PIN_9
#define CAP_KEY_PORT              		GPIOA
#define CAP_KEY_PIN               		GPIO_PIN_15



#define CAP_OUT_RES      0.004f
#define CAP_IN_RES       0.002f
#define ever ;;
#define PID_WIDE_0_13W      0 
#define PID_WIDE_14_28W		1
#define PID_WIDE_29_W       0
#define PID




void CAP_Buck_Disable(void);
void CAP_Buck_Enable(void);
void CAP_Buck_GetCapVoltage(void);
void CAP_Switch(void) ;
void CAP_Buck_PowerControl(void) ;


void Cap_Charging_Control(void);
void Cap_Control_SetVoltage(float vol);
float Buck_Voltage_Decoder(float exp_val);

void Cap_Control_SetCurrent(float current_val);
void Cap_Control_Backflush(void) ;
void Cap_Control_Can_Decoder(void) ;
void Boost_RxCallback(uint8_t * pdata);
void Buck_Control_Pid_Clac(void);
void Short_Circuit_Error_Handle(void);
void Cap_Control_PowerPath_Switch(void);
void Soft_Start(void) ;

extern FunctionalState Cap_State;
extern float Power_Limit;


extern PID_PIDparam_Typedef Cap_Chargingpidparam_low_and_high ;
extern PID_PIDTypeDef Cap_CapChargingPid;
extern uint32_t add_num;
extern uint16_t Cap_Control_Buck_Data;

extern float    Cap_Charging_Power;


extern uint8_t counting_flag ;

extern float  sum_charging_energy;
extern float  sum_motor_energy;

extern uint8_t timing_num;
extern float Cap_State_Power_Offset;
extern float Jduge_State_Power_Offset;
extern uint8_t  BackFlush_State;



typedef enum {
	Back_Flush_State = 0,
	Boost_State   = 1
}Back_Flush_Switch_StateEnum;



typedef enum {
	Judge_State = 0,
	CapSwitch_State   = 1
}Cap_Switch_StateEnum;


typedef enum { 
	Cap_Enable = 1,
	Cap_Disable = 0
}Cap_Enable_StateEnum;

typedef enum { 
	Chassis_Starting = 1,
	Chassis_Not_Starting = 1
}Cap_Control_Chassis_Starting_StateEnum;

typedef enum {
	Motor_Safe = 1,
	Motor_Unsafe = 0
}Motor_Safety_StateEnum;//���̵����û�е���

typedef enum {
	Chassis_Power_On = 1,
	Chassis_Power_Off = 0
}Chassis_Power_StateEnum;//���̵�����û�е��ж�

typedef enum {
	Communicate_Offline = 0,
	Communicate_Online = 1
}Communicate_StateEnum;//�͵������ص�ͨѶ״̬


typedef enum { 
	Chassis_Down = 1,
	Chassis_Not_Down = 1
}Cap_Control_Chassis_Down_StateEnum; //

typedef enum {
	 Buck_En = 1,
	 Buck_Dis = 0
}Buck_Enable_StateEnum;

typedef enum {
	 LowPower_Push = 1,
	 LowPower_Notpush= 0
}Cap_LowPower_Push_StateEnum;

typedef struct {
	uint8_t Cap_State;
	uint8_t Boost_State;
	uint8_t Cap_Must_En;
	float   Cap_val;
	float   Cap_Charging_Power_Ref;
	float   Boost_val;
	
	float   Power_Limit;
	
	Back_Flush_Switch_StateEnum Back_Flush_Switch_State;
	Cap_Switch_StateEnum        Cap_Switch_State;
	uint8_t buff_power;
	float   ref_power ;
	float   Cap_Ban_Voltage;
	Cap_Enable_StateEnum        Cap_Enable_State;
	Cap_Control_Chassis_Starting_StateEnum  Chassis_Starting_State;
  Cap_Control_Chassis_Down_StateEnum  Chassis_Down_State;
	uint32_t  Cap_Disable_Tick;
	uint32_t  Backflush_Tick;
	uint32_t  Cap_Enable_Tick;
	uint32_t  Motor_Heavy_Load_Tick;
	uint32_t  Motor_Light_Load_Tick;
	uint8_t   Cap_Remain_Energy;
	float     System_sum_Power;
	uint8_t   Output_En;
	Motor_Safety_StateEnum Motor_Safety_State;
	Chassis_Power_StateEnum  Chassis_Power_State;
	uint32_t  Communicate_Last_Tick;
	Communicate_StateEnum Communicate_State;
	float Cap_Control_Power_Max;//实际闭环的功率大小
	Buck_Enable_StateEnum Buck_State;
	float Motor_Energy_Sum;//100ms的能量总和
	Cap_LowPower_Push_StateEnum LowPower_Push_State;
	uint8_t voltage_max_state;
} Cap_Control_Typedef;

extern Cap_Control_Typedef Cap_Control_State;
extern uint8_t Uart2_Board_Community_Buffer[24];
extern uint8_t Uart3_Board_Community_Buffer[24];
extern PID_PIDTypeDef Judge_BufferPowerPid;
extern PID_PIDparam_Typedef Judge_BufferPowerpidparam  ;
#endif

