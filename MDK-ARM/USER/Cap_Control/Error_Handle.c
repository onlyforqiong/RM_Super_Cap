#include "Error_Handle.h"
#include "cmsis_os.h"
#include "Decoder.h"
#include "debug_BTlog.h"
void Task_Error_Detect(void const * argument) {
	
	Queue_Init(&motor_power_queue,100);
	
	for(ever){
		if(Adc_Motor<=5.0f && Adc_Current_Motor >= 1.0) {
			Short_Circuit_Error_Handle();
		}
		if(Cap_Control_State.Motor_Safety_State == 0){
			Motor_Error_Handle();	
		}
		if(Cap_Control_State.Communicate_Last_Tick <= HAL_GetTick() - 30) {
			Communicate_Offline_Error_Handle();
		}
		else {
			Cap_Control_State.Communicate_State = Communicate_Online;
		}
		if(Cap_Control_State.Motor_Energy_Sum >= (Cap_Control_State.buff_power - 20)&& Cap_Control_State.Cap_State != 1 &&
			Cap_Control_State.Communicate_State == Communicate_Online) {
				Cap_Control_State.Cap_Must_En = 1;
		}
		else if(Cap_Control_State.buff_power >= 40 && Cap_Control_State.LowPower_Push_State == LowPower_Notpush && 
			Cap_Control_State.Communicate_State == Communicate_Online) {
				if(Adc_Cap <= 21) {
				Cap_Control_State.Cap_Must_En = 0;
				}
		}
			
		
		Queue_Push(&motor_power_queue,Motor_Power);
		Cap_Control_State.Motor_Energy_Sum = motor_power_queue.avg * 0.1f;
		//BTlog_Send();
		osDelay(1);
	}
	
 }
void Task_BlueTooth(void const * argument) {
	BTlog_Init();
	for(ever) {
		if(Cap_Control_State.Cap_State == 1) {
			Cap_Control_State.System_sum_Power = Cap_Charging_Power + 3;
		}else {
			Cap_Control_State.System_sum_Power = Cap_Charging_Power + Motor_Power;
		}
		
		
		BTlog_Send();
		osDelay(1);
	}
	
}

void Short_Circuit_Error_Handle(void) {
		//Cap_Control_Change_Backflush_PowerPath();
	Cap_Control_Stop_Cap_PowerPath();
	Cap_Control_Stop_Out_PowerPath();
	while(1)
	{
	}
	
}

void Communicate_Offline_Error_Handle(void) {
	
	Cap_Control_State.Communicate_State = Communicate_Offline ; //ͨ�Ŷ��˽�����������ģʽ�������ò���ϵͳ������
	Cap_Control_State.Power_Limit =  50 - 5 ;
	Cap_Control_State.Cap_Control_Power_Max = Cap_Control_State.Power_Limit;
}


void RxError_Handler(void){
	
	while(1)
	{
		osDelay(1);
	}
	
}
 
void Motor_Error_Handle(void){
	if( Adc_Vcc >= 20.0f && Adc_Motor>=27.0f){
		Cap_Control_Change_Backflush_PowerPath();
		Cap_Control_Change_Jduge_PowerPath();
	}
}

