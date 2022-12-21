#include "Cap_Control.h"
#include "Power_Control.h"
#include "math.h"
#include "fdcan.h"
#include "fdcan_util.h"
#include "uart_util.h"
#include "Decoder.h"


#define  Power_Control_CanPort hcan1

void Task_Power_Control(void const * argument) { 
	
  //Uart_DMA_Init(&huart2);
	Uart_DMA_Init(&huart3);
	//HAL_UART_Receive_IT(&huart2,Uart2_Board_Community_Buffer,8);
	//HAL_UART_Receive_IT(&huart3,Uart3_Board_Community_Buffer,8);
	//Uart_ReceiveDMA(&huart2,Uart2_Board_Community_Buffer,24);
	Uart_ReceiveDMA(&huart3,Uart3_Board_Community_Buffer,24);
	FDCAN_IntFilterAndStart(&hfdcan1);
	
	for(ever) {
		Cap_Control_State.System_sum_Power  =  Motor_Power + Cap_Charging_Power;
		Cap_Control_State.Cap_Remain_Energy = (int)((Adc_Cap/Cap_Max_Voltage)*(Adc_Cap/Cap_Max_Voltage)*100);
		Power_Control_SendMessage_Can();
		Power_Control_SendMessage_Uart(); 
		

		osDelay(2);

	}
	
}
void Task_Ref_Power_Limit(void const * argument) { // 所有和裁判系统有关的东西
	
	PID_InitPID(&Judge_BufferPowerPid);
	PID_SetPIDRef(&Cap_CapChargingPid,50);
	for(ever){
		if(Adc_Cap>=22.0f || Cap_Control_State.buff_power <= 20.0f) {
			Cap_Control_State.Cap_Must_En = 1;
		}
		else if(Adc_Cap<22.0f && Cap_Control_State.buff_power >= 40.0f){
			Cap_Control_State.Cap_Must_En = 0;
		}//这里是对缓冲功率的闭环
		if(Cap_Control_State.Communicate_State == Communicate_Online) {// 如果通信在线才有进入到pid闭环的必要
			PID_SetPIDRef(&Judge_BufferPowerPid,50);
			PID_SetPIDFdb(&Judge_BufferPowerPid,Cap_Control_State.buff_power);
			PID_ClacPID(&Judge_BufferPowerPid,&Judge_BufferPowerpidparam);
			Cap_Control_State.Cap_Control_Power_Max -= PID_GetPIDOutput(&Judge_BufferPowerPid);
			if(Cap_Control_State.Cap_Control_Power_Max > Cap_Control_State.Power_Limit + 10) {
				Cap_Control_State.Cap_Control_Power_Max = Cap_Control_State.Power_Limit + 10;
			}else if(Cap_Control_State.Cap_Control_Power_Max < Cap_Control_State.Power_Limit - 10) {
				Cap_Control_State.Cap_Control_Power_Max = Cap_Control_State.Power_Limit - 10;
			}
		}
//		else {
//{
//		Cap_Control_State.Cap_Control_Power_Max = 120;
//		Cap_Control_State.Motor_Safety_State = 1;
//		Cap_Control_State.Chassis_Power_State = 1;
//		}
		
////		if(Cap_Control_State.Cap_State == 0 && Motor_Power <= Cap_Control_State.Power_Limit - 5 \
////			&& Cap_Control_State.ref_power >= Cap_Control_State.Power_Limit){
////			Cap_State_Power_Offset -= (Cap_Control_State.ref_power - Cap_Control_State.Power_Limit)/2.0;
//		}
//		else if(Cap_Control_State.Cap_State == 1 && \
//			Cap_Control_State.ref_power >= Cap_Control_State.Power_Limit) {
//			Cap_State_Power_Offset -= (Cap_Control_State.ref_power - Cap_Control_State.Power_Limit)/2.0;
//		}	
//		if(Cap_Control_State.buff_power <= 20.0f) {
//			Cap_Control_State.Cap_Must_En = 1;
//		}
//		else if(Cap_Control_State.buff_power >= 40.0f) {
//			Cap_Control_State.Cap_Must_En = 0;
//		}
		osDelay(50);
		
	}
	
}

void Power_Control_SendMessage_Can(void)
{
	uint8_t txdata_1[8];
	txdata_1[0] = *((uint8_t*)&Motor_Power);
	txdata_1[1] = *((uint8_t*)&Motor_Power+1);
	txdata_1[2] = *((uint8_t*)&Motor_Power+2);
	txdata_1[3] = *((uint8_t*)&Motor_Power+3);
	txdata_1[4] = Cap_Control_State.Cap_Remain_Energy;

	FDCAN_SendMessageWithOutBaudSwitch(&hfdcan1,txdata_1,FDCAN_DLC_BYTES_8,0x299);
//Can_SendMessage(&Power_Control_CanPort,txdata_1,FDCAN_DLC_BYTES_8,0x99);
	uint8_t txdata[8];

	txdata[0] = *((uint8_t*)&Adc_Motor);
	txdata[1] = *((uint8_t*)&Adc_Motor+1);
	txdata[2] = *((uint8_t*)&Adc_Motor+2);
	txdata[3] = *((uint8_t*)&Adc_Motor+3);
	txdata[4] = *((uint8_t*)&Adc_Current_Motor);
	txdata[5] = *((uint8_t*)&Adc_Current_Motor+1);
	txdata[6] = *((uint8_t*)&Adc_Current_Motor+2);
	txdata[7] = *((uint8_t*)&Adc_Current_Motor+3);	

	FDCAN_SendMessageWithOutBaudSwitch(&hfdcan1,txdata,FDCAN_DLC_BYTES_8,0x298);
	//Can_SendMessage(&Power_Control_CanPort,0x100,txdata,8);
}

uint8_t txdata_uart[25];
uint8_t offline_temp = 0;
Communicate_StateEnum temp_State = 0;
void Power_Control_SendMessage_Uart(void)  {

	txdata_uart[0] = *((uint8_t*)&Adc_Motor);
	txdata_uart[1] = *((uint8_t*)&Adc_Motor+1);
	txdata_uart[2] = *((uint8_t*)&Adc_Motor+2);
	txdata_uart[3] = *((uint8_t*)&Adc_Motor+3);
//	txdata_uart[4] = *((uint8_t*)&Adc_Current_Motor);
//	txdata_uart[5] = *((uint8_t*)&Adc_Current_Motor+1);
//	txdata_uart[6] = *((uint8_t*)&Adc_Current_Motor+2);
//	txdata_uart[7] = *((uint8_t*)&Adc_Current_Motor+3);
	txdata_uart[4] = *((uint8_t*)&Motor_Power);
	txdata_uart[5] = *((uint8_t*)&Motor_Power+1);
	txdata_uart[6] = *((uint8_t*)&Motor_Power+2);
	txdata_uart[7] = *((uint8_t*)&Motor_Power+3);
	txdata_uart[8] = Cap_Control_State.Cap_Remain_Energy;
	if(Cap_Control_State.Communicate_State == Communicate_Offline && temp_State == Communicate_Online ) {
			offline_temp = 1;
	}
	
	temp_State = Cap_Control_State.Communicate_State;
	txdata_uart[9] = Cap_Control_State.Cap_State&0x01 |
								(Cap_Control_State.Boost_val>27.0f?1:0)<<1 |
								((uint8_t)Back_Flush_State << 2) |
								(Cap_Control_State.Cap_Enable_State <<  3)|
								(1<<5);//问就是傻逼hjy这一位不给1，舵电机会瞎几把乱动
  txdata_uart[10] = (uint8_t)Adc_Temp ;
	txdata_uart[11] = *((uint8_t*)&Adc_Source);
	txdata_uart[12] = *((uint8_t*)&Adc_Source+1);
	txdata_uart[13] = *((uint8_t*)&Adc_Source+2);
	txdata_uart[14] = *((uint8_t*)&Adc_Source+3);
	
	txdata_uart[15] = *((uint8_t*)&Adc_Boost);
	txdata_uart[16] = *((uint8_t*)&Adc_Boost+1);
	txdata_uart[17] = *((uint8_t*)&Adc_Boost+2);
	txdata_uart[18] = *((uint8_t*)&Adc_Boost+3);
	
	txdata_uart[19] = *((uint8_t*)&Adc_Cap);
	txdata_uart[20] = *((uint8_t*)&Adc_Cap+1);
	txdata_uart[21] = *((uint8_t*)&Adc_Cap+2);
	txdata_uart[22] = *((uint8_t*)&Adc_Cap+3);
//	txdata_uart[15] = (uint8_t)Cap_Charging_Power;
//	txdata_uart[16] = (uint8_t)Cap_Control_State.Cap_Control_Power_Max;
		
	

	//HAL_UART_Transmit_IT(&huart2,txdata_uart,11);
	HAL_UART_Transmit_IT(&huart3,txdata_uart,25);
}
uint32_t counter;
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
//	

//	if(huart == &huart2) {
//		Boost_CommunicatorDecoder(Uart2_Board_Community_Buffer);
//		HAL_UART_Receive_IT(&huart2,Uart2_Board_Community_Buffer,8);
//	}
//	else if(huart == &huart3) {
//		counter++;
//		Boost_CommunicatorDecoder(Uart3_Board_Community_Buffer);
//		HAL_UART_Receive_IT(&huart3,Uart3_Board_Community_Buffer,8);
//	}
//	
//	
//}

