#include "Cap_Control.h"
#include "math.h"
#include "tim.h"
#include "AD8402_Periph.h"
#include "fdcan.h"
#include "fdcan_util.h"
#include "uart_util.h"
#include "boost_Periph.h"
#include "flash_util.h"
#include "Power_Control.h"
#include "cmsis_os.h"

//舵轮26

Cap_Control_Typedef Cap_Control_State = {0, 0, 0, 26, 40, 22, 100, 0, 0, 60, 0, 9.0f, 1, 0, 0, 0, 0, 0, 1, 1, 0, 0, 50};

float Cap_Max_Voltage = 27.7f;

float charging_offset = 5;
float Dac_Now = 0.1;

float backflush_current = -0.8f;
float backflush_voltage = 15.0f;

uint8_t counting_flag = 0;

float sum_charging_energy = 0;
float sum_motor_energy = 0;

uint8_t BackFlush_State;

uint16_t Cap_Control_Buck_Data;
uint8_t Max_En = 1;
float Disable_Limit = 0.0f;
uint8_t timing_num = 0;
uint8_t Uart2_Board_Community_Buffer[24];
uint8_t Uart3_Board_Community_Buffer[24];

float Cap_Charging_Power_Val_2 = 0; //其实一共有三个闭环，3790给我做的电流环，上面一层功率环，最上层是裁判系统的缓冲功率闭环
// PID_PIDparam_Typedef Cap_Chargingpidparam							 = {0.005f,0.0001f,0,1,10000} ;//旧板子
PID_PIDparam_Typedef Cap_Chargingpidparam = {0.0005f, 0.000f, 0.0, 1, 10000}; //新板子
PID_PIDparam_Typedef Judge_BufferPowerpidparam = {0.1, 0.2, 2, 1000};		  //最上层功率环的pid

uint8_t Ref_State = 0;
PID_PIDTypeDef Cap_CapChargingPid;
PID_PIDTypeDef Judge_BufferPowerPid;
float adding_param = 1.0;
float Dac_Min = 0.1;
float Dac_Max = 0.5;

float Dac_Vcc_Change = 5;

float Cap_Charging_MinEfficency = 0.8;

float lowpass_param = 1;
int32_t tick_deta;

float Cap_State_Power_Offset = -4;
float Jduge_State_Power_Offset = -4;

void Task_CapBuckControl(void const *argument)
{
	// Flash_WriteSingleAddress(Current_OffSet_Address,(uint32_t*)Offset_Array,40);
	// Flash_ReadData(Current_OffSet_Address,(uint32_t*)Offset_Array,10);
	//由于使用裁判系统的数据来校准，所以
	osDelay(10);
	CurrentIn_Offset = Offset_Array[0];
	CurrentMotor_Offset = Offset_Array[1];
	Adc_Vcc_Offset = Offset_Array[2];
	Adc_Cap_Offset = Offset_Array[3];
	Adc_Motor_Offset = Offset_Array[4];
	if (CAR_TYPE == DUOLUN)
	{
		Cap_Control_State.Cap_val = 25.5;
		Cap_Max_Voltage = 26.2;
	}
	else if (CAR_TYPE == MAILUN)
	{
		Cap_Control_State.Cap_val = 27;
		Cap_Max_Voltage = 27;
	}
	else if (CAR_TYPE == HERO)
	{
		Cap_Control_State.Cap_val = 27;
		Cap_Max_Voltage = 27.7;
	}
	Cap_Control_State.Motor_Safety_State = 1;
	Cap_Control_State.Chassis_Power_State = 1;

	// Cap_Control_State.Cap_val = Offset_Array[5];
	// Cap_Max_Voltage =  Offset_Array[6] ;
	Cap_Control_State.Cap_Control_Power_Max = Cap_Control_State.Power_Limit;
	Cap_Control_Change_Cap_PowerPath();

	AD8402_RESET();
	Soft_Start();

	CAP_Buck_Disable();
	Adc_Init();
	DAC_Init();
	DAC_SetVal(0.02f);

	//	osDelay(10);
	Cap_Control_SetVoltage(Cap_Control_State.Cap_val);
	while (Adc_Vcc <= 12.0f)
	{ // 让vcc高于12v才能开始工作
		Cap_Control_Change_Cap_PowerPath();
	}

	PID_InitPID(&Cap_CapChargingPid);
	PID_SetPIDRef(&Cap_CapChargingPid, 10);
	HAL_TIM_Base_Start_IT(&htim15); //刹车回冲
	HAL_TIM_Base_Start_IT(&htim16); //电容充电
	HAL_TIM_Base_Start(&htim2);
	// HAL_TIM_Base_Start_IT(&htim1);//修改电压
	for (ever)
	{
		Cap_Control_SetVoltage(Cap_Control_State.Cap_val);
		tick_deta = Cap_Control_State.Motor_Heavy_Load_Tick - Cap_Control_State.Motor_Light_Load_Tick;
		//		Cap_Control_Change_Cap_PowerPath();
		//		Cap_Control_Change_Backflush_PowerPath();
		if (Adc_Vcc <= 12.0f || Cap_Control_State.Chassis_Power_State == Chassis_Power_Off)
		{
			Cap_Control_Change_Cap_PowerPath();
			Cap_Control_Stop_Out_PowerPath();
			CAP_Buck_Disable();
			Cap_Control_State.Boost_State = 0;
			osDelay(1);
			continue;
		}
		CAP_Buck_Enable();
		// else if(Adc_Vcc>=14.00f&&Cap_Control_State.Cap_State == 0) {
		// 		Cap_Control_Change_Jduge_PowerPath();
		// 		Cap_Control_Change_Backflush_PowerPath();
		// }
		if (Adc_Cap <= 11.0f)
		{ //充分榨干电容的电
			Cap_Control_State.Cap_Enable_State = Cap_Disable;
			if (Motor_Power >= Cap_Control_State.Power_Limit + 5)
			{
				Cap_Control_State.Cap_Must_En = 1;
				Cap_Control_State.LowPower_Push_State = LowPower_Push;
			}
			else if (Motor_Power < Cap_Control_State.Power_Limit - 5)
			{
				// Cap_Control_Change_Jduge_PowerPath();
				Cap_Control_State.Cap_Must_En = 0;
				Cap_Control_State.LowPower_Push_State = LowPower_Notpush;
			}
			else if (Cap_Control_State.buff_power >= 30)
			{
				Cap_Control_State.Cap_Must_En = 0;
				Cap_Control_State.LowPower_Push_State = LowPower_Notpush;
			}
			if (Cap_Control_State.buff_power <= 20.0f || (Cap_Control_State.buff_power <= 40 && Motor_Power >= Cap_Control_State.Power_Limit + 20.0f))
			{
				Cap_Control_State.Cap_Must_En = 1;
				Cap_Control_State.LowPower_Push_State = LowPower_Push;
			}
		}
		else if (Adc_Cap >= 12.0f)
		{
			Cap_Control_State.Cap_Enable_State = Cap_Enable;
		}
		Cap_Control_State.Cap_State = Cap_Control_State.Cap_State | Cap_Control_State.Cap_Must_En;
		if (Cap_Control_State.Motor_Safety_State == 0 && Adc_Vcc >= 20.0f && Adc_Motor >= 27.0f)
		{
			Cap_Control_Change_Jduge_PowerPath();
			Cap_Control_Change_Backflush_PowerPath();
			osDelay(1000);
		}
		else
		{
			Cap_Control_PowerPath_Switch();
			osDelay(1);
		}
	}
}

void Task_Feed_Dog(void)
{
	// HAL_IWDG_Refresh(&hiwdg);
	for (ever)
	{
		//	HAL_IWDG_Refresh(&hiwdg);
		osDelay(10);
	}
}

void CAP_Buck_Disable(void)
{

	HAL_GPIO_WritePin(CAP_BUCK_SWITCH_PORT, CAP_BUCK_SWITCH_PIN, GPIO_PIN_RESET);
	Cap_Control_State.Buck_State = Buck_Dis;
}

void CAP_Buck_Enable(void)
{

	HAL_GPIO_WritePin(CAP_BUCK_SWITCH_PORT, CAP_BUCK_SWITCH_PIN, GPIO_PIN_SET);
	Cap_Control_State.Buck_State = Buck_En;
}

void Cap_Control_SetCurrent(float current_val)
{

	if (current_val >= 3.0f)
		current_val = 3.0f;
	else if (current_val <= 0.0f)
		current_val = 0.0f;
	DAC_SetVal(current_val);
}

void Cap_Control_SetVoltage(float vol)
{

	AD8402_Periph_Ohm[1] = Buck_Voltage_Decoder(vol);
	AD8402_Periph_Ohm[0] = 0.0f;
	AD8402_SetValue(AD8402_Periph_Ohm);
	// AD8402_Send_10Bits(Cap_Control_Buck_Data);
}

void Task_Ledshine(void const *argument)
{
	/* USER CODE BEGIN task_led_shine */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
	/* Infinite loop */
	for (;;)
	{

		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_15);
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_8);
		if (Cap_Control_State.Communicate_State == Communicate_Offline)
		{
			osDelay(50);
		}
		else if (Adc_Cap <= 12.0f)
		{
			osDelay(10);
		}

		else if (Adc_Cap >= 12.0f && Adc_Cap <= 26)
		{
			osDelay(500);
		}
		else if (Adc_Cap > 26)
		{
			osDelay(200);
		}
	}
	/* USER CODE END task_led_shine */
}

void Cap_Control_Backflush(void)
{

	// Cap_Control_State.Backflush_Tick = HAL_GetTick();
	if (Adc_Cap >= 23.0f)
	{
		if (Adc_current_motor_lowpass >= backflush_current)
		{
			if (BackFlush_State == 1)
			{
				//				Cap_Control_Change_Jduge_PowerPath();
				if (Cap_Control_State.Cap_State == 1)
				{
					Cap_Control_Change_Cap_PowerPath();
					Cap_Control_Change_Backflush_PowerPath();
				}
				else
				{
					Cap_Control_Change_Jduge_PowerPath();
				}
				BackFlush_State = 0;
			}
		}
		else
		{
			if (BackFlush_State == 0)
			{
				Cap_Control_State.Backflush_Tick = HAL_GetTick();
				if (Cap_Control_State.Backflush_Tick - Cap_Control_State.Cap_Disable_Tick >= 100)
				{
					BackFlush_State = 1;
				}
				Cap_Control_Change_Backflush_PowerPath();
				Cap_Control_Change_Cap_PowerPath();
			}
			else if (BackFlush_State == 1)
			{
				if (Adc_Motor <= backflush_voltage)
				{
					if (Cap_Control_State.Cap_State == 1)
					{
						Cap_Control_Change_Backflush_PowerPath();
						Cap_Control_Change_Cap_PowerPath();
					}
					else
					{
						Cap_Control_Change_Backflush_PowerPath();
						Cap_Control_Change_Jduge_PowerPath();
					}
				}
				else
				{
					Cap_Control_Change_Backflush_PowerPath();
					Cap_Control_Change_Cap_PowerPath();
				}
			}
		}
		return;
	}
	if (Adc_current_motor_lowpass >= backflush_current)
	{
		if (BackFlush_State == 1)
		{ // 若电流大于给定的刹车反冲电流阈值
			if (Cap_Control_State.Cap_State == 1)
			{
				Cap_Control_Change_Cap_PowerPath();
			}
			else
			{
				Cap_Control_Change_Jduge_PowerPath();
			}
			if (Cap_Control_State.Cap_State == 1 && Cap_Control_State.Boost_State == 1)
			{
				Cap_Control_Change_Boost_PowerPath();
			}
			else
			{
				Cap_Control_Change_Backflush_PowerPath();
			}
			BackFlush_State = 0;
		}
	}
	else
	{
		if (BackFlush_State == 0)
		{
			Cap_Control_State.Backflush_Tick = HAL_GetTick();
			if (Cap_Control_State.Backflush_Tick - Cap_Control_State.Cap_Disable_Tick >= 50 && Cap_Control_State.Motor_Heavy_Load_Tick - Cap_Control_State.Motor_Light_Load_Tick >= 600)
			{
				BackFlush_State = 1;
				Cap_Control_Change_Backflush_PowerPath();
				Cap_Control_Change_Cap_PowerPath();
			}
		}
		else if (BackFlush_State == 1)
		{
			if (Adc_Motor <= backflush_voltage)
			{
				Cap_Control_Change_Jduge_PowerPath();
				if (Cap_Control_State.Cap_State == 1)
				{
					Cap_Control_Change_Boost_PowerPath();
					Cap_Control_Change_Cap_PowerPath();
				}
				else
				{
					Cap_Control_Change_Backflush_PowerPath();
					Cap_Control_Change_Cap_PowerPath();
				}
			}
			else
			{
				Cap_Control_Change_Backflush_PowerPath();
				Cap_Control_Change_Cap_PowerPath();
			}
		}
	}
}

void Buck_Control_Pid_Clac(void)
{
	float ref;

	if (Cap_Control_State.Cap_State == 1)
	{
		ref = Cap_Control_State.Cap_Control_Power_Max;
		//			if(timing_num >= 2 && timing_num<=3) ref+=20;
	}
	else
	{
		float temp_motor_power = Motor_Power >= 0 ? Motor_Power : 0;
		ref = Cap_Control_State.Cap_Control_Power_Max - temp_motor_power;
	}

	Cap_Control_State.Cap_Charging_Power_Ref = (ref >= Disable_Limit ? ref : Disable_Limit);
	if (Cap_Control_State.Cap_Charging_Power_Ref <= Disable_Limit)
	{
		CAP_Buck_Disable();
		return;
	}
	else
	{
		CAP_Buck_Enable();
	}

	PID_SetPIDRef(&Cap_CapChargingPid, Cap_Control_State.Cap_Charging_Power_Ref);
	PID_SetPIDFdb(&Cap_CapChargingPid, Cap_Charging_Power);
	PID_ClacPID(&Cap_CapChargingPid, &Cap_Chargingpidparam);
	Dac_Now += Cap_CapChargingPid.output;
	if (Dac_Now > 3.0f)
		Dac_Now = 3.0f;
	else if (Dac_Now < 0.0f)
		Dac_Now = 0.0f;
	Cap_Control_SetCurrent(Dac_Now);
}

void Soft_Start(void)
{

	Cap_Control_Stop_Cap_PowerPath();
	Cap_Control_Stop_Out_PowerPath();
	osDelay(20);
	Cap_Control_Change_Jduge_PowerPath();
	Cap_Control_Change_Backflush_PowerPath();
	osDelay(2000);
	//	Cap_Control_Change_Boost_PowerPath();
	//	osDelay(100);
	Cap_Control_State.Boost_State = 0;
}

void Cap_Control_PowerPath_Switch(void)
{

	if (BackFlush_State == 0)
	{

		Cap_Control_State.Cap_State = Cap_Control_State.Cap_State | Cap_Control_State.Cap_Must_En;
		if (Adc_Motor >= 32.0f)
		{
			Cap_Control_Change_Backflush_PowerPath();
			Cap_Control_Change_Cap_PowerPath();
			Cap_Control_State.voltage_max_state = 1;
			return;
		}
		else if (Cap_Control_State.voltage_max_state == 1 && Adc_Motor >= 30.0f)
		{
			Cap_Control_Change_Backflush_PowerPath();
			Cap_Control_Change_Cap_PowerPath();
			Cap_Control_State.voltage_max_state = 1;
			return;
		}
		else
		{
			Cap_Control_State.voltage_max_state = 0;
		}

		if ((Cap_Control_State.Cap_State == 1 && Cap_Control_State.Cap_Enable_State == Cap_Enable) || Cap_Control_State.Cap_Must_En == 1)
		{

			if (Adc_Cap >= 23.0f)
			{
				Cap_Control_Change_Cap_PowerPath();
				// Cap_Control_Change_Backflush_PowerPath();
				if (Cap_Control_State.Boost_val >= 27)
				{
					Cap_Control_State.Boost_State = 1;
					Cap_Control_Change_Boost_PowerPath();
				}
				else
				{
					Cap_Control_State.Boost_State = 0;
					Cap_Control_Change_Backflush_PowerPath();
				}
			}
			else if (Adc_Cap < 23.0f && Adc_Cap >= 22.0f && Cap_Control_State.Boost_State == 0)
			{
				Cap_Control_Change_Cap_PowerPath();
				if (Cap_Control_State.Boost_val >= 27)
				{
					Cap_Control_State.Boost_State = 1;
					Cap_Control_Change_Boost_PowerPath();
				}
				else
				{
					Cap_Control_State.Boost_State = 0;
					Cap_Control_Change_Backflush_PowerPath();
				}
			}
			else if (Adc_Cap < 23.0f && Adc_Cap >= 22.0f && Cap_Control_State.Boost_State == 1)
			{
				Boost_BoostSetValue(&Boost_BoostDataHandler); //先给boost模块发升压的数据，马上让boost升压
				Cap_Control_Change_Boost_PowerPath();
				Cap_Control_Change_Cap_PowerPath();
				Cap_Control_State.Boost_State = 1;
			}
			else if (Adc_Cap < 22.0f)
			{
				Boost_BoostSetValue(&Boost_BoostDataHandler); //先给boost模块发升压的数据，马上让boost升压
				Cap_Control_Change_Boost_PowerPath();
				uint32_t temp_tick = HAL_GetTick();
				//					if(		  temp_tick	- Cap_Control_State.Cap_Enable_Tick <=60){
				//						Cap_Control_Change_Jduge_PowerPath();
				//					}
				//					else {
				//						Cap_Control_Change_Cap_PowerPath();
				//					}
				if (temp_tick - Cap_Control_State.Motor_Light_Load_Tick <= 20 && Cap_Control_State.Motor_Light_Load_Tick <= Cap_Control_State.Motor_Heavy_Load_Tick)
				{
					Cap_Control_Change_Jduge_PowerPath();
				}
				else
				{
					Cap_Control_Change_Cap_PowerPath();
				}
				Cap_Control_State.Boost_State = 1;
			}
		}
		else if (Adc_Cap > 11.0f && Cap_Control_State.Cap_Enable_State == Cap_Disable)
		{
			Cap_Control_State.Cap_Enable_State = Cap_Enable;
		}
		else
		{
			Cap_Control_Change_Jduge_PowerPath();
			Cap_Control_Change_Backflush_PowerPath();
		}
	}
}
