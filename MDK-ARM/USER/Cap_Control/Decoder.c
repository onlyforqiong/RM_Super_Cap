#include"Decoder.h"
#include"adc_util.h"
#include "Cap_Control.h"
#include "Error_Handle.h"



float res_down = 22;
float res_up   = 520;


float    Adc_Vcc_Val[Adc_OverSampling_Times];  
float    Adc_Cap_Val[Adc_OverSampling_Times];
float    Adc_Current_In_Val[Adc_OverSampling_Times];

float    Adc_Current_Motor_Val[Adc_OverSampling_Times];
float    Adc_Motor_Val[Adc_OverSampling_Times];
float    Adc_Temp_Val[Adc_OverSampling_Times];


float    Adc_Source_Val[Adc_OverSampling_Times];
float    Adc_Boost_Val[Adc_OverSampling_Times];

float    Adc_Vcc;        //��λΪV
float    Adc_Cap;
float    Adc_Current_In;
float    Adc_Current_Motor; //��λΪA
float    Adc_Motor;
float    Adc_Source; //��λΪA
float    Adc_Boost;
float    Adc_Temp;

float    Adc_current_motor_lowpass = 0;

float    Cap_Charging_Power;
float    Cap_Charging_Power_Val;
float    Cap_Charging_Power_Avg = 0;;
float    Cap_Charging_Power_Avg_2;
float    current_last_val = 0;
float    Motor_Power;
float    Sum_Power;



float    Offset_Array[10] = {-0.07,0.08,1.004,1.00241,1.0,27.0,27.6};//0、输入电流校准，1、电机输出电流校准2、vcc校准，3、cap校准，4、motor校准
float    CurrentIn_Offset;                        //5、电容应该设定的电压值 6、电容满电的时候的电压值
float    CurrentMotor_Offset;
float    Adc_Vcc_Offset;
float    Adc_Cap_Offset;
float    Adc_Motor_Offset;


float    Adc_Out;
float    Adc_Current;
float    Adc_Ref;  

Queue_Str_Typedef  motor_power_queue;

lowpass_filterdata Adc_Current_Filter = {1 , 0 ,0};//�°���0.01 �ɰ���0.1
lowpass_filterdata Adc_Cap_Filter     = {1 , 0 ,0};
lowpass_filterdata Adc_Vcc_Filter     = {1 , 0 ,0};
lowpass_filterdata Adc_Current_Motor_Filter = {1 , 0 ,0};
lowpass_filterdata Adc_Motor_Filter     = {1 , 0 ,0};
lowpass_filterdata Dac_Current_Filter     = {1, 0 ,0};
lowpass_filterdata Adc_Source_Filter     = {1 , 0 ,0};
lowpass_filterdata Adc_Boost_Filter     = {1, 0 ,0};
kalman_filterdata Adc_Current_motor_kalman_filter = {0.02,0,0,0,0.0001,100,0,0};

//	float last_P;//上次估算协方差，初始化值为0.02
//	float now_P;//档次估算协方差，初始化值为0
//	float out;//滤波器输出，初始化值为0
//	float Kg;//卡尔曼增益，初始化为0
//	float Q;//过程噪声协方差，初始化值为0.001
//	float R;//观测噪声协方差，初始化值为0.543




void Cap_Buck_Decoder(void) {

	for (int j = 0; j < Adc_OverSampling_Times * ADC1_CHANNEL_NUM ; j++) {
        Adc_Sum_Val[j]  = (float)Adc_Val[j] / 65535.0f * 3.3f; //��һ���3.325���ڶ����3.292 ��cx Ӣ��3.309
				if(j%ADC1_CHANNEL_NUM == 0){ Adc_Boost_Val[j/ADC1_CHANNEL_NUM] = (Adc_Sum_Val[j])*11.0f;} //cx Ӣ�� 1.248
        else  if(j%ADC1_CHANNEL_NUM == 1){Adc_Source_Val[j/ADC1_CHANNEL_NUM] = Adc_Sum_Val[j]*11.0f;}
        else  if(j%ADC1_CHANNEL_NUM == 2){Adc_Current_Motor_Val[j/ADC1_CHANNEL_NUM] = (Adc_Sum_Val[j] - 1.80f) * 10.0f;}
				else  if(j%ADC1_CHANNEL_NUM == 3){Adc_Current_In_Val[j/ADC1_CHANNEL_NUM] = (Adc_Sum_Val[j] - 1.80f) * 10.0f;} 
				else  if(j%ADC1_CHANNEL_NUM == 4){Adc_Temp_Val[j/ADC1_CHANNEL_NUM] = ((Adc_Sum_Val[j] - 0.76f) / 0.0025f + 30.0f );}
 
    }
    float temp_Temp = 0,Source_Temp = 0 ,Boost_Temp = 0,Current_In_Temp = 0,Current_Motor_Temp = 0;
    for (int j = 0; j < Adc_OverSampling_Times; j++) {
				Source_Temp += Adc_Source_Val[j];
				Boost_Temp += Adc_Boost_Val[j];
				Current_In_Temp += Adc_Current_In_Val[j];
				Current_Motor_Temp        += Adc_Current_Motor_Val[j];
				temp_Temp += Adc_Temp_Val[j];
    }
	Adc_Temp =temp_Temp / Adc_OverSampling_Times;
	Adc_Boost = lowpass_filter(&Adc_Boost_Filter,Boost_Temp / Adc_OverSampling_Times);
	Adc_Source  = lowpass_filter(&Adc_Source_Filter,Source_Temp / Adc_OverSampling_Times);
  Adc_Current_In = lowpass_filter(&Adc_Current_Filter,((Current_In_Temp/Adc_OverSampling_Times) + CurrentIn_Offset));	
	Cap_Charging_Power = Adc_Vcc * Adc_Current_In;
	Adc_current_motor_lowpass = lowpass_filter(&Adc_Current_Motor_Filter,(Current_Motor_Temp/Adc_OverSampling_Times)+CurrentMotor_Offset); 
	Adc_Current_Motor = Adc_current_motor_lowpass;//kalman_filter(&Adc_Current_motor_kalman_filter,((Current_Motor_Temp/5.0f)+CurrentMotor_Offset));
	if(Adc_Current_Motor >= 1.0f) {
		Cap_Control_State.Motor_Heavy_Load_Tick = HAL_GetTick();
	}
	else if(Adc_Current_Motor<=0.5f && Adc_Current_Motor>=0.01f){
		Cap_Control_State.Motor_Light_Load_Tick = HAL_GetTick();
	}
		
}
void Power_Switch_Decoder() {

	for (int j = 0; j < Adc_OverSampling_Times * ADC2_CHANNEL_NUM; j++) {
        Adc_Sum_Val_2[j]  = (float)Adc_Val_2[j] / 65535.0f * 3.30f;//��һ���3.325���ڶ����3.292��cx Ӣ�� 3.309
				 if(j%ADC2_CHANNEL_NUM == 0) Adc_Motor_Val[j/ADC2_CHANNEL_NUM] = (Adc_Sum_Val_2[j])*11.0f; //cx Ӣ�� 1.798
        else  if(j%ADC2_CHANNEL_NUM == 1)Adc_Cap_Val[j/ADC2_CHANNEL_NUM] = Adc_Sum_Val_2[j]*11.0f;
				else  if(j%ADC2_CHANNEL_NUM == 2)Adc_Vcc_Val[j/ADC2_CHANNEL_NUM] = Adc_Sum_Val_2[j]*11.0f;
    }
    float Motor_Temp = 0 ;
		float Cap_Temp = 0 ,Vcc_Temp = 0;
    for (int j = 0; j < Adc_OverSampling_Times; j++) {
        Motor_Temp      += Adc_Motor_Val[j];
				Cap_Temp        += Adc_Cap_Val[j];
				Vcc_Temp        += Adc_Vcc_Val[j];
    }
    Adc_Motor = lowpass_filter(&Adc_Motor_Filter,Motor_Temp/Adc_OverSampling_Times*Adc_Motor_Offset);        //��λΪV
		Adc_Vcc = lowpass_filter(&Adc_Vcc_Filter,Vcc_Temp/Adc_OverSampling_Times*Adc_Vcc_Offset);        //��λΪV
    Adc_Cap = lowpass_filter(&Adc_Cap_Filter,Cap_Temp/Adc_OverSampling_Times*Adc_Cap_Offset);

		Motor_Power = Adc_Motor*Adc_Current_Motor ;
		//Motor_Power = Adc_Motor*Adc_Current_Motor_lowpass ;
}


float Buck_Voltage_Decoder(float exp_val) {
		
		if(exp_val < 6.5f) exp_val = 6.5f;
  	float AD8402_Value = 0;
  	AD8402_Value = (1.2f* (RES_DOWN + RES_UP) - exp_val * RES_DOWN) / (exp_val-1.20f) ;
  	return AD8402_Value;
}

void Cap_Control_Can_Decoder(void) {
	 
	  //CAN_RxHeaderTypeDef rxhander;
		FDCAN_RxHeaderTypeDef rxhander;
		uint8_t Can_RxData[8];
		uint32_t ret = HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &rxhander, Can_RxData);
		switch(rxhander.Identifier) {
    case 0x98:
		Boost_CommunicatorDecoder(Can_RxData);
    break;
  }
 }
 
 
 void Boost_CommunicatorDecoder(uint8_t* rxdata) {

   
	uint8_t Cap_State_temp = Cap_Control_State.Cap_State;
	if(rxdata[0]>>1 != 0x77) {
		return;
	}
	
	Cap_Control_State.Communicate_Last_Tick = HAL_GetTick();
	Cap_Control_State.Cap_State = (rxdata[0]&0x01)|Cap_Control_State.Cap_Must_En;//Cap_Must_En为更加高等级的逻辑
	if(Cap_State_temp == 0 && Cap_Control_State.Cap_State == 1) {
		Cap_Control_State.Cap_Disable_Tick = HAL_GetTick();
	}
	if(Cap_Control_State.Cap_State == 1 && Cap_State_temp == 0) {
		Cap_Control_State.Cap_Enable_Tick= HAL_GetTick();
	}
	
//	if(Cap_Control_State.Cap_State == 1) {
//		Cap_Control_State.Boost_State = 1;
//	}
//	else {
//		Cap_Control_State.Boost_State = 0;
//	}
	uint8_t boost_state_temp;
	boost_state_temp = rxdata[1]& 0x0001 ;
	Cap_Control_State.Chassis_Starting_State = ((rxdata[1]&0x04)>>2) ;
	Cap_Control_State.Chassis_Down_State = ((rxdata[1]& 0x02)>>1) ;
	Cap_Control_State.Motor_Safety_State = ((rxdata[1]&0x02)>>1);
	Cap_Control_State.Chassis_Power_State = ((rxdata[2]&0x04)>>2) || (Adc_Vcc>=20.0f);
	if(boost_state_temp == 1 && Adc_Cap>= 9.0f) Cap_Control_State.Boost_val = 27.5;
	else Cap_Control_State.Boost_val = 22;
	uint8_t temp_limit = Cap_Control_State.Power_Limit;
 	Cap_Control_State.Power_Limit = rxdata[2];	
	if(temp_limit!= Cap_Control_State.Power_Limit) {
		Cap_Control_State.Cap_Control_Power_Max = Cap_Control_State.Power_Limit;
	}
	Cap_Control_State.buff_power  = rxdata[3];
	Cap_Control_State.ref_power   = *((float*)(&(rxdata[4])));

 
}

