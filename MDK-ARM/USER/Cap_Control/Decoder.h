/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DECODER_H
#define __DECODER_H

#ifdef __cplusplus
extern "C"
{
#endif
#include "filter_alg.h"
#include "fdcan_util.h"
#include "Queue_Str.h"
#define RES_UP    510 //��һ����510 �ڶ����ĵ�910
#define RES_DOWN   22  //��һ����22   �ڶ����ĵ�39

   




extern float    Adc_Vcc;        //��λΪV
extern float    Adc_Cap; 
extern float    Adc_Current_In; //��λΪA
extern float    Adc_Current_Motor; 
extern float    Adc_Motor;
extern float    Motor_Power;
extern float    Adc_Temp;
extern float    Offset_Array[10];
extern float    CurrentIn_Offset;
extern float    CurrentMotor_Offset;
extern float    Adc_Vcc_Offset;
extern float    Adc_Cap_Offset;
extern float    Adc_Motor_Offset;
extern float    Adc_current_motor_lowpass ;

extern float    Adc_Source; //��λΪA
extern float    Adc_Boost;
extern Queue_Str_Typedef  motor_power_queue;



void Power_Switch_Decoder(void);
void Cap_Buck_Decoder(void);
void Cap_Boost_Decoder(void);
float Buck_Voltage_Decoder(float exp_val);

void Cap_Control_Can_Decoder(void);
void Boost_CommunicatorDecoder(uint8_t* rxdata);



#endif
