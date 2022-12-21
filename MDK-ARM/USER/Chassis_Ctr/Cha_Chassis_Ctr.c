#include"Cha_Chassis_Ctr.h"
#include"motor_Periph.h"
#include"remote_Periph.h"
#include"tim.h"
#include"cmsis_os.h"


int16_t x_value = 0, y_value = 0, row_value = 0;

void Chassis_Task(void const * argument) {
    
    for(;;) {
        Cha_ChassisControl();
        Cha_ChassisSendMessage();
        osDelay(1);        
    }
}

void Cha_ChassisInit(void) {

    Uart_DMA_Init(&const_remote_uarthander);
    Uart_ReceiveDMA(&const_remote_uarthander, Remote_rxdata, 54);
    Can_IntFilterAndStart(&const_chassismotor_canhander);
    Motor_MotorGroupInit(&Motor_chassisMotors, 4, Motor_CANMOTOR, &const_chassismotor_canhander, &htim6);
}

void Cha_ChassisControl(void) {

    Cha_McNhamWheelDecoder();
    Motor_MotorGroupsPID_Clac(&Motor_chassisMotors);
}

void Cha_ChassisSendMessage(void) {

    Motor_SendMessage(&Motor_chassisMotors);
}

void Cha_McNhamWheelDecoder(void) {

    int16_t x_value_temp = 0, y_value_temp = 0, row_value_temp = 0;
    x_value_temp = Remote_remotectr_data.Remote_remote.ch0;
    y_value_temp = Remote_remotectr_data.Remote_remote.ch1;
    row_value_temp = Remote_remotectr_data.Remote_remote.ch2;
    x_value = x_value_temp; 
    y_value = y_value_temp;
    row_value = row_value_temp;

    Motor_chassisMotors.motor_motorhandler[0].motor_pidhander.ref = 10 * (y_value+x_value+row_value);
    Motor_chassisMotors.motor_motorhandler[1].motor_pidhander.ref = 10 * (y_value-x_value+row_value);
    Motor_chassisMotors.motor_motorhandler[2].motor_pidhander.ref = 10 * (-1) * (y_value-x_value-row_value);
    Motor_chassisMotors.motor_motorhandler[3].motor_pidhander.ref = 10 * (-1) * (y_value+x_value-row_value);
}



