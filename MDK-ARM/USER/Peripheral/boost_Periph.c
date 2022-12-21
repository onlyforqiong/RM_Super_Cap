#include"boost_Periph.h"
#include"cmsis_os.h"
#include"Cap_Control.h"

Boost_Boostdata Boost_BoostDataHandler = {25,&hfdcan1,0x297};


void Task_Boost_Control(void const * argument) {
	Boost_BoostSetValue(&Boost_BoostDataHandler);
	for(;;)
  {
		Boost_BoostSetValue(&Boost_BoostDataHandler);
    osDelay(10);
  }
}

void Boost_BoostSetValue(Boost_Boostdata * pdata)
{
	uint8_t txdata[8];
	txdata[0] = 0xE1;
	pdata->voltage = Cap_Control_State.Boost_val;
	txdata[1] = *(uint8_t*)(&pdata->voltage);
	txdata[2] = *((uint8_t*)(&pdata->voltage)+1);
	txdata[3] = *((uint8_t*)(&pdata->voltage)+2);
	txdata[4] = *((uint8_t*)(&pdata->voltage)+3);
	txdata[5] = 0;
	txdata[6] = Cap_Control_State.Boost_State;
	txdata[7] = 66;
	
	FDCAN_SendMessageWithOutBaudSwitch(pdata->phcan,txdata,FDCAN_DLC_BYTES_8,pdata->Can_SendID);
}




