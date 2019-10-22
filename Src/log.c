/*
 * log.c
 *
 *  Created on: Jun 14, 2019
 *      Author: manboo
 */


#include "global.h"


uint16_t log_counter = 0;

void Log_Saver(void)
{
	if(log_counter < MAX_LOG_NUMBER){
		log_buff.log1[log_counter] = log_data1;
	//	log_buff.log2[log_counter] = sensor_data.sideR;

//	log_buff.machine_velocity[log_counter] = rot_machine.distance;
//	log_buff.Rot_Target[log_counter] = rot_target.distance;

	}
	log_counter++;
}


void Log_Shower(void)
{
	int16_t log_number = 0;

	for(log_number=0; log_number<MAX_LOG_NUMBER; log_number ++)
	{
	//printf("%f\t%f\r\n",log_buff.log1[log_number],log_buff.log2[log_number]);
	printf("%f\r\n",log_buff.log1[log_number]);

	HAL_Delay(50);

	}
}
