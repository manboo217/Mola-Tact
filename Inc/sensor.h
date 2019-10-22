/*
 * sensor.h
 *
 *  Created on: 2019/05/18
 *      Author: manboo
 */

#ifndef SENSOR_H_
#define SENSOR_H_

#define ADC_BUFFER_LENGTH   ((uint32_t)  4)


#ifdef MAIN_C_
	uint16_t   ADCBuffer[ADC_BUFFER_LENGTH];
	uint32_t Sensor_Off_Data[ADC_BUFFER_LENGTH];
	uint32_t Sensor_On_Data[ADC_BUFFER_LENGTH];
	volatile int8_t adc_case;

#else
	extern uint16_t   ADCBuffer[ADC_BUFFER_LENGTH];
	extern uint32_t Sensor_Off_Data[ADC_BUFFER_LENGTH];
	extern uint32_t Sensor_On_Data[ADC_BUFFER_LENGTH];
	extern volatile int8_t adc_case;

#endif


void ADC_Start(void);
void ADC_Stop(void);
void Get_Sensor_Data(int16_t*);
void ADC_Convert_Check(void);
void get_base_info(void);
void update_sensor_data(void);
void Set_Threshold(sensor*, uint32_t, uint32_t, uint32_t, uint32_t);
void get_wall_info(void);
#endif /* SENSOR_H_ */
