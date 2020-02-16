/*
 * log.h
 *
 *  Created on: 2019/05/18
 *      Author: manboo
 */

#ifndef LOG_H_
#define LOG_H_

#define MAX_LOG_NUMBER 1000

typedef struct {
	float log1[MAX_LOG_NUMBER];
	float log2[MAX_LOG_NUMBER];
//	float machine_velocity[MAX_LOG_NUMBER];
//	float Rot_Target[MAX_LOG_NUMBER];
//	float target_distance[MAX_LOG_NUMBER];
//	float target_velocity[MAX_LOG_NUMBER];

}log_buffer;

#ifdef MAIN_C_
	log_buffer log_buff;
	float log_data1;
#else
//	extern uint16_t log_counter;
	extern log_buffer log_buff;
	extern float log_data1;

#endif

void Log_Saver();
void Log_Shower();


#endif /* LOG_H_ */
