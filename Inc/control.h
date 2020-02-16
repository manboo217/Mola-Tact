/*
 * control.h
 *
 *  Created on: May 1, 2019
 *      Author: manboo
 */

#include "drive.h"
#include "params.h"
#include "imu.h"


#ifndef CONTROL_H_
#define CONTROL_H_

typedef struct{
	float previous_error;
	float sum;
}PID_info;

#ifdef MAIN_C_
	float omega_z[2];
	float machine_angle[2];
	float eps_angle_z[2];
	float tar_angle_z;
	float sum_eps_angle_z;
	float pwm_out;
	PID_info trans_info;
	PID_info rot_info;
	float side_wall_control_value;
	float side_wall_control_l;
	float side_wall_control_r;
	float front_wall_control_value;
#else
	extern float omega_z[2];
	extern float machine_angle[2];
	extern float eps_angle_z[2];
	extern float tar_angle_z;
	extern float sum_eps_angle_z;
	extern float pwm_out;
	extern PID_info trans_info;
	extern PID_info rot_info;
	extern float side_wall_control_value;
	extern float side_wall_control_l;
	extern float side_wall_control_r;

	extern float front_wall_control_value;

#endif

void Set_PID_Params(PID_Gain*, float, float, float);//OK
void Set_Search_Params(motion*, float, float);
void Set_Trapezoid_Params(trapezoid*, float, float, float, float, float);//OK
void Acceleration_Control(motion*, trapezoid*);//OK
void Side_Wall_Control(void);
void Front_Wall_Control(void);
void Calculate_Target(motion*);//OK
void integralDistance(float*, float*);//OK
void PID_Control(motion*, motion*, PID_info* , PID_Gain*, duty*, trapezoid*,uint8_t); //OK
void Wall_Control(void);
#endif /* CONTROL_H_ */
