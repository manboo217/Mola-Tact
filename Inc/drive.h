/*
 * drive.h
 *
 *  Created on: May 1, 2019
 *      Author: cinqu
 */


#ifndef DRIVE_H_
#define DRIVE_H_

#ifdef MAIN_C_
	uint8_t direction_num;
	volatile int32_t motion_counter;
#else
	extern uint8_t direction_num;
	extern volatile int32_t motion_counter;


#endif


void Motor_Init();

void Motor_Direction_Decide(duty*);
void Motor_PWM_OUT(duty*);
void Motor_Drive_Stop(void);
void Straight_Mode(float,float,float,float,float);
void Stay_Straight();//OK
void Stay_Start_Straight();
void Stay_Rotation();//OK
void Wait_Motion(int32_t);
void Set_Slalom_Params(slalom_params*, float,float,float,float);
void Rotation_Mode(float,float,float,float);
void Accel_Half_Section(float,float);
void Forward_One_Section(float);
void Adjust_Forward(float,float);
void Adjust_Back();
void Accel_Known_Section(float,float,float);
void Decel_Known_Section(float,float,float);
void Decel_Half_Section(float,float);
void PivotL90(float,float);
void PivotR90(float,float);
void PivotL180(float,float);
void Slalom_L(float);
void Slalom_R(float);
void SlalomR_V90(void);
void SlalomL_V90(void);



#endif /* DRIVE_H_ */
