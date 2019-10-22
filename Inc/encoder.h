/*
 * encoder.h
 *
 *  Created on: May 11, 2019
 *      Author: cinqu
 */

#ifndef ENCODER_H_
#define ENCODER_H_

typedef struct {
  int16_t left;
  int16_t right;
}t_enc_value;

#ifdef MAIN_C_
	t_enc_value enc_buff;
	float delta_angle_l;
	float delta_angle_r;
	float delta_distance_l;
	float delta_distance_r;

#else
	extern t_enc_value enc_buff;
	extern float delta_angle_l;
	extern float delta_angle_r;
	extern float delta_distance_l;
	extern float delta_distance_r;
#endif

void Encoder_Init();
void Encoder_Start();
void Encoder_Update();
void Calculate_Velocity(t_enc_value*, motion*, motion*, motion*, motion*);
void Calculate_Tire_Angle();
void Calculate_Machine_Distance();
int8_t mode_change();


#endif /* ENCODER_H_ */
