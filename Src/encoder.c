/*
 * encoder.c
 *
 *  Created on: May 1, 2019
 *      Author: manboo
 */



#include "global.h"
#include "stm32f4xx_hal_tim.h"



void Encoder_Init(void)
{
	TIM1->CNT = 0;
	TIM8->CNT = 0;

	delta_angle_r = 0.0f;
	delta_distance_l = 0.0f;
	delta_distance_r = 0.0f;

}

void Encoder_Start(void){
	  HAL_TIM_Encoder_Start( &htim1, TIM_CHANNEL_ALL );
	  HAL_TIM_Encoder_Start( &htim8, TIM_CHANNEL_ALL );
	  TIM1->CNT = 0;
	  TIM8->CNT = 0;
}

void Encoder_Update( t_enc_value *enc )
{
	(enc->right) = -TIM1->CNT;
	(enc->left)  = TIM8->CNT;

	  TIM1->CNT = 0;
	  TIM8->CNT = 0;

}


void Calculate_Velocity(t_enc_value *enc, motion *angle_l, motion *angle_r,
		motion *trans_l, motion *trans_r){
	//angular velocity of motor shaft

	float left_omega = 0.0f, right_omega = 0.0f;

	left_omega =  (float) (enc->left) * 360  / RESOLUTION / TIM4_INTERVAL;
	right_omega = (float) (enc->right) * 360 / RESOLUTION / TIM4_INTERVAL;
	//角速度ω計算
	angle_l->velocity = left_omega;
	angle_r->velocity = right_omega;

	//angular velocity of tire
	angle_l->velocity /= REDUCTION_RADIO;
	angle_r->velocity /= REDUCTION_RADIO;

	//velocity of tire v= r*ω[rad/s]

	trans_l->velocity = angle_l->velocity * (TIRE_DIAMETER / 2) /180 * M_PI;
	trans_r->velocity = angle_r->velocity * (TIRE_DIAMETER / 2) /180 * M_PI;

	trans_machine.velocity = (trans_l->velocity + trans_r->velocity) / 2.0;
	//4096*4=16384でピニオンが1周．　減速比11:40より16384*40/11~=59578でタイヤが1周．
	//　タイヤ径Diameterがわかればタイヤが一周して進む距離がわかる
}

void Calculate_Tire_Angle(void){
	delta_angle_l = angle_l.velocity * TIM4_INTERVAL;
	angle_l.distance += delta_angle_l;

	delta_angle_r = angle_r.velocity * TIM4_INTERVAL;
	angle_r.distance += delta_angle_r;
}

void Calculate_Machine_Distance(void){
	delta_distance_l = trans_l.velocity  * TIM4_INTERVAL;
	trans_l.distance += delta_distance_l;

	delta_distance_r = trans_r.velocity * TIM4_INTERVAL;
	trans_r.distance += delta_distance_r;
	trans_machine.distance = (trans_l.distance + trans_r.distance)/2.0;
}


int8_t mode_change()
{
	int8_t mode = 0;
	float angle_offset = 0.0f;

	angle_offset = angle_r.distance;
	Front_LED_Light(0, 0, 0);

	while(1){
	mode = (int8_t)((angle_r.distance - angle_offset) / 360);
//	printf("%f,%f,%d,",angle_r.distance,angle_offset,mode);

	mode %= 16;

	if(mode > 15) mode = 0;
	if(mode < 0) mode = 15;
		Front_LED_Light(mode & 0b100, mode & 0b010, mode & 0b001);
	//	printf("%d\r\n",mode);
		   if( gyro_raw.accel_z >15.0f){
			   return mode;
		   }
	}
}
