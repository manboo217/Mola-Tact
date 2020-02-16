/*
 * interrupt.c
 *
 *  Created on: May 01, 2019
 *      Author: manboo
 */

#include "global.h"
#include "stm32f4xx_hal_tim.h"

extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;

extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;

uint8_t log_cnt = 0;

static int8_t failsafe_counter;
static float gyro_change_value = 0.0f;


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	// Get Gyro & Encoder Info
 if(htim == &htim5){
	ADC_Convert_Check();//adc_case
 }
 if(htim == &htim4){
	motion_counter++;

	ICM20602_DataUpdate();//ジャイロ更新
	Encoder_Update(&enc_buff);//エンコーダ更新

	if(gyro_calib_flag){
			ICM20602_Calibration();
	}else {
		Calculate_Velocity(&enc_buff, &angle_l, &angle_r, &trans_l, &trans_r);
		Calculate_Machine_Rotation();
		gyro_change_value = omega_z_buff[1] -omega_z_buff[0];

		if(!MF.FLAG.MT_CTRL){
		Calculate_Tire_Angle();
		Calculate_Machine_Distance();
		}
	}

	  if ( MF.FLAG.MT_CTRL == 1 && MF.FLAG.FAILSAFE == 0){
		  /***** FailSafe ****/
	    if ( trans_trapezoid_params.reverse_flag == 0
	    		&& trans_target.velocity >= 300
	    		&& ( trans_l.velocity < 100.0f || trans_r.velocity < 100.0f
	    		|| gyro_raw.accel_x >80.0f || gyro_raw.accel_x < -80.0f)){	//	要見直し

	    	failsafe_counter++;

	    	if ( failsafe_counter > 10 ){
	        failsafe_counter = 0;
	        	MF.FLAG.FAILSAFE = 1;
//	          Front_LED_Light(1,1,1);
	  		Motor_Drive_Stop();
			HAL_ADC_Stop_DMA(&hadc1);
	        }
	    } else {
	      failsafe_counter = 0;
	    }
	  }


	//Motor Control
	if(MF.FLAG.MT_CTRL){
		if(MF.FLAG.WALL_CTRL){
		Side_Wall_Control();
	    Front_Wall_Control();
		}

		if(trans_trapezoid_params.flag == 1){
			Acceleration_Control(&trans_target, &trans_trapezoid_params);
		}
		if(rot_trapezoid_params.flag == 1){
			Acceleration_Control(&rot_target, &rot_trapezoid_params);
			Calculate_Target(&rot_target);
		}

	PID_Control(&trans_target, &trans_machine, &trans_info ,&trans_gain, &motor_duty, &trans_trapezoid_params,0);

		if ( trans_trapezoid_params.reverse_flag == 0 || trans_target.velocity > 100.0f ){
			PID_Control(&rot_target, &rot_machine, &rot_info ,&rot_gain, &motor_duty, &rot_trapezoid_params,1);
		}

	Calculate_Target(&trans_target);
	integralDistance( &trans_r.velocity, &trans_r.distance );

	Motor_Direction_Decide(&motor_duty);
	Motor_PWM_OUT(&motor_duty);


		 log_cnt = 1;

	if(log_cnt==1) {
			Log_Saver();
		}

	//dutyリセット
	motor_duty.left = 0;
	motor_duty.right = 0;

	Buzzer_Control();

	} else {
		Motor_Drive_Stop();
	}
  }
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
	Get_Sensor_Data(&adc_case);
}
