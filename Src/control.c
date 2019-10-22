/*
 * control.c
 *
 *  Created on: May 2, 2019
 *      Author: manboo
 */

#include "global.h"


void Set_PID_Params(PID_Gain *gain, float Kp, float Ki, float Kd)
{
	gain->Kp = Kp;
	gain->Ki = Ki;
	gain->Kd = Kd;
}


void Set_Search_Params(motion *motion, float velocity, float acceleration)
{
	motion->velocity = velocity;
	motion->acceleration = acceleration;
}

//+++++++++++++++++++++++++++++++++++++++++++++++
//Set_Trapezoid_Params
//Store parameters for Trapezoid Acceleration
//+++++++++++++++++++++++++++++++++++++++++++++++

void Set_Trapezoid_Params( trapezoid *trapezoid, float distance, float acceleration,
                float initial_velocity, float terminal_velocity, float max_velocity )
{
	trapezoid->reverse_flag = 0;
	//バック&右ターンの処理
	if(distance < 0.0f){
		trapezoid -> reverse_flag = 1;
		distance *= -1.0f;
	}

	//trapezoid構造体にパラメータを格納
  trapezoid->distance = distance;
  trapezoid->acceleration = acceleration;
  trapezoid->initial_velocity = initial_velocity;
  trapezoid->terminal_velocity = terminal_velocity;
  trapezoid->max_velocity = max_velocity;

  if ( acceleration != 0.0f ){
    trapezoid->accel_distance = ( max_velocity * max_velocity - initial_velocity * initial_velocity ) / ( 2.0f * acceleration );
    trapezoid->decel_distance = ( max_velocity * max_velocity - terminal_velocity * terminal_velocity ) / ( 2.0f * acceleration );
  } else {
    trapezoid->accel_distance = 0.0f;
    trapezoid->decel_distance = 0.0f;
  }
}


//+++++++++++++++++++++++++++++++++++++++++++++++
/* Acceleration_Control
 * Calculate target distance/velocity/acceleration for Trapezoid Acceleration
 * 					(or angle/angular_velocity/angular_acceleration)
 *
 *		motion *target:目標情報を格納
 *		trapezoid *trapezoid:台形加速情報を読み込み
 */
//+++++++++++++++++++++++++++++++++++++++++++++++
void Acceleration_Control( motion *target, trapezoid *trapezoid )
{
  if ( target->distance < trapezoid->accel_distance ){
	  if ( target->velocity < trapezoid->max_velocity ){
		  target->acceleration = trapezoid->acceleration;
	  } else {
		  target->acceleration = 0.0f;
		  target->velocity = trapezoid->max_velocity;
	  }

  } else if ( target->distance < trapezoid->distance - trapezoid->decel_distance){
	  if ( target->velocity < trapezoid->max_velocity ){
		  target->acceleration = trapezoid->acceleration;
	  } else {
		  target->acceleration = 0.0f;
		  target->velocity = trapezoid->max_velocity;
	  }

  } else if ( target->velocity > trapezoid->terminal_velocity ){
    target->acceleration = -trapezoid->acceleration;

  } else {
	target->acceleration = 0.0f;
    target->velocity = trapezoid->terminal_velocity;
    //走行フラグオフ
    trapezoid->flag = 0;
  }
  //Calculate Target Distance & Velocity
  //v=at, x= v_0*t+(at^2)/2 これを無限に繰り返すことでtargetを更新していく
  //target->velocity += target->acceleration * TIM4_INTERVAL;
  //target->distance += (target->velocity) * TIM4_INTERVAL + (target->acceleration) * TIM4_INTERVAL * TIM4_INTERVAL / 2.0f;

}

void Calculate_Target( motion *target )
{
  target->velocity += target->acceleration * TIM4_INTERVAL;
  target->distance += target->velocity * TIM4_INTERVAL + target->acceleration * TIM4_INTERVAL * TIM4_INTERVAL / 2.0f;
}

void integralDistance( float *velocity, float *distance )
{
  *distance += *velocity * TIM4_INTERVAL;
}



//+++++++++++++++++++++++++++++++++++++++++++++++
//PID_Control
/* Determine outputs of Motors with PID Control
 *    	   motion *target:マシンの目標値を読み込み
 *         motion *left: 左モータの情報を読み込み(Encoder.c)
 *         motion *right:右モータの情報を読み込み(Encoder.c)
 *         PID_gain *gain:PIDゲインの読み込み
 *         duty *duty:出力ゲインを格納
 */
//+++++++++++++++++++++++++++++++++++++++++++++++


/*
トルク.右 = 速度制御量 + 角度制御量 + 壁制御量;
トルク.左 = 速度制御量 - 角度制御量 - 壁制御量;
 */

void PID_Control(motion *target, motion *left, motion *right, PID_info *store ,
		PID_Gain *gain ,duty *duty, trapezoid *trapezoid, uint8_t rotation_flag)
{
  int32_t duty_l, duty_r;
  float error_l, error_r;
  float diff_l, diff_r;
  float Kp, Ki, Kd;

  Kp = gain->Kp;
  Ki = gain->Ki;
  Kd = gain->Kd;

  if ( trapezoid->reverse_flag == 1 ){
    left->velocity = -1.0f * ( left->velocity + right->velocity ) / 2.0f;
    right->velocity = left->velocity;
  } else {
    left->velocity = ( left->velocity + right->velocity ) / 2.0f;
    right->velocity = left->velocity;
  }

  //wall control

  if (rotation_flag){
    left->velocity += side_wall_control_value;
    right->velocity += side_wall_control_value;
  } /*else {
    left->velocity += frontwall_control_value;
    right->velocity += frontwall_control_value;
  }
*/


  //Calculate error
  error_l = ( target->velocity - left->velocity );
  error_r = ( target->velocity - right->velocity );

  //偏差の変化量を計算
  diff_l = error_l - (store->previous_error_l);
  diff_r = error_r - (store->previous_error_r);

  //積分値を計算
  store->sum_l += (error_l + (store->previous_error_l))/ 2.0f * TIM4_INTERVAL;
  store->sum_r += (error_r + (store->previous_error_r))/ 2.0f * TIM4_INTERVAL;

  //出力を計算
  duty_l = (int16_t)(Kp * error_l + Ki * (store->sum_l) + Kd * diff_l);
  duty_r = (int16_t)(Kp + error_r + Ki * (store->sum_r) + Kd * diff_r);

  //previous_errorに格納
  store->previous_error_l = error_l;
  store->previous_error_r = error_r;


  if (rotation_flag){
    duty_l = -1 * duty_l;
  }

  if(trapezoid->reverse_flag)
  {
	  duty_l = -1 * duty_l;
	  duty_r = -1 * duty_r;
  }

  	  duty->left += duty_l;
  	  duty->right += duty_r;
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//Set_Trapezoid_Params
//->interrupt.c
//+++++++++++++++++++++++++++++++++++++++++++++++

void Side_Wall_Control(void)
{
	float error = 0.0f;
	side_wall_control_value = 0.0f;
	float diff_value = 7.0f;

	if(MF.FLAG.SIDE_WALL_CTRL && (trans_target.velocity > 300.0f) && (difference.sideL < diff_value) && (difference.sideR < diff_value)){

		//左右共に制御閾値より大きい時
		if((sensor_data.sideL > control_threshold.sideL) && (sensor_data.sideR > control_threshold.sideR)){
	//	if(wall_data.sideL == 1 && wall_data.sideR == 1){
		      error = (float)((sensor_data.sideL - base_data.sideL)
		    		  	  -(sensor_data.sideR - base_data.sideR));
		      Front_LED_Light(0,1,0);
		}

		//左右共に制御閾値より小さい時
		else if((sensor_data.sideL <= control_threshold.sideL)&&(sensor_data.sideR <= control_threshold.sideR)){
	//	else if(wall_data.sideL == 0 && wall_data.sideR == 0){
			error = 0.0f;
		      Front_LED_Light(0,0,0);

		}

		//右だけ制御閾値より大きい時
		else if(sensor_data.sideR > control_threshold.sideR){
			//	else if(wall_data.sideR == 1 && wall_data.sideL ==  0){

			error = (float) -2.0f * (sensor_data.sideR - base_data.sideR);
		      Front_LED_Light(0,0,1);

		}
		//左だけ制御閾値より大きい時
		else {
			error = (float) 2.0f * (sensor_data.sideL - base_data.sideL);
		      Front_LED_Light(1,0,0);

		}

		//上限下限設定
	    if ( error > 100.0f ){
		        error = 100.0f;
		} else if ( error < -100.0f ){
		        error = -100.0f;
		}
	}

	side_wall_control_value = sensor_gain.Kp * error;

	/* else if ( dirwall_control_flag == 1 ){
		sidewall_control_value = 0.0f;
		// 4つのセンサのそれぞれの値の閾値を決めてそれに対して制御量を気持ち与える。

		if ( sen_fl.now > 120 && sen_fl.diff < 100 ){
  	  	  	  sidewall_control_value = (float)0.6f * ( sen_fl.now - 80 );
		}
		else if ( sen_l.now > 690 && sen_l.diff_1ms < 100 ){
  	  	  	  	sidewall_control_value = (float)0.6f * ( sen_l.now - 660 );
		}
		else if ( sen_fr.now > 110 && sen_fr.diff < 100 ){
  	  	  	    sidewall_control_value = (float)-0.6f * ( sen_fr.now - 70 );
		}
		else if ( sen_r.now > 690 && sen_r.diff_1ms < 100 ){
  	  	  	  	sidewall_control_value = (float)-0.6f * ( sen_r.now - 660 );
		}

	} else {
		sidewall_control_value = 0.0f;
}
*/
}

//+++++++++++++++++++++++++++++++++++++++++++++++
//Front_Wall_Control
// 前壁制御
// 前壁制御フラグON・前壁有・速度が低くなってきたら制御をかける
//+++++++++++++++++++++++++++++++++++++++++++++++

void Front_Wall_Control( void )
{
  if ( MF.FLAG.FRONT_WALL_CTRL == 1 && (wall_info & 0x80) && trans_machine.velocity < 200.0f ){
    front_wall_control_value = (float) 0.2f * (sensor_data.frontL - control_threshold.frontL);
  } else {
    front_wall_control_value = 0.0f;
  }
}
