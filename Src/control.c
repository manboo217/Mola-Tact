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
	//Back & Right Turn
	if(distance < 0.0f){
		trapezoid -> reverse_flag = 1;
//		distance *= -1.0f;
	}

	//Storage params into typedef trapezoid
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

	float distance = 0.0f;

	if(trapezoid->reverse_flag == 0) {
		distance = trapezoid->distance - trapezoid->decel_distance;
  if ( target->distance < trapezoid->accel_distance ){
	  if ( target->velocity < trapezoid->max_velocity ){
		  target->acceleration = trapezoid->acceleration;
	  } else {
		  target->acceleration = 0.0f;
		  target->velocity = trapezoid->max_velocity;
	  }

  } else if ( target->distance < distance){
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
    //	走行フラグオフ
    trapezoid->flag = 0;
  	  }

  /*/*****reverse*****/
	}else{
	distance = trapezoid->distance + trapezoid->decel_distance;
		if ( target->distance > -trapezoid->accel_distance ){
			if ( target->velocity > -trapezoid->max_velocity ){
				target->acceleration = -trapezoid->acceleration;
			} else {
				target->acceleration = 0.0f;
				target->velocity = -trapezoid->max_velocity;
			}
		} else if ( target->distance > distance){
			if ( target->velocity > trapezoid->max_velocity ){
				target->acceleration = -trapezoid->acceleration;
			} else {
				target->acceleration = 0.0f;
				target->velocity = -trapezoid->max_velocity;
			}

		} else if ( target->velocity < trapezoid->terminal_velocity ){
				target->acceleration = trapezoid->acceleration;
		} else {
				target->acceleration = 0.0f;
				target->velocity = trapezoid->terminal_velocity;
				//	走行フラグオフ
				trapezoid->flag = 0;
  	  }
	}
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
 *         rotation_flag -> 回転の場合にHIGH
 */
//+++++++++++++++++++++++++++++++++++++++++++++++


/*
トルク.右 = 速度制御量 + 角度制御量 + 壁制御量;
トルク.左 = 速度制御量 - 角度制御量 - 壁制御量;
 */

void PID_Control(motion *target, motion *machine, PID_info *store ,
		PID_Gain *gain ,duty *duty, trapezoid *trapezoid, uint8_t rotation_flag)
{
  int32_t duty_l, duty_r;
  float error;
  float difference;
  float Kp, Ki, Kd;

  Kp = gain->Kp;
  Ki = gain->Ki;
  Kd = gain->Kd;
/*
  if ( trapezoid->reverse_flag == 1 ){
    machine->velocity = -1.0f * ( left->velocity + right->velocity ) / 2.0f;
    right->velocity = left->velocity;
  } else {
    left->velocity = ( left->velocity + right->velocity ) / 2.0f;
    right->velocity = left->velocity;
  }
*/
  //wall control
  if (rotation_flag){
    machine->velocity += side_wall_control_value;
  }else {
    machine->velocity += front_wall_control_value;
  }

  //Calculate error
  error =  target->velocity - machine->velocity ;

  //Calculate differential
  difference = error - (store->previous_error);

  //Calculate sum(Integral)
  store->sum += (error + (store->previous_error))/ 2.0f * TIM4_INTERVAL;

  //Calculate output
  duty_l = (int32_t)(Kp * error + Ki * (store->sum) + Kd * difference);
  duty_r = (int32_t)(Kp + error + Ki * (store->sum) + Kd * difference);

  //previous_errorに格納
  store->previous_error = error;

  //	回転時は回転方向を逆に
  if (rotation_flag){
    duty_l = -1 * duty_l;
  }
/*
  if(trapezoid->reverse_flag)
  {
	  duty_l = -1 * duty_l;
	  duty_r = -1 * duty_r;
  }*/

  	  duty->left += duty_l;
  	  duty->right += duty_r;
}

void FeedForward_Control(float velocity_l,float velocity_r, float acceleration, float velocity,
		duty *duty, float battery_voltage, uint8_t reverse_flag){
	float machine_velocity = 0.0f;
	float reverse_voltage = 0.0f;
	float output_power = 0.0f;
	float output_duty = 0.0f;
	float machine_acceleration = 0.0f;
	// calculate reverse voltage = 逆起電力定数 * モータ回転数
	if(acceleration != 0.0f){
		reverse_voltage = 0.0000171f * velocity;
	}

	//out_power = 車輪半径 * 機体重量 * 加速度 / ( 2.0f * 減速比);
	 if ( acceleration > 0.0f ){
	      output_power = (machine_acceleration + 2.0f) * 0.00024555f;
	    } else if ( acceleration == 0.0f ){
	      output_power = 0.0f;
	    } else {
	      output_power = machine_acceleration * 0.00024555f;
	    }

	//	 出力すべき値の計算
	 //out_duty = ( MOTOR_RESISTOR * out_power / MOTOR_TORQUE_CONSTANT +  motor_reverse_v ) / Vbat;
	if ( reverse_flag == 0 ){
		output_duty = ( 757.576f * output_power + reverse_voltage ) / battery_voltage;
	 } else {
	    output_duty = 0.0f;
	 }
	duty->left += (int32_t)(output_duty * 400);
	duty->right += (int32_t)(output_duty * 400);
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

		//	左右共に制御閾値より大きい時
		if((sensor_data.sideL > control_threshold.sideL) && (sensor_data.sideR > control_threshold.sideR)){
	//	if(wall_data.sideL == 1 && wall_data.sideR == 1){
		      error = (float)((sensor_data.sideL - base_data.sideL)
		    		  	  -(sensor_data.sideR - base_data.sideR));
		      Front_LED_Light(0,1,0);
		}

		//	左右共に制御閾値より小さい時
		else if((sensor_data.sideL <= control_threshold.sideL)&&(sensor_data.sideR <= control_threshold.sideR)){
	//	else if(wall_data.sideL == 0 && wall_data.sideR == 0){
			error = 0.0f;
		      Front_LED_Light(0,0,0);

		}

		//	右だけ制御閾値より大きい時
		else if(sensor_data.sideR > control_threshold.sideR){
			//	else if(wall_data.sideR == 1 && wall_data.sideL ==  0){

			error = (float) -2.0f * (sensor_data.sideR - base_data.sideR);
		      Front_LED_Light(0,0,1);

		}
		//	左だけ制御閾値より大きい時
		else {
			error = (float) 2.0f * (sensor_data.sideL - base_data.sideL);
		      Front_LED_Light(1,0,0);

		}

		//set upper limit
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
//	前壁制御
//	前壁制御フラグON・前壁有・速度が低くなってきたら制御をかける
//+++++++++++++++++++++++++++++++++++++++++++++++

void Front_Wall_Control( void )
{
  if ( MF.FLAG.FRONT_WALL_CTRL == 1 && (wall_info & 0x80) && trans_machine.velocity < 200.0f ){
    front_wall_control_value = (float) 0.2f * (sensor_data.frontL - control_threshold.frontL);
  } else {
    front_wall_control_value = 0.0f;
  }
}

