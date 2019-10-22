/*
 * drive.c
 *
 *  Created on: May 01, 2019
 *      Author: manboo
 */

#include "global.h"


void Motor_Init(){
	  if(HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3)!=HAL_OK)
	  {
		  printf("Ch3 Start Error\r\n");
		  Error_Handler();
	  }

	  if(HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4)!=HAL_OK)
	  {
		  printf("Ch4 Start Error\r\n");
		  Error_Handler();
	  }
}



//+++++++++++++++++++++++++++++++++++++++++++++++
//Straight_Mode
// 加速パラメータを直進モードに設定->走行フラグを立てる
//+++++++++++++++++++++++++++++++++++++++++++++++
void Straight_Mode( float distance, float acceleration, float initial_velocity,
		float terminal_velocity, float max_velocity)
{

	Set_Trapezoid_Params( &trans_trapezoid_params, distance, acceleration,
		  initial_velocity, terminal_velocity, max_velocity );

	trans_trapezoid_params.flag = 1;//走行フラグ
	trans_target.velocity = trans_trapezoid_params.initial_velocity;
}

//+++++++++++++++++++++++++++++++++++++++++++++++
//Stay_Drive <- waitStraight
// 直進モード続行
//+++++++++++++++++++++++++++++++++++++++++++++++

void Stay_Straight(void)
{

	while(trans_trapezoid_params.flag == 1){

	}
	//reset target
	trans_target.distance = 0.0f;
	trans_target.velocity = trans_trapezoid_params.terminal_velocity;
	trans_target.acceleration = 0.0f;

	//reset PID
	trans_info.previous_error_l = 0.0f;
	trans_info.previous_error_r = 0.0f;
	trans_info.sum_l = 0.0f;
	trans_info.sum_r = 0.0f;

	//reset reverse flag
	trans_trapezoid_params.reverse_flag = 0;//後退フラグ
	rot_trapezoid_params.reverse_flag = 0;//右ターンフラグ

	MF.FLAG.SIDE_WALL_CTRL = 0;
	MF.FLAG.FRONT_WALL_CTRL = 0;
}

void Stay_Start_Straight( void )
{

	while( trans_trapezoid_params.flag == 1 ){
	}


  // reset target data
  trans_target.acceleration = 0.0f;
  trans_target.distance = 0.0f;
  trans_target.velocity = trans_trapezoid_params.terminal_velocity;

  // reset trape back turn flag
  trans_trapezoid_params.reverse_flag = 0;

  // reset pid parameter
	trans_info.previous_error_l = 0.0f;
	trans_info.previous_error_r = 0.0f;
	trans_info.sum_l = 0.0f;
	trans_info.sum_r = 0.0f;

//壁・前壁制御フラグ
  MF.FLAG.SIDE_WALL_CTRL = 0;
  MF.FLAG.FRONT_WALL_CTRL = 0;
 // dirwall_control_flag = 0;


  rot_trapezoid_params.reverse_flag = 0;

}

//+++++++++++++++++++++++++++++++++++++++++++++++
//Rotation_Mode
//回転モードに設定
//+++++++++++++++++++++++++++++++++++++++++++++++
void Rotation_Mode( float angle, float angular_acceleration,
		float max_angular_velocity, float machine_velocity )
{
	Set_Trapezoid_Params( &rot_trapezoid_params, angle, angular_acceleration,
		  	  	  	  0.0f, 0.0f, max_angular_velocity );

  rot_target.velocity = 0.0f;
  trans_target.acceleration = 0.0f;
  trans_target.velocity = machine_velocity;

  rot_trapezoid_params.flag = 1;//走行フラグ

  Stay_Rotation();
  rot_trapezoid_params.flag = 0;//走行フラグ

}

//+++++++++++++++++++++++++++++++++++++++++++++++
//Stay_Rotation <- setRotation
// 回転モード続行
//+++++++++++++++++++++++++++++++++++++++++++++++

void Stay_Rotation(){
	//フラグが立っている間は回転
	while(rot_trapezoid_params.flag){
	}

	//初期化
	trans_target.distance = 0.0f;

	rot_target.acceleration = 0.0f;
	rot_target.velocity = 0.0f;
	rot_target.distance = 0.0f;

	/*
	rot_info.previous_error_l = 0.0f;
	rot_info.previous_error_r = 0.0f;
	rot_info.sum_l = 0.0f;
	rot_info.sum_r = 0.0f;
*/
	rot_trapezoid_params.reverse_flag = 0;
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//Wait_Motion <- waitMotion
// 動作終了を待機
//+++++++++++++++++++++++++++++++++++++++++++++++

void Wait_Motion(volatile int32_t time_ms){
	  motion_counter = 0;

	  rot_info.sum_l = 0.0f;
	  rot_info.sum_r= 0.0f;

	  while( motion_counter < time_ms ){
	    // 動作をしない

	  }
	  rot_info.sum_l = 0.0f;
	  rot_info.sum_r= 0.0f;
}


void Wait_Slalom_Out(void){

	while(trans_trapezoid_params.flag){

	}
	trans_target.acceleration = 0.0f;
	trans_target.distance = 0.0f;
	trans_target.velocity = trans_trapezoid_params.terminal_velocity;

	trans_trapezoid_params.reverse_flag = 0;

	trans_info.previous_error_l = 0.0f;
	trans_info.previous_error_r = 0.0f;
	trans_info.sum_l = 0.0f;
	trans_info.sum_r = 0.0f;

	rot_trapezoid_params.reverse_flag = 0;

}

//+++++++++++++++++++++++++++++++++++++++++++++++
//Set_Slalom_Params　<-setSlalomOffset
// スラロームパラメータ設定
//+++++++++++++++++++++++++++++++++++++++++++++++


void Set_Slalom_Params(slalom_params *slalom, float angular_acceleration, float max_angular_velocity, float offset_in, float offset_out)
{
	slalom->angular_acceleration = angular_acceleration;
	slalom->max_angular_velocity = max_angular_velocity;
	slalom->offset_in = offset_in;
	slalom->offset_out = offset_out;
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//Accel_Half_Section
// 半区画加速
//+++++++++++++++++++++++++++++++++++++++++++++++

void Accel_Half_Section(float acceleration , float velocity)
{
	  MF.FLAG.SIDE_WALL_CTRL = 0;  // 壁制御有効
	Straight_Mode( HALF_SECTION, acceleration, 0.0f, velocity, velocity );
	Stay_Straight();//目標距離到達まで待機
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//Forward_One_Section
// 1区間等速前進
//+++++++++++++++++++++++++++++++++++++++++++++++
void Forward_One_Section(float velocity)
{
	MF.FLAG.SIDE_WALL_CTRL = 1;
	Straight_Mode( ONE_SECTION, 0.0f, velocity, velocity, velocity );
	Stay_Straight();//目標距離到達まで待機

}

void Adjust_Forward( float acceleration, float velocity )
{
//  wall_out_flag = 1;          // 壁切れを読みの許可
	MF.FLAG.SIDE_WALL_CTRL = 0;
	Straight_Mode( ADJUST_FRONT_DISTANCE, acceleration, 0.0f, velocity, velocity );
	Stay_Straight();
}


void Adjust_Back( void )
{
	Straight_Mode( -150.0f, 1500.0f, 0.0f, 0.0f, 150.0f );
	Stay_Straight();
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//Accel_Known_Section
// 既知区間加速前進
//+++++++++++++++++++++++++++++++++++++++++++++++
void Accel_Known_Section(float acceleration, float initial_velocity ,float terminal_velocity)
{
	MF.FLAG.SIDE_WALL_CTRL = 0;
	Straight_Mode( ONE_SECTION, acceleration, initial_velocity, terminal_velocity, terminal_velocity );
	Stay_Straight();//目標距離到達まで待機

}


//+++++++++++++++++++++++++++++++++++++++++++++++
//Decel_Known_Section
// 既知区間加速前進
//+++++++++++++++++++++++++++++++++++++++++++++++
void Decel_Known_Section(float acceleration, float initial_velocity ,float terminal_velocity)
{
	MF.FLAG.SIDE_WALL_CTRL = 0;
	MF.FLAG.FRONT_WALL_CTRL = 1;
	Straight_Mode( ONE_SECTION, acceleration, initial_velocity, terminal_velocity, initial_velocity );
	Stay_Straight();//目標距離到達まで待機

}

//+++++++++++++++++++++++++++++++++++++++++++++++
//Deccel_Half_Section
// 半区画減速後停止
//+++++++++++++++++++++++++++++++++++++++++++++++

void Decel_Half_Section(float acceleration , float velocity)
{
	MF.FLAG.SIDE_WALL_CTRL = 0;  // 壁制御有効
	MF.FLAG.FRONT_WALL_CTRL = 1;

	Straight_Mode( HALF_SECTION, acceleration, velocity, 0.0f, velocity );
	Stay_Straight();//目標距離到達まで待機

}


//+++++++++++++++++++++++++++++++++++++++++++++++
//PivotL90
// 超信地旋回L90モードに設定
//+++++++++++++++++++++++++++++++++++++++++++++++

void PivotL90( float acceleration, float velocity )
{
  Rotation_Mode( 90.0f, acceleration, velocity, 0.0f );
  Wait_Motion(300);
}

//+++++++++++++++++++++++++++++++++++++++++++++++
//PivotR90
// 超信地旋回R90モードに設定
//+++++++++++++++++++++++++++++++++++++++++++++++

void PivotR90( float acceleration, float velocity )
{
  MF.FLAG.SIDE_WALL_CTRL = 0;  // 壁制御有効

  Rotation_Mode( -90.0f, acceleration, velocity, 0.0f );
  Wait_Motion(300);
}

//+++++++++++++++++++++++++++++++++++++++++++++++
//PivotL180
// 超信地旋回L180モードに設定
//+++++++++++++++++++++++++++++++++++++++++++++++

void PivotL180(float acceleration, float velocity)
{
	MF.FLAG.SIDE_WALL_CTRL = 0;  // 壁制御有効

  Rotation_Mode( 180.0f, acceleration, velocity, 0.0f );
}

void PivotR180(float acceleration, float velocity)
{
	MF.FLAG.SIDE_WALL_CTRL = 0;  // 壁制御有効

  Rotation_Mode( -180.0f, acceleration, velocity, 0.0f );
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//Slalom_L
//スラローム左旋回モードに設定
//+++++++++++++++++++++++++++++++++++++++++++++++

void Slalom_L(float velocity)
{
	MF.FLAG.SIDE_WALL_CTRL = 0;  // 壁制御有効
	Straight_Mode( slalom.offset_in, 0.0f, velocity, velocity, velocity );
	Stay_Straight();
	Rotation_Mode( 90.0f, slalom.angular_acceleration, slalom.max_angular_velocity, velocity );
	Straight_Mode( slalom.offset_out, 0.0f, velocity, velocity, velocity );
	Wait_Slalom_Out();
}

//+++++++++++++++++++++++++++++++++++++++++++++++
//Slalom_R
//スラローム左旋回モードに設定
//+++++++++++++++++++++++++++++++++++++++++++++++

void Slalom_R(float velocity)
{
	MF.FLAG.SIDE_WALL_CTRL = 0;	// 壁制御有効

	Straight_Mode( slalom.offset_in, 0.0f, velocity, velocity, velocity );
	Stay_Straight();
	Rotation_Mode( -90.0f, slalom.angular_acceleration, slalom.max_angular_velocity, velocity );
	Straight_Mode( slalom.offset_out, 0.0f, velocity, velocity, velocity );
	Wait_Slalom_Out();

}


void SlalomR_V90(void){

}


void SlalomL_V90(void){

}

//+++++++++++++++++++++++++++++++++++++++++++++++
//Motor_Direction_Decide
//左右の回転方向をduty値によって変更
//+++++++++++++++++++++++++++++++++++++++++++++++

void Motor_Direction_Decide(duty *duty)
{

	if((duty->left)>=0)
	{
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,SET);//Left Motor IN1
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,RESET);//Left Motor IN2
	}else{
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,RESET);//Left Motor IN1
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,SET);//Left Motor IN2
	}

	if((duty->right)>=0)
	{
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,RESET);//Right Motor IN1
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,SET);//Right Motor IN2
	}else{
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,SET);//Right Motor IN1
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,RESET);//Right Motor IN2
	}
}

//+++++++++++++++++++++++++++++++++++++++++++++++
//Motor_PWM_OUT
/*	モータPWM出力
 *   	->上限処理
 * 		->MT_CTRLフラグ0で停止
 */
//+++++++++++++++++++++++++++++++++++++++++++++++

void Motor_PWM_OUT(duty *duty)
{
	int32_t PWM_L, PWM_R = 0;
//	duty->left  +=  side_wall_control_value;
//	duty->right -=  side_wall_control_value;
	//負整数処理
	if((duty->left) <0)	{
		PWM_L = - (duty->left);
	}
	else {
		PWM_L = (duty->left);
	}
	if((duty->right)<0)	{
		PWM_R = - (duty->right);
	}
	else {
		PWM_R = (duty->right);
	}

	//上限速度処理
	if (PWM_L > 400)	PWM_L = 400;

	if (PWM_R > 400)	PWM_R = 400;

	//MT_FLAGが下がったら出力停止
	if(MF.FLAG.MT_CTRL==0) {
		PWM_L = 0;
		PWM_R = 0;
	}

	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,PWM_L);
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4,PWM_R);
}

void Motor_Drive_Stop(void){
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3, 0);
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4, 0);

}

void Drive_Test(void){

}
