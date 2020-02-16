/*
 * global.h
 *
 *  Created on: May 12, 2019
 *      Author: cinqu
 */

#ifndef GLOBAL_H_
#define GLOBAL_H_

#include "main.h"
#include "params.h"
#include "stm32f4xx_hal_tim.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim8;

extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;


/*------------------------------------------------------------
    共用・構造体の定義
------------------------------------------------------------*/
/**********
共用・構造体とは，共用体と構造体を組み合わせたもので，
内部の一括操作も，メンバ単位での操作も可能なものである。
例えば，以下のmouse_flags共用・構造体のMFでは，
MF.FLAGS = 0; と全体を一括変更できるほか，
MF.FLAG.DECL = 1; とメンバを指定して単体で変更することも出来る。
**********/

//----フラグ共用・構造体----
typedef union {         // 共用体の宣言
  uint16_t FLAGS;
  struct ms_flags{      // 構造体の宣言
    uint16_t MT_CTRL:1;    // モータ制御フラグ(B0)   (:1 は1ビット分の意味，ビットフィールド)
    uint16_t SCND:1;    // 二次走行フラグ(B1)
    uint16_t REVERSE:1;    //(B2)
    uint16_t SIDE_WALL_CTRL:1;    // 制御フラグ(B3)
    uint16_t FRONT_WALL_CTRL:1;    // 加速フラグ(B4)
    uint16_t KNOWN:1;    // 減速フラグ(B5)
    uint16_t FAILSAFE:1;     // デフォルトインターバルフラグ(B6)
    uint16_t WALL_CTRL:1;    // 予備ビット(B7)
    uint16_t RSV8:1;    // 予備ビット(B8)
    uint16_t RSV9:1;    // 予備ビット(B9)
    uint16_t RSV10:1;   // 予備ビット(B10)
    uint16_t RSV11:1;   // 予備ビット(B11)
    uint16_t RSV12:1;   // 予備ビット(B12)
    uint16_t RSV13:1;   // 予備ビット(B13)
    uint16_t RSV14:1;   // 予備ビット(B14)
    uint16_t RSV15:1;   // 予備ビット(B15)
  }FLAG;
} mouse_flags;


typedef struct {
	float distance;
	float velocity;
	float acceleration;
}motion;

typedef struct{
	int16_t left;
	int16_t right;
}duty;

typedef struct {
	  volatile float distance;
	  volatile float acceleration;
	  volatile float max_velocity;
	  volatile float initial_velocity;
	  volatile float terminal_velocity;
	  volatile float accel_distance;
	  volatile float decel_distance;
	  volatile uint8_t flag;
	  volatile uint8_t reverse_flag;
}trapezoid;

typedef struct {
  float Kp;
  float Ki;
  float Kd;
}PID_Gain;

typedef struct {
	float angular_acceleration;
	float max_angular_velocity;
	float offset_in;
	float offset_out;
}slalom_params;


typedef struct {
	uint32_t frontL;
	uint32_t frontR;
	uint32_t sideL;
	uint32_t sideR;
}sensor;


#ifdef MAIN_C_
	volatile mouse_flags MF;
	/*並進運動*/
	motion trans_l;				//左モータの並進情報
	motion trans_r;				//右モータの並進情報
	motion trans_machine;		//ロボットの並進情報
	motion trans_target;		//ロボットの目標並進情報
	motion trans_params;		//並進速度・加速度情報
	motion trans_params_known;		//並進速度・加速度情報

	trapezoid trans_trapezoid_params;	//並進の台形加速情報
	PID_Gain trans_gain;				//PIDゲイン


	/*回転運動*/
	motion angle_l;				//	左モータの角度所法
	motion angle_r;				//	右モータの角度情報
	motion rot_machine;			//	ロボットの角度情報
	motion rot_target;			//	ロボットの目標角度情報
	motion rot_params;

	trapezoid rot_trapezoid_params;		//	回転の台形加速情報
	PID_Gain rot_gain;				//PIDゲイン

	slalom_params slalom;

	duty motor_duty;			//	モータのduty

	sensor sensor_data;
	sensor sensor_old;

	sensor wall_data;
	sensor difference;


	sensor base_data;
	sensor threshold;
	sensor control_threshold;

	PID_Gain sensor_gain;

#else
	extern volatile mouse_flags MF;
	/*	並進運動	*/
	extern motion trans_l;				//	左モータの並進情報
	extern motion trans_r;				//	右モータの並進情報
	extern motion trans_machine;		//	ロボットの並進情報
	extern motion trans_target;		//	ロボットの目標並進情報
	extern motion trans_params;		//	並進速度・加速度情報
	extern motion trans_params_known;		//	並進速度・加速度情報


	extern trapezoid trans_trapezoid_params;
	extern PID_Gain trans_gain;

	/*回転運動*/
	extern motion angle_l;
	extern motion angle_r;
	extern motion rot_machine;
	extern motion rot_target;
	extern motion rot_params;
	extern PID_Gain rot_gain;
	extern trapezoid rot_trapezoid_params;

	extern slalom_params slalom;

	extern duty motor_duty;

	extern sensor base_data;
	extern sensor sensor_data;
	extern sensor sensor_old;
	extern sensor wall_data;
	extern sensor difference;

	extern sensor threshold;
	extern sensor control_threshold;

	extern PID_Gain sensor_gain;


#endif

#include <stdio.h>
#include <stdint.h>

#include "control.h"
#include "drive.h"
#include "encoder.h"
#include "imu.h"
#include "interrupt.h"
#include "led.h"
#include "log.h"
#include "mode.h"
#include "adc.h"
#include "search.h"
#include "tim.h"
#include "flash.h"

#endif /* GLOBAL_H_ */
