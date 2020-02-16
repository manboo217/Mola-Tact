/*
 * mode.c
 *
 *  Created on: May 5, 2019
 *      Author: manboo
 */

#include "global.h"


void Mode_Select(int16_t mode){


	printf("Mode %d Start!\r\n", mode);

	LED_Light1();

	//Set Params
	Set_Search_Params(&trans_params, 450.0f, 2000.0f); //velocity ・ acceleration
	Set_Search_Params(&rot_params, 450.0f, 5500.0f);//angular velocity ・ angular acceleration
	Set_Slalom_Params(&slalom, 6000.0f, 450.0f, 13.5f, 13.5f); // 4600 450 20 20

	Set_Search_Params(&trans_params_known, 1000.0f, 3800.0f); //velocity ・ acceleration

	Set_PID_Params(&trans_gain, 1.8f, 30.0f, 0.0f); //1.7f, 36.0f->10.0f, 0.0f
	Set_PID_Params(&rot_gain, 0.5f, 60.0f,0.0f);	//φ24.5のとき0.47f, 36.0f,0.0f

	Set_PID_Params(&sensor_gain, 0.13f, 0.0f,0.0f);	//	壁制御 0.007

	//	壁判断閾値
	Set_Threshold(&threshold,100, 100, 100, 100);			//FL,SL,SR,FR 28,41,38,28

	//	壁制御閾値->右にずれたら左の閾値をあげる
	Set_Threshold(&control_threshold,200,350,200,250);			//FL,SL,SR,FR 220,300,250,250

	HAL_Delay(500);
	IMU_Init();
	ICM20602_Init();

	switch(mode){
	case 0:
		gyro_calib_flag = 1;
		   while(gyro_calib_flag){
		   }

		Mode_Zero();
		break;

	case 1:
		gyro_calib_flag = 1;
		   while(gyro_calib_flag){
		   }

		Mode_One();
		break;
	case 2:
		gyro_calib_flag = 1;
		   while(gyro_calib_flag){
		   }

		Mode_Two();
		break;

	case 3:
		Set_Search_Params(&trans_params, 600.0f, 1500.0f); //velocity ・ acceleration
		Set_Search_Params(&rot_params, 600.0f, 5500.0f);//angular velocity ・ angular acceleration
		Set_Slalom_Params(&slalom, 4200.0f, 500.0f, 19.5f, 19.5f); // 4600 450 20 20

		gyro_calib_flag = 1;

		while(1){
		Mode_Three();
		}
		break;
	case 4:
		Mode_Four();
		break;
	case 5:
		Mode_Five();
		break;
	case 6:
		Mode_Six();
		break;

	case 7:
		Mode_Seven();
		break;
	default:
		Mode_Default();
		break;
	}
}

void Mode_Zero(){
	printf("\nMODE0 First Search MODE\n\r");

	//	一次探索スラローム走行
	MF.FLAG.WALL_CTRL = 1;
	MF.FLAG.SCND = 0;
	goal_x = GOAL_X;
	goal_y = GOAL_Y;
	adc_case = 0;
	ADC1_Start();

	//	基準値を取る
	while(adc_case<=2){
	}
	get_base_info();

	while(sensor_data.frontL < 120 || sensor_data.frontR< 120){
	}

	for(uint8_t i=0; i<3; i++){
		Back_LED_Light(1,0,1);
		HAL_Delay(150);
		Back_LED_Light(0,0,0);
		HAL_Delay(150);
	}

	MF.FLAG.MT_CTRL = 1;

	Buzzer_Scale(1000, 100);
	Maze_Search_Slalom(&trans_params, &trans_params_known, &rot_params);

    if (MF.FLAG.FAILSAFE)
    {
    	Warning2();
    }

	MF.FLAG.MT_CTRL = 0;

	Motor_Drive_Stop();
	HAL_Delay(2000);
	Buzzer_PWM_OUT( 99, 1000 );

	goal_x = goal_y = 0;

	MF.FLAG.MT_CTRL = 1;
	Maze_Search_Slalom(&trans_params,&trans_params_known,&rot_params);
	MF.FLAG.MT_CTRL = 0;

	goal_x = GOAL_X;
	goal_y = GOAL_Y;

	Motor_Drive_Stop();
}

void Mode_One(){
	printf("\nMODE0 First Search MODE\n\r");

	//	一次探索スラローム走行
	MF.FLAG.WALL_CTRL = 0;
	MF.FLAG.SCND = 0;
	goal_x = GOAL_X;
	goal_y = GOAL_Y;
	adc_case = 0;
	ADC1_Start();

	//	基準値を取る
	while(adc_case<=2){
	}

	get_base_info();

	while(sensor_data.frontL < 120 || sensor_data.frontR< 120){
	}

	for(uint8_t i=0; i<3; i++){
		Back_LED_Light(1,0,1);
		HAL_Delay(150);
		Back_LED_Light(0,0,0);
		HAL_Delay(150);
	}

	MF.FLAG.MT_CTRL = 1;

	Buzzer_Scale(1000, 100);
	Maze_Search_Slalom(&trans_params, &trans_params_known, &rot_params);
	MF.FLAG.MT_CTRL = 0;

	Motor_Drive_Stop();
	HAL_Delay(2000);

	goal_x = goal_y = 0;

	MF.FLAG.MT_CTRL = 1;
	Maze_Search_Slalom(&trans_params,&trans_params_known,&rot_params);
	MF.FLAG.MT_CTRL = 0;

	goal_x = GOAL_X;
	goal_y = GOAL_Y;

	Motor_Drive_Stop();
}


void Mode_Two(){
	//conf_route getwallinfo writemapとか全部消す
	printf("\nMODE1 Second Search MODE\n\r");

	//	一次探索スラローム走行
	MF.FLAG.WALL_CTRL = 1;

	MF.FLAG.SCND = 1;
	MF.FLAG.KNOWN = 0;
	Set_Search_Params(&trans_params, 400.0f, 2500.0f); //velocity ・ acceleration

	goal_x = GOAL_X;
	goal_y = GOAL_Y;
	adc_case = 0;
	ADC1_Start();

	//	基準値を取る
	while(adc_case<=2){
	}

	get_base_info();


	while(sensor_data.frontL < 300 || sensor_data.frontR< 300){
	}

	for(uint8_t i=0; i<3; i++){
		Back_LED_Light(1,0,1);
		HAL_Delay(150);
		Back_LED_Light(0,0,0);
		HAL_Delay(150);
	}


	MF.FLAG.MT_CTRL = 1;
	Maze_Search_Slalom(&trans_params, &trans_params_known, &rot_params);
	MF.FLAG.MT_CTRL = 0;

	Motor_Drive_Stop();
	HAL_Delay(2000);

	goal_x = goal_y = 0;

	MF.FLAG.MT_CTRL = 1;
	Maze_Search_Slalom(&trans_params,&trans_params_known,&rot_params);
	MF.FLAG.MT_CTRL = 0;


	goal_x = GOAL_X;
	goal_y = GOAL_Y;

	Motor_Drive_Stop();
}

void Mode_Three(){
	printf("\nMODE2 Second Search (Known) MODE\n\r");

	MF.FLAG.WALL_CTRL = 1;
	MF.FLAG.SCND = 1;
	MF.FLAG.KNOWN = 1;
	Set_Search_Params(&trans_params, 400.0f, 2500.0f); //velocity ・ acceleration

	goal_x = GOAL_X;
	goal_y = GOAL_Y;
	adc_case = 0;
	ADC1_Start();


	//	基準値を取る
	while(adc_case<=2){
	}

	get_base_info();

	while(sensor_data.frontL < 300 || sensor_data.frontR< 300){
	}

	for(uint8_t i=0; i<3; i++){
		Back_LED_Light(1,0,1);
		HAL_Delay(150);
		Back_LED_Light(0,0,0);
		HAL_Delay(150);
	}


	MF.FLAG.MT_CTRL = 1;
	Maze_Search_Slalom(&trans_params, &trans_params_known, &rot_params);
	MF.FLAG.MT_CTRL = 0;

	Motor_Drive_Stop();
	HAL_Delay(2000);

	goal_x = goal_y = 0;

	MF.FLAG.MT_CTRL = 1;
	Maze_Search_Slalom(&trans_params,&trans_params_known,&rot_params);
	MF.FLAG.MT_CTRL = 0;

	goal_x = GOAL_X;
	goal_y = GOAL_Y;

	Motor_Drive_Stop();

}

void Mode_Four(){
	printf("\nMODE3 Drive Check MODE\n\r");

	//Check Mode

	int8_t test_mode = mode_change();

	LED_Light1();
	HAL_Delay(1000);
	gyro_calib_flag = 1;
	   while(gyro_calib_flag){
	   }

	switch(test_mode){

	case 0:
		MF.FLAG.MT_CTRL = 1;
		MF.FLAG.WALL_CTRL = 0;
/*
		Adjust_Back();
		Adjust_Forward(trans_params.acceleration, trans_params.velocity);

		Forward_One_Section(trans_params.velocity);
		Decel_Half_Section(trans_params.acceleration, trans_params.velocity);
*/

		Adjust_Forward(trans_params.acceleration, trans_params.velocity);
		Slalom_R(trans_params.velocity);//オフセットスピード設定
		Slalom_L(trans_params.velocity);//オフセットスピード設定
		Slalom_L(trans_params.velocity);//オフセットスピード設定
		Slalom_L(trans_params.velocity);//オフセットスピード設定
		Slalom_L(trans_params.velocity);//オフセットスピード設定
		Forward_One_Section(trans_params.velocity);
		Slalom_R(trans_params.velocity);//オフセットスピード設定
		Slalom_R(trans_params.velocity);//オフセットスピード設定
		Slalom_R(trans_params.velocity);//オフセットスピード設定
		Forward_One_Section(trans_params.velocity);
		Slalom_R(trans_params.velocity);//オフセットスピード設定
		Decel_Half_Section(trans_params.acceleration, trans_params.velocity);

		MF.FLAG.MT_CTRL = 0;

		break;

	case 1:
		//Slalom右
		MF.FLAG.MT_CTRL = 1;
		MF.FLAG.WALL_CTRL = 0;

		Back_LED_Light(1,0,0);
//		Adjust_Forward(trans_params.acceleration, trans_params.velocity);
		Accel_Half_Section(trans_params.acceleration, trans_params.velocity);

		Slalom_R(trans_params.velocity);//オフセットスピード設定
      	Back_LED_Light(0,1,0);

		Decel_Half_Section(trans_params.acceleration, trans_params.velocity);
		Back_LED_Light(0,0,1);

		MF.FLAG.MT_CTRL = 0;

		Motor_Drive_Stop();
		break;

	case 2:
		//Slalom左
		MF.FLAG.MT_CTRL = 1;
		MF.FLAG.WALL_CTRL = 0;

		Back_LED_Light(1,0,0);
		Accel_Half_Section(trans_params.acceleration, trans_params.velocity);

		Slalom_L(trans_params.velocity);//オフセットスピード設定
		Back_LED_Light(0,1,0);

		Decel_Half_Section(trans_params.acceleration, trans_params.velocity);
		Back_LED_Light(0,0,1);

		MF.FLAG.MT_CTRL = 0;

		Motor_Drive_Stop();
		break;

	case 3:
		//Slalom右
		MF.FLAG.MT_CTRL = 1;
		MF.FLAG.WALL_CTRL = 0;

		Back_LED_Light(1,0,0);
		Accel_Half_Section(trans_params.acceleration, trans_params.velocity);
		for(uint8_t i=0; i<8;i++){
		Slalom_R(trans_params.velocity);//オフセットスピード設定
		Forward_One_Section(trans_params.velocity);

		Back_LED_Light(0,1,0);
		}
		Decel_Half_Section(trans_params.acceleration, trans_params.velocity);
		Back_LED_Light(0,0,1);

		MF.FLAG.MT_CTRL = 0;

		Motor_Drive_Stop();
		break;


	case 4:
		//Slalom左
		MF.FLAG.MT_CTRL = 1;
		MF.FLAG.WALL_CTRL = 0;

		Back_LED_Light(1,0,0);

		Accel_Half_Section(trans_params.acceleration, trans_params.velocity);

		for(uint8_t cnt=0;cnt<8;cnt++){
		Slalom_L(trans_params.velocity);//オフセットスピード設定
		Forward_One_Section(trans_params.velocity);

		Back_LED_Light(0,1,0);
		}

		Decel_Half_Section(trans_params.acceleration, trans_params.velocity);

		Back_LED_Light(0,0,1);

		MF.FLAG.MT_CTRL = 0;

		Motor_Drive_Stop();

		break;


	case 5:
		//Pivot左回転
		MF.FLAG.MT_CTRL = 1;
		MF.FLAG.WALL_CTRL = 0;
		Back_LED_Light(1,0,0);
		Accel_Half_Section(trans_params.acceleration, trans_params.velocity);
		Decel_Half_Section(trans_params.acceleration, trans_params.velocity);

		PivotL90(rot_params.acceleration,rot_params.velocity);

		Accel_Half_Section(trans_params.acceleration, trans_params.velocity);
		Decel_Half_Section(trans_params.acceleration, trans_params.velocity);

		Back_LED_Light(0,0,1);

		MF.FLAG.MT_CTRL = 0;
		Motor_Drive_Stop();
		break;

	case 6:
		//Pivot右回転
			MF.FLAG.MT_CTRL = 1;
			MF.FLAG.WALL_CTRL = 0;

			Back_LED_Light(1,0,0);
			Accel_Half_Section(trans_params.acceleration, trans_params.velocity);
			Decel_Half_Section(trans_params.acceleration, trans_params.velocity);

			PivotR90(rot_params.acceleration,rot_params.velocity);

			Accel_Half_Section(trans_params.acceleration, trans_params.velocity);
			Decel_Half_Section(trans_params.acceleration, trans_params.velocity);

			Back_LED_Light(0,0,1);

			MF.FLAG.MT_CTRL = 0;
			Motor_Drive_Stop();

			break;

	case 7:
		//drive check 4 区間
			printf("Drive Start!\r\n");

			MF.FLAG.MT_CTRL = 1;
			MF.FLAG.WALL_CTRL = 0;

			Accel_Half_Section(trans_params.acceleration, trans_params.velocity);
			Back_LED_Light(1,0,0);

			for(int i=0; i<5; i++){
				Forward_One_Section(trans_params.velocity);
				printf("Accel!\r\n");

				Back_LED_Light(0,1,0);
			}

				printf("Decel!\r\n");

			Decel_Half_Section(trans_params.acceleration, trans_params.velocity);
			Back_LED_Light(0,0,1);

			MF.FLAG.MT_CTRL = 0;

			Motor_Drive_Stop();

			break;
	}
}

void Mode_Five(){
	//Log Check
	printf("Log Check Mode!\r\n");
	Log_Shower();
}
void Mode_Six(){
}

void Mode_Seven(){
}

void Mode_Eight(){
}

void Mode_Default(){

	adc_case = 0;
		ADC1_Start();
		HAL_Delay(500);
		get_base_info();
		while(1){
		//	printf("OFF->FL:%4d,SL:%4d,SR:%4d,FR:%4d ON->FL:%4d,SL:%4d,SR:%4d,FR:%4d\r\n",
		//		Sensor_Off_Data[1],Sensor_Off_Data[3],Sensor_Off_Data[2],Sensor_Off_Data[0],
		//			Sensor_On_Data[1],Sensor_On_Data[3],Sensor_On_Data[2],Sensor_On_Data[0]);

			printf("FL:%4ld,SL:%4ld,SR:%4ld,FR:%4ld,diffL:%4ld, diffR:%4ld error:%4ld\r\n",sensor_data.frontL,sensor_data.sideL, sensor_data.sideR, sensor_data.frontR,
				difference.sideL,difference.sideR,((sensor_data.sideL - base_data.sideL) - (sensor_data.sideR - base_data.sideR)));
			HAL_Delay(50);
			}
}
