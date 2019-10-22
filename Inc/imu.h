/*
 * imu.h
 *
 *  Created on: May 1, 2019
 *      Author: manboo
 */

#ifndef IMU_H_
#define IMU_H_

#define GYRO_DT 0.001f
#define FACTOR_TOLERANCE 0.99f
extern SPI_HandleTypeDef hspi2;

typedef struct{
	float omega_x;
	float omega_y;
	float omega_z;
	float accel_x;
	float accel_y;
	float accel_z;
}GYRO_DATA;

#ifdef MAIN_C_

	GYRO_DATA gyro_raw;
	GYRO_DATA gyro_true;
	GYRO_DATA gyro_offset;
	uint8_t set_flag;
    uint16_t gyro_calib_cnt;
	volatile uint8_t gyro_calib_flag;
//	float gyro_offset;

	float omega_z_buff[2]; //0->now 1->prev
/*
	float machine_angle[2];
	float tar_angle_z;
	float eps_angle_z[2];
	float sum_eps_angle_z;
	float tar_omega_z;
	float eps_omega_z[2];
	float sum_eps_omega_z;
*/
#else
	extern GYRO_DATA gyro_raw;
	extern GYRO_DATA gyro_true;
	extern GYRO_DATA gyro_offset;
	extern uint8_t set_flag;
    extern uint16_t gyro_calib_cnt;
    extern volatile uint8_t gyro_calib_flag;
//	extern float gyro_offset;

	extern float omega_z_buff[2]; //0->now 1->prev
	/*
	extern float machine_angle[2];
	extern float tar_angle_z;
	extern float eps_angle_z[2];
	extern float sum_eps_angle_z;
	extern float tar_omega_z;
	extern float eps_omega_z[2];
	extern float sum_eps_omega_z;
*/
#endif

void IMU_Init();
uint8_t read_byte(uint8_t);
void write_byte(uint8_t, uint8_t);
void ICM20602_Init();
float ICM20602_GYRO_READ(uint8_t);
float ICM20602_ACCEL_READ(uint8_t);
void ICM20602_DataUpdate();
void Calculate_Machine_Rotation();
void ICM20602_Calibration();

#endif /* IMU_H_ */
