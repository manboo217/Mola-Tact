/*
 * imu.c
 *
 *  Created on: May 11, 2019
 *      Author: manboo
 */

#include "global.h"
#include "stm32f4xx_hal_spi.h"

void IMU_Init(void){

	set_flag = 0;
	rot_machine.velocity = 0.0f;
	rot_machine.distance = 0.0f;
	gyro_calib_flag = 0;
	gyro_calib_cnt = 0;
	omega_z[0] = omega_z[1] = 0.0f; //0->now 1->prev
	machine_angle[0] = machine_angle[1] = 0.0f;
	tar_angle_z = 0.0;
	eps_angle_z[0] = eps_angle_z[1] = 0.0f;
	sum_eps_angle_z = 0.0f;

}

//+++++++++++++++++++++++++++++++++++++++++++++++
//read_byte関数
/*SPI Operational Features
1.Data is delivered MSB first and LSB last
2.Data is latched on the rising edge of SCLK
3.Data should be transitioned on the falling edge of SPC
4.The maximum frequency of SPC is 10MHz
5.SPI read and write operations are completed in 16 or more clock cycles(two or more bytes.)
The first byte conains the SPI Adress
The following bytes contain the SPI data
The first bit of the first byte contains the Read/Write bit and indicates the Read(1) or Weite(0) operation.
The following 7 bits is the Resister Address.
*/
//+++++++++++++++++++++++++++++++++++++++++++++++

uint8_t read_byte( uint8_t reg )
{
	uint8_t ret,val;
	HAL_GPIO_WritePin( GPIOB, GPIO_PIN_7, GPIO_PIN_RESET); //cs = 0;
	ret = reg | 0x80;
	HAL_SPI_Transmit(&hspi2, &ret,1,100);
	HAL_SPI_Receive(&hspi2,&val,1,100);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET); //cs = 1;
	return val;
}

//+++++++++++++++++++++++++++++++++++++++++++++++
//write_byte関数
/*SPI Operational Features
1.Data is delivered MSB first and LSB last
2.Data is latched on the rising edge of SCLK
3.Data should be transitioned on the falling edge of SPC
4.The maximum frequency of SPC is 10MHz
5.SPI read and write operations are completed in 16 or more clock cycles(two or more bytes.)
The first byte conains the SPI Adress
The following bytes contain the SPI data
The first bit of the first byte contains the Read/Write bit and indicates the Read(1) or Weite(0) operation.
The following 7 bits is the Resister Address.
*/
//+++++++++++++++++++++++++++++++++++++++++++++++

void write_byte( uint8_t reg, uint8_t val )
{
	uint8_t ret;
	ret = reg & 0x7F ;
	HAL_GPIO_WritePin( GPIOB, GPIO_PIN_7, GPIO_PIN_RESET ); //cs = 0;
	HAL_SPI_Transmit( &hspi2, &ret,1,100 );
	HAL_SPI_Transmit( &hspi2, &val,1,100 );
	HAL_GPIO_WritePin( GPIOB, GPIO_PIN_7, GPIO_PIN_SET ); //cs = 1;
}

//+++++++++++++++++++++++++++++++++++++++++++++++
//ICM20602_Init
//+++++++++++++++++++++++++++++++++++++++++++++++

void ICM20602_Init( void )
{
	uint8_t who_am_i = 0;
	who_am_i = read_byte(0x75);	// check WHO_AM_I (0x75)
	printf( "who_am_i = 0x%x\r\n",who_am_i ); 	// Who am I = 0x12

	if (who_am_i != 0x12){	// recheck
		HAL_Delay(100);
		who_am_i = read_byte(0x75);

		if (who_am_i != 0x12){
			printf( "gyro_error\r\n\n");
			while(1){
				}
		}
	}

 //PWR_MIGHT_1 0x6B
	write_byte( 0x6B, 0x00 );	//Set pwr might
	HAL_Delay(50);
 //PWR_MIGHT_2 0x6C
	write_byte( 0x6C, 0x00 );
	HAL_Delay(50);
 //set gyro config
 //GYRO_CONFIG 0x1B
	write_byte( 0x1B, 0x18 ); // use 2000 dps
	HAL_Delay(50);
 //ACCEL_CONFIG 0x1C
	write_byte( 0x1B, 0x18 ); // use pm 16g
	HAL_Delay(50);

	 set_flag = 1;
}


float ICM20602_GYRO_READ( uint8_t H_reg )
{
	int16_t data = (int16_t)( ((uint8_t)read_byte(H_reg) << 8) | (uint8_t)read_byte(H_reg+1) );
	float omega = (float)(data / 16.4f); //[deg/s] FS_SEL=3-> Scale Factor=16.4[LSB/(dps)]
	return omega;
}


float ICM20602_ACCEL_READ( uint8_t H_reg )
{
	int16_t data = (int16_t)( ((uint8_t)read_byte(H_reg) << 8) | (uint8_t)read_byte(H_reg+1) );
	float accel = (float)(data / 2048.0f);
	return accel;
}

void ICM20602_DataUpdate(void)
{

	if ( set_flag == 1 ){
	// get yawrate
	gyro_raw.omega_x = -1 * ICM20602_GYRO_READ( 0x45 ) * FACTOR_TOLERANCE; //-90度座標回転
	gyro_raw.omega_y = ICM20602_GYRO_READ( 0x43 ) * FACTOR_TOLERANCE;
	gyro_raw.omega_z = ICM20602_GYRO_READ( 0x47 ) * FACTOR_TOLERANCE;

	// get accel
	gyro_raw.accel_x = -1 * ICM20602_ACCEL_READ( 0x3D );
	gyro_raw.accel_y = ICM20602_ACCEL_READ( 0x3B );
	gyro_raw.accel_z = ICM20602_ACCEL_READ( 0x3F );

	//True Value(Consider Offset)
	gyro_true.omega_x = gyro_raw.omega_x - gyro_offset.omega_x;
	gyro_true.omega_y = gyro_raw.omega_y - gyro_offset.omega_y;
	gyro_true.omega_z = gyro_raw.omega_z - gyro_offset.omega_z;
	gyro_true.accel_x = gyro_raw.accel_x - gyro_offset.accel_x;
	gyro_true.accel_y = gyro_raw.accel_y - gyro_offset.accel_y;
	gyro_true.accel_z = gyro_raw.accel_z - gyro_offset.accel_z;


	}
}

void Calculate_Machine_Rotation(void)
{
	rot_machine.velocity = gyro_true.omega_z;
	omega_z_buff[0] = rot_machine.velocity;
	rot_machine.distance += (omega_z_buff[0] + omega_z_buff[1]) /2.0f * GYRO_DT;
	omega_z_buff[1] = omega_z_buff[0];
}

void ICM20602_Calibration(void)
{
	if(gyro_calib_flag){
	gyro_offset.omega_z += gyro_raw.omega_z;
	gyro_calib_cnt ++;

		if(gyro_calib_cnt ==999){
			gyro_offset.omega_z /= 1000;
			gyro_calib_flag = 0;
			printf("gyro calibration finished! offset=%f,flag=%d\n\r", gyro_offset.omega_z, gyro_calib_flag);
			Back_LED_Light(0,1,0);
		}
	}
}
