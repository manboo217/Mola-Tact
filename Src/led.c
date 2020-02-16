/*
 * led.c
 *
 *  Created on: 2019/05/02
 *      Author: manboo
 */

#include "global.h"

void Front_LED_Light(uint8_t Left,uint8_t Middle,uint8_t Right)
{
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,Left);	 //Front1
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,Middle); //Front2
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,Right);  //Front3
}



void Back_LED_Light(uint8_t Left,uint8_t Middle,uint8_t Right)
{
	  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,Left);//BackLeft
	  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,Middle);//BackMid
	  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,Right);//BackRight
}


void Red_LED_Light(uint8_t SL, uint8_t FL, uint8_t FR, uint8_t SR)
{
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,SL);//LED_SideLeft
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,FL);//LED_FrontLeft
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,FR);//LED_FrontRight
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,SR);//LED_SideRight
}


void LED_Light1(void){
	for(int i=0; i<3; i++){
			Back_LED_Light(1,0,1);
			HAL_Delay(150);
			Back_LED_Light(0,0,0);
			HAL_Delay(150);
		}
}

void Start_LED(void){
	uint8_t i = 0;
		for(i=0;i<3;i++){
			Front_LED_Light(0,0,0);
			HAL_Delay(100);
			Front_LED_Light(0,1,0);
			HAL_Delay(100);
		}
	Back_LED_Light(0,1,0);
}

void Warning1(void){
	while(1){
			Back_LED_Light(1,1,1);
			Buzzer_Scale(5000, 100);
			printf("%fV\r\n",battery_voltage);
			HAL_Delay(500);
			Back_LED_Light(0,0,0);
			Buzzer_Scale(0, 100);
			HAL_Delay(500);

		}
}


//FailSafe Warning
void Warning2(void){
	while(1){
			Back_LED_Light(1,0,1);
			Buzzer_Scale(7000, 100);
			HAL_Delay(200);
			Back_LED_Light(0,0,0);
			Buzzer_Scale(0, 100);
			HAL_Delay(200);
		}
}
