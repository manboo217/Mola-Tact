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
