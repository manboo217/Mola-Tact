/*
 * sensor.c
 *
 *  Created on: 2019/05/18
 *      Author: manboo
 */

#include "global.h"
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern DMA_HandleTypeDef hdma_adc1;


void ADC1_Start(void){
	adc_case = 0;
	   if (HAL_ADC_Start_DMA(&hadc1,
	                         (uint32_t *)ADCBuffer,
	                         sizeof(ADCBuffer)
	                        ) != HAL_OK)
	   {
	     Error_Handler();
	   }
}

void ADC_Stop(void){
	adc_case = 3;
	HAL_ADC_Stop_DMA(&hadc1);
}


void Get_Sensor_Data(int16_t *adc_case){

	volatile int8_t i;

	switch(*adc_case){

	case 0://LED消灯時のデータ格納

		HAL_ADC_Stop_DMA(&hadc1);

		Sensor_Off_Data[0] = ADCBuffer[0];
		Sensor_Off_Data[1] = ADCBuffer[1];
		Sensor_Off_Data[2] = ADCBuffer[2];
		Sensor_Off_Data[3] = ADCBuffer[3];

		Red_LED_Light(1,0,0,1);

		for(i=0;i<120;i++){//横LEDの立ち上がりを待つ

		}

		*adc_case = 1;//	タスクポインタ進める

		//AD変換をスタートさせて終了
		if (HAL_ADC_Start_DMA(&hadc1,
		                         (uint32_t *)ADCBuffer,
		                         sizeof(ADCBuffer)
		                        ) != HAL_OK)
		   {
		     Error_Handler();
		   }

		break;


	case 1://Side Value
		Red_LED_Light(0,0,0,0);//	消灯

		HAL_ADC_Stop_DMA(&hadc1);

		Sensor_On_Data[2] = ADCBuffer[2];
		Sensor_On_Data[3] = ADCBuffer[3];

		sensor_data.sideR  = Sensor_On_Data[2] - Sensor_Off_Data[2];
		sensor_data.sideL  = Sensor_On_Data[3] - Sensor_Off_Data[3];


		Red_LED_Light(0,1,1,0);//frontLED ON

		for(i=0;i<125;i++){ //130×
		}

		*adc_case = 2;

		   if (HAL_ADC_Start_DMA(&hadc1,
		                         (uint32_t *)ADCBuffer,
		                         sizeof(ADCBuffer)
		                        ) != HAL_OK)
		   {
		     Error_Handler();
		   }

		break;


	case 2://Front Value
		Red_LED_Light(0,0,0,0);

		HAL_ADC_Stop_DMA(&hadc1);

		Sensor_On_Data[0] = ADCBuffer[0];
		Sensor_On_Data[1] = ADCBuffer[1];

		sensor_data.frontR = Sensor_On_Data[0] - Sensor_Off_Data[0];
		sensor_data.frontL = Sensor_On_Data[1] - Sensor_Off_Data[1];

		*adc_case = 3;

		for(i=0;i<100;i++){
		}

		break;

	default:
		break;
	}
}


void ADC_Convert_Check(void){
	  if ( adc_case == 3 ){
	      adc_case = 0;
	      update_sensor_data();
	      HAL_ADC_Start_DMA( &hadc1, (uint32_t *)ADCBuffer, sizeof(ADCBuffer) );
	   }
}


void get_base_info(void){
	base_data.sideL = sensor_data.sideL;
	base_data.sideR = sensor_data.sideR;
	printf("base_L=%4ld, base_R=%4ld\n\r",base_data.sideL, base_data.sideR);

}


void Set_Threshold(sensor *threshold, uint32_t frontL, uint32_t sideL, uint32_t sideR, uint32_t frontR)
{
	threshold->frontL = frontL;
	threshold->sideL = sideL;
	threshold->sideR = sideR;
	threshold->frontR = frontR;
}

void get_wall_info(void){

	//----	壁情報の初期化----
	wall_info = 0x00;									//壁情報を初期化
	//----	前壁を見る----
	if(sensor_data.frontL > threshold.frontL){
		//AD値が閾値より大きい（=壁があって光が跳ね返ってきている）場合
		wall_info |= 0x88;								//壁情報を更新
	    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,1);//BackMid
	}
	else{
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,0);//BackLeft
	}

	//----	右壁を見る----
	if(sensor_data.sideR > threshold.sideR){
		//AD値が閾値より大きい（=壁があって光が跳ね返ってきている）場合
		wall_info |= 0x44;								//壁情報を更新
	  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,1);//BackRight
	}
	else{
	  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,0);//BackRight
	}
	//----	左壁を見る----
	if(sensor_data.sideL > threshold.sideL){
		//AD値が閾値より大きい（=壁があって光が跳ね返ってきている）場合
		wall_info |= 0x11;								//壁情報を更新
	   HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,1);//BackLeft

	}
	else{
	   HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,0);//BackLeft
	}

}


void update_sensor_data( void )
{
	//	前回の取得値との差分を取る
	difference.sideR = sensor_data.sideR - sensor_old.sideR;
	if(difference.sideR < 0) difference.sideR *= -1;
	difference.sideL = sensor_data.sideL - sensor_old.sideL;
	if(difference.sideL < 0) difference.sideL *= -1;


  if ( sensor_data.sideL < threshold.sideL ){
    wall_data.sideL = 0;

  } else {
    wall_data.sideL = 1;
  }

  if ( sensor_data.sideR < threshold.sideR ){
    wall_data.sideR = 0;
  } else {
    wall_data.sideR = 1;
  }

  if ( sensor_data.frontR < threshold.frontR ){
    wall_data.frontR = 0;
  } else {
    wall_data.frontR = 1;
  }

// Log_Saver();
 sensor_old.sideR =  sensor_data.sideR;
 sensor_old.sideL =  sensor_data.sideL;

}


float Battery_Voltage( void )
{
  int16_t battery_analog = 0;
  float battery_voltage;

  HAL_ADC_Start( &hadc2 );
  HAL_ADC_PollForConversion( &hadc2,100 );  // trans
  battery_analog = HAL_ADC_GetValue( &hadc2 );   // get value
  HAL_ADC_Stop( &hadc2 );

  battery_voltage = (float)(battery_analog /1024.0f * 3.3f *3.2f);

  return battery_voltage;
}


void Battery_Monitor( void ){
	HAL_ADC_Start(&hadc2);
	battery_voltage = Battery_Voltage();
	printf("%fV\r\n",battery_voltage);
	HAL_Delay(500);

	if(battery_voltage < 7.5f){
		Warning1();
	}
}
