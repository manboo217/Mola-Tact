/*
 *  tim.c
 *
 *  Created on: 2019/09/21
 *      Author: manboo
 */

#include "global.h"

static int32_t buzzer_period_buff = 0;

void Fun_Control( uint8_t mode )
{
	  int32_t pwm = 0;

	  if(HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4)!=HAL_OK)
	  {
		  printf("Ch4 Start Error\r\n");
		  Error_Handler();
	  }
	  pwm = 300;

/*
  if ( mode == 1 ) {
    pwm = 300;
  }
  else {
	pwm = 0;
  }
  */
  // set duty
  __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,pwm);
}

///////////////////////////////////////////////////////////////////////
// set buzzer pwm
// [argument] pwm,period (0 ~ 1000)
// [Substitutiong] nothing
// [return] nothing
///////////////////////////////////////////////////////////////////////
void Buzzer_PWM_OUT( uint32_t pwm, uint32_t period )
{
  if ( buzzer_period_buff != period ){
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = period;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 99;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
    {
      Error_Handler();
    }
    buzzer_period_buff = period;
  }

  __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,pwm);

	if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1) != HAL_OK){
		Error_Handler();
	}
}



void Buzzer_Scale( uint16_t scale, uint16_t time_beep )
{
	Buzzer_PWM_OUT( 99, scale );
    buzzer_counter = 0;
    beep_time = time_beep;
}

void Buzzer_Control( void )
{
  if ( buzzer_counter > beep_time ){
    Buzzer_PWM_OUT( 0, NORMAL );
  } else {
    buzzer_counter++;
  }
}
/*
void buzzermodeSelect( int8_t mode )
{
  switch( mode ){
    case 0:
      break;

    case 1:
      buzzerSetMonophonic( C_SCALE, 200 );
      break;

    case 2:
      buzzerSetMonophonic( D_SCALE, 200 );
      break;

    case 3:
      buzzerSetMonophonic( E_SCALE, 200 );
      break;

    case 4:
      buzzerSetMonophonic( F_SCALE, 200 );
      break;

    case 5:
      buzzerSetMonophonic( G_SCALE, 200 );
      break;

    case 6:
      buzzerSetMonophonic( A_SCALE, 200 );
      break;

    case 7:
      buzzerSetMonophonic( B_SCALE, 200 );
      break;

    case 8:
      buzzerSetMonophonic( C_H_SCALE, 200 );
      break;

    default:
      break;
  }
}*/
