/*
 * tim.h
 *
 *  Created on: May 11, 2019
 *      Author: manboo
 */

#ifndef TIM_H_
#define TIM_H_

#define NORMAL 799
#define C_SCALE 3058
#define D_SCALE 2727
#define E_SCALE 2427


#ifdef MAIN_C_
uint16_t beep_time;
uint16_t buzzer_counter;

#else

extern uint16_t beep_time;
extern uint16_t buzzer_counter;

#endif


void Fun_Control( uint8_t );
void Buzzer_PWM_OUT( uint32_t, uint32_t);


#endif
