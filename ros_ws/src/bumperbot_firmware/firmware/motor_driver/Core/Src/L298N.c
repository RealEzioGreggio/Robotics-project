/*
 * L298N.c
 *
 *  Created on: Mar 27, 2026
 *      Author: lorenzo
 */

#include "L298N.h"

HAL_StatusTypeDef driverInit(L298N_t *m, TIM_HandleTypeDef *timer, uint32_t channel, uint16_t dir_1, uint16_t dir_2, GPIO_TypeDef* Port_dir_1, GPIO_TypeDef* Port_dir_2){

	HAL_StatusTypeDef ret = HAL_OK;

		if(m == NULL || timer == NULL){

			return HAL_ERROR;
		}

	m->tim = timer;
	m->channel = channel;
	m->dir_1 = dir_1;
	m->dir_2 = dir_2;
	m->Port_dir_1 = Port_dir_1;
	m->Port_dir_2 = Port_dir_2;

	/*SET THE FREQUENCY*/
	//__HAL_TIM_SET_AUTORELOAD(m->tim, 2250);
	//__HAL_TIM_SET_PRESCALER(m->tim, 0);

	/*SET CCR TO 0*/
	if(channel == TIM_CHANNEL_1){

		m->tim->Instance->CCR1 = 0;
	}
	else if (channel == TIM_CHANNEL_2) {

		m->tim->Instance->CCR2 = 0;
	}
	else if (channel == TIM_CHANNEL_3) {

		m->tim->Instance->CCR3 = 0;
	}
	else if (channel == TIM_CHANNEL_4) {

		m->tim->Instance->CCR4 = 0;
	}

	m->tim->Instance->EGR = TIM_EGR_UG;

	setdirection(m, CLOCKWISE);

	ret = HAL_TIM_PWM_Start(m->tim, m->channel);

	if(ret != HAL_OK){

		return HAL_ERROR;
	}

	return HAL_OK;
}


void setdirection(L298N_t *m, Motor_direction_t direction){

	if(direction == CLOCKWISE){

		HAL_GPIO_WritePin(m->Port_dir_1, m->dir_1, GPIO_PIN_SET);
		HAL_GPIO_WritePin(m->Port_dir_2, m->dir_2, GPIO_PIN_RESET);

	} else {

		HAL_GPIO_WritePin(m->Port_dir_1, m->dir_1, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(m->Port_dir_2, m->dir_2, GPIO_PIN_SET);

	}
}

void motor_set_pwm(L298N_t *m, float duty_cycle){


	if(duty_cycle >= 0){

		m->direction = CLOCKWISE;
	}else {

		m->direction = ANTICLOCKWISE;
		duty_cycle = -(duty_cycle);
	}

	setdirection(m, m->direction);

	m->ccr = (uint32_t)(duty_cycle*(float)(1 + m->tim->Instance->ARR));


	/*Applico l'azione di controllo sul canale del timer che gli hai passato nell'init*/
	if(m->channel == TIM_CHANNEL_1){

		m->tim->Instance->CCR1 = m->ccr;
	}
	else if (m->channel == TIM_CHANNEL_2) {

		m->tim->Instance->CCR2 = m->ccr;
	}
	else if (m->channel == TIM_CHANNEL_3) {

		m->tim->Instance->CCR3 = m->ccr;
	}
	else if (m->channel == TIM_CHANNEL_4) {

		m->tim->Instance->CCR4 = m->ccr;
	}

	m->tim->Instance->EGR = TIM_EGR_UG;

}

void motor_stop(L298N_t *m){

	if(m->channel == TIM_CHANNEL_1){

			m->tim->Instance->CCR1 = 0;
	}
	else if (m->channel == TIM_CHANNEL_2) {

		m->tim->Instance->CCR2 = 0;
	}
	else if (m->channel == TIM_CHANNEL_3) {

		m->tim->Instance->CCR3 = 0;
	}
	else if (m->channel == TIM_CHANNEL_4) {

		m->tim->Instance->CCR4 = 0;
	}

	m->tim->Instance->EGR = TIM_EGR_UG;

}

