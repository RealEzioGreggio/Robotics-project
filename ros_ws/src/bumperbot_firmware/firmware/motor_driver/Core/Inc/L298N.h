/*
 * L298N.h
 *
 *  Created on: Mar 27, 2026
 *      Author: lorenzo
 */

#ifndef INC_L298N_H_
#define INC_L298N_H_

#include "main.h"

typedef enum{

	CLOCKWISE = 0,
	ANTICLOCKWISE = 1
}Motor_direction_t;


typedef struct{
	/*TIMER*/
	TIM_HandleTypeDef *tim;
	uint32_t channel;

	/*PIN*/
	uint16_t dir_1;
	uint16_t dir_2;
	uint16_t en;
	GPIO_TypeDef *Port_dir_1;
	GPIO_TypeDef *Port_dir_2;
	GPIO_TypeDef *Port_en;

	/*SPIN DIRECTION*/
	Motor_direction_t direction;

	uint32_t ccr;


}L298N_t;

HAL_StatusTypeDef driverInit(L298N_t *m, TIM_HandleTypeDef *timer, uint32_t Channel, uint16_t dir_1, uint16_t dir_2, GPIO_TypeDef* Port_dir_1, GPIO_TypeDef* Port_dir_2);
void setdirection(L298N_t *m, Motor_direction_t direction);
void motor_set_pwm(L298N_t *m, float duty_cycle);
void motor_stop(L298N_t *m);


#endif /* INC_L298N_H_ */
