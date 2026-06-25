/*
 * Encoder.h
 *
 *  Created on: Jan 29, 2025
 *      Author: loryx
 */

#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_

#include "main.h"

typedef enum{

	ENCODER_RES_2 = 2,
	ENCODER_RES_4 = 4
}encoder_resolution_t;

typedef struct{

	TIM_HandleTypeDef *tim;
	uint32_t last_sampling_t;
	uint32_t last_count_pos;
	uint32_t last_count_vel;
	int ppr;
	encoder_resolution_t resolution;

	float position_rad;
	float position_deg;
	float velocity_pps;
	float position_counts;



}Encoder_t;


HAL_StatusTypeDef encoder_init(Encoder_t *e, TIM_HandleTypeDef *tim, encoder_resolution_t resolution, int ppr);

float encoder_get_velocity_rads(Encoder_t *e);
float encoder_get_velocity_rpm(Encoder_t *e);
float encoder_get_velocity_rps(Encoder_t *e);
float encoder_get_position_deg(Encoder_t *e);
float encoder_get_position_rad(Encoder_t *e);




#endif /* INC_ENCODER_H_ */
