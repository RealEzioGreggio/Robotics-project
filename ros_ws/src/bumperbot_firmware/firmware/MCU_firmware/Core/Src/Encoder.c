/*
 * Encoder.c
 *
 *  Created on: Jan 29, 2025
 *      Author: loryx
 */


#include "Encoder.h"
#include <math.h>

#define ENCODER_OFFSET	(1<<15)



HAL_StatusTypeDef encoder_init(Encoder_t *e, TIM_HandleTypeDef *tim, encoder_resolution_t resolution, int ppr){

	HAL_StatusTypeDef ret = HAL_OK;

	if(e == NULL ||tim == NULL ){

		return HAL_ERROR;
	}

	e->tim = tim;

	ret = HAL_TIM_Encoder_Start(e->tim, TIM_CHANNEL_ALL);
	e->tim->Instance->CNT = 1<<15;

	if(ret != HAL_OK){

		return ret;
	}


	e->resolution = resolution;
	e->velocity_pps = 0.0;
	e->last_sampling_t = HAL_GetTick();
	e->last_count_vel = tim->Instance->CNT;
	e->last_count_pos = tim->Instance->CNT;
	e->ppr = ppr;

	return ret;

}


inline static void __encoder_update(Encoder_t *e)
{
    uint32_t now = HAL_GetTick();

    if(now == e->last_sampling_t){
        return;
    }

    int32_t cur_cnt = (int32_t)e->tim->Instance->CNT;

    int32_t diff = (int16_t)(cur_cnt - e->last_count_vel);

    float dt = (float)(now - e->last_sampling_t) / 1000.0f;

    float cur_velocity = ((float)diff / dt) / (float)e->resolution;

    e->velocity_pps = 0.15f * cur_velocity + 0.85f * e->velocity_pps;

    e->last_count_vel = cur_cnt;
    e->last_sampling_t = now;
}

float encoder_get_velocity_rps(Encoder_t *e){

	__encoder_update(e);
	return (e->velocity_pps / (float) e->ppr);

}

float encoder_get_velocity_rpm(Encoder_t *e){

	__encoder_update(e);
	return (e->velocity_pps / (float)e->ppr) * 60.0;

}

float encoder_get_velocity_rads(Encoder_t *e){

	__encoder_update(e);
	return (e->velocity_pps / (float)e->ppr) *(2.0*M_PI);
}

/*Non è un vero e prorpio getter ma va bene così*/
float encoder_get_position_deg(Encoder_t *e){

	uint32_t cur_cnt = e->tim->Instance->CNT;
	return (((float)cur_cnt - ENCODER_OFFSET)/(float)e->ppr/(float)e->resolution*360.0);
}

float encoder_get_position_rad(Encoder_t *e){

	int32_t cur_cnt = (int32_t)e->tim->Instance->CNT;
	int32_t diff = cur_cnt - e->last_count_pos;

	    // gestione overflow 16-bit (se TIM è a 16 bit)
	    if (diff > 32767)
	        diff -= 65536;
	    else if (diff < -32768)
	        diff += 65536;

	    e->position_counts += diff;
	    e->last_count_pos = cur_cnt;

	    return ((float)e->position_counts / e->ppr / e->resolution) * (2.0f * M_PI);
}


