#ifndef _ENCODER_H_
#define _ENCODER_H_

#include "main.h"
#include "stm32f4xx_hal.h"

#define RATE_BUF_SIZE 6
typedef struct{
	int32_t raw_value;   						
	int32_t last_raw_value;					
	int32_t ecd_value;                    
	int32_t diff;											
	int32_t temp_count;              
	uint8_t buf_count;						
	int32_t ecd_bias;									
	int32_t ecd_raw_rate;								
	float rate_buf[RATE_BUF_SIZE];	
	int32_t round_cnt;							
	int32_t filter_rate;							
	float ecd_angle;			
	float last_ecd_angle;
	float ecd_speed;
}Encoder;

void EncoderProcess(volatile Encoder *v, uint8_t* msg);
void GetEncoderBias(volatile Encoder *v,CAN_RxHeaderTypeDef *rxHeader,uint8_t* msg);

#endif
