#include "encoder.h"

extern uint32_t can_count;

void EncoderProcess(volatile Encoder *v, uint8_t* msg)
{
	int i=0;
	int32_t temp_sum = 0;    
	v->last_raw_value = v->raw_value;
	v->raw_value = (msg[0]<<8)|msg[1];
	v->diff = v->raw_value - v->last_raw_value;
	if(can_count < 50)
	{
		v->ecd_raw_rate = 0;
	}
	else
	{
		if(v->diff < -7000)    
		{
			v->round_cnt++;
			v->ecd_raw_rate = v->diff + 8192;
		}
		else if(v->diff>7000)
		{
			v->round_cnt--;
			v->ecd_raw_rate = v->diff- 8192;
		}		
		else
		{
			v->ecd_raw_rate = v->diff;
		}
	}
	
	v->ecd_value = v->raw_value + v->round_cnt * 8192;
	
	//v->ecd_angle = (float)(v->raw_value - v->ecd_bias)*360/8192 + v->round_cnt * 360;
	v->ecd_angle = (float)(v->raw_value - v->ecd_bias)*360/8192.0f;
	v->rate_buf[v->buf_count++] = v->ecd_raw_rate;
	if(v->buf_count == RATE_BUF_SIZE)
	{
		v->buf_count = 0;
	}
	
	for(i = 0;i < RATE_BUF_SIZE; i++)
	{
		temp_sum += v->rate_buf[i];
	}
	v->filter_rate = (int32_t)(temp_sum/RATE_BUF_SIZE);		

}
void GetEncoderBias(volatile Encoder *v,CAN_RxHeaderTypeDef *rxHeader,uint8_t* msg)
{

            v->ecd_bias = (msg[0]<<8)|msg[1]; 
            v->ecd_value = v->ecd_bias;
            v->last_raw_value = v->ecd_bias;
            v->temp_count++;
}
