#ifndef __tim_H
#define __tim_H
#ifdef __cplusplus
 extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "main.h"

#define SET_PWM1(x) user_pwm_setvalue_friction1(x)
#define SET_PWM2(x) user_pwm_setvalue_friction1(x)

#define InitFrictionWheel()     \
        SET_PWM1(1000);          \
        SET_PWM2(1000);
#define SetFrictionWheelSpeed(x) \
        SET_PWM1(x);                \
        SET_PWM2(x);
void gun_init(void);
void user_pwm_setvalue_friction1(int);
extern TIM_HandleTypeDef htim4;
extern void _Error_Handler(char *, int);
void MX_TIM4_Init(void);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);


#ifdef __cplusplus
}
#endif
#endif 