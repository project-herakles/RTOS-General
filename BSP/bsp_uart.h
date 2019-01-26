/**
 ***************************************(C) COPYRIGHT 2018 DJI***************************************
 * @file       bsp_uart.h
 * @brief      this file contains the common defines and functions prototypes for 
 *             the bsp_uart.c driver
 * @note         
 * @Version    V1.0.0
 * @Date       Jan-30-2018      
 ***************************************(C) COPYRIGHT 2018 DJI***************************************
 */

#ifndef __BSP_UART_H__
#define __BSP_UART_H__

#include "usart.h"
#include "keyboard_def.h"

#define UART_RX_DMA_SIZE (1024)
#define DBUS_MAX_LEN     (50)
#define DBUS_BUFLEN      (18)
#define DBUS_HUART       huart1 /* for dji remote controler reciever */
#define SPEED_CONST			 0.00068 //the const for rc_dealer to calcu the speed ref
#define ROTATION_CONST 	 0.00045 //the const for rc_dealer to calcu the rotate ref

/** 
  * @brief  remote control information
  */

typedef __packed struct
{
	int16_t forward_back_direction;
	int16_t left_right_direction;
	int16_t rotation_direction;
	int16_t speed_mood;
}KbCtrl;

#define NO_KEY_PRESSED(Control)			Control.forward_back_direction && Control.left_right_direction && Control.rotation_direction			
	
typedef __packed struct
{
  /* rocker channel information */
  int16_t ch1;
  int16_t ch2;
  int16_t ch3;
  int16_t ch4;
	
	/*keyboard-mouse information*/
	int16_t ch5;//mouse-y
	int16_t ch6;//mouse-x
	int16_t ch7;//mouse-z
	KbCtrl kb_ctrl;//buffer channel 14, for chassis direction control
	uint8_t kb_othe;//buffer channel 15, for other functions

  /* left and right lever information */
  uint8_t sw1;
  uint8_t sw2;
} rc_info_t;

typedef __packed struct
{
	int16_t forward_back_speed_ref;
	int16_t left_right_speed_ref;
	int16_t rotation_speed_ref;
} chassis_ctrl;

typedef __packed struct
{
	int16_t forward_back_speed_ref;
	int16_t left_right_speed_ref;
	int16_t rotation_speed_ref;
} km_info_t;

void uart_receive_handler(UART_HandleTypeDef *huart);
void dbus_uart_init(void);
void USART_SendData(USART_TypeDef* USARTx, uint16_t Data);
void rc_dealler(const rc_info_t *);
#endif

