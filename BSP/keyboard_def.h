/*
 * @brief      this file contains the macro for values of keys
 * @note         
 * @Version    V1.0.0
 * @Date       NOV-11-2018      
 ***************************************(C) COPYRIGHT 2018 DJI***************************************
 */
 
 
#define FAST_SPEED								200 // speed ref for keyboard control
#define NORMAL_SPEED							100 // speed ref for keyboard control
#define ROTATION_SPEED						150 // speed ref for keyboard control
#define STOP_SPEED									0 // speed ref for keyboard control

enum keys_14
{
	W = 0x01, S = 0x02, A = 0x04, D = 0x08, Shift = 0x10, Ctrl = 0x20, Q = 0x40, E = 0x80
};

enum keys_15
{
	R = 0x01, F = 0x02, G = 0x04, Z = 0x08, X = 0x10, C = 0x20, V = 0x40, B = 0x80
};


enum forward_back_direction
{
	Forward = 1, Backward = -1, FB_Stop = 0
};

enum right_left_direction
{
	Right = 1, Left = -1, RL_Stop = 0
};

enum rotation_direction
{
	CW = 1, CCW = -1, RT_Stop = 0
};
