#ifndef CONTROL_H
#define CONTROL_H

#include "bool.h"
#include "keyboard.h"

#define ENGINEER

#define HIGH_SPEED        		256       //Maximum speed when chassis is in HIGH-SPEED-MODE
#define LOW_SPEED         		128       //Maximum speed when chassis is in LOW-SPEED-MODE
#define INI_SPEED         		1         //Initialization of chassis movement before speeding up when KEY is in use
#define ROTATION_SPEED    		150       //Const for chassis rotation when KEY is in use
#define SWING_SPEED       		90        //Const for dotch
#define SPEED_CONST		  			0.00068f    //Const for movement ref when RC is in use
#define ANGLE_CONST       		0.025f    //Const for rotation ref when MOUSE is in use
#define ROTATION_CONST_PITCH  0.000045f  //Const for gimbal pitch rotation ref when RC is in use
#define ROTATION_CONST_YAW		0.000045f//Const for gimbal yaw rotation ref when RC is in use
#define BACK_CONST        		0.7f      //Const for rotation ref when chassis is going to follow gimbal
#define ANGLE_ERROR       		2         //(Half of) Range of the available angle between chassis and gimbal
#define ANGLE_INBET       		5         //Constant for PID of chassis follow gimbal mode (only p in use now)
#define RAISING_HEAD					50				//positional signal when the raising mechanism reaches the top
#define	RAISING_BOTTOM				0					//positional signal when the raising mechanism reacher the bottom

#define CW                1
#define CCW              -1

typedef   signed short     int int16_t;
typedef   signed          char int8_t;
typedef unsigned          char uint8_t;
typedef unsigned short     int uint16_t;

//Initlization definition
#define SIGINIT     {0,0}
#define KbStateInit {{False,True},{False,True},{False,True},{False,True}}
#define KbInit      {SIGINIT,SIGINIT,SIGINIT,SIGINIT,False,False,SIGINIT,False,0,KbStateInit}
#define RcInit      {0,0,0,0,0,0,0,0,0,0,KbInit,0,0}
#define chassisInit {0,0,0}
#define gimbalInit  {0,0}
#define CtrlInit    {.state={False,False,False,False},.gimbal_ctrl_ptr=&GimbalSig,.chassis_ctrl_ptr=&ChassisSig}

#define OpenMagazineLid(a)	(int16_t)a*90
//a is the func signal magazineLid

#ifndef  ENGINEER
#define FuncInit    {0,0,0,0,0}
#else
#define FuncInit    {0,0,0,0}
#endif

typedef struct
{
    #ifndef   ENGINEER
    uint8_t fWheel : 1;
		uint8_t magazineLid : 1; //1=>90degree; 0=>0degree
    uint8_t shoot  : 2;
    uint8_t shootMode : 1;
		uint8_t raising : 2; //00=>hold, 10=>up, 01=>down
		#else
		uint8_t grabRobot : 1;
		uint8_t grabBullet : 1;
		uint8_t releaseRobot : 1;
		uint8_t releaseBullet : 1;
    #endif
}func_t;

//SHOOT: 00(0)----N; 01(1)----one bullet one time; 10(2)----three bullet one time; 11(2)----keep shooting
//SHOOTMODE: 0(0)----Waiting for signal/Done; 1(1)----Firing
//HOLDER/FECTH_ROBOT/GET_BULLET: 00(0)----Retraction; 01(1)----Extending; 10(-2)----Retracting; 11(-1)----Extended
                            //   OUTPUT_TO_LINEAR_MOTOR: X-X>>1
typedef struct
{
	uint8_t n;
	uint8_t p;
}statesignal_t;

typedef struct
{
    bool_t w_state[2];
    bool_t a_state[2];
    bool_t s_state[2];
    bool_t d_state[2];
} keyState_t;

typedef struct
{
    keysignal_t W;
    keysignal_t A;
    keysignal_t S;
    keysignal_t D;
    bool_t Q : 1;
    bool_t E : 1;
    keysignal_t High; //High speed
    bool_t Swing; //Swing
    uint8_t othKey; //Other Keys
    keyState_t keyState; 
} KbCtrl_t;

typedef struct
{
    uint8_t state;
	/* rocker channel information */
	int16_t ch1;
	int16_t ch2;
	int16_t ch3;
	int16_t ch4;
		
	/*keyboard-mouse information*/
	int16_t ch5;									//mouse-y
	int16_t ch6;									//mouse-x
	int16_t ch7;									//mouse-z
	uint8_t ch8;									//mouse-l
	uint8_t ch9;									//mouse-r
	KbCtrl_t kb_ctrl;

	/* left and right lever information */
	statesignal_t sw1;
	uint8_t sw2;
} rc_info_t;

typedef struct
{
	int16_t forward_back_speed_ref;
	int16_t left_right_speed_ref;
	int16_t rotation_speed_ref;
} chassis_ctrl_t;

typedef struct
{
	int16_t horizontal_angle_ref;
	int16_t vertical_angle_ref;
} gimbal_ctrl_t;

typedef struct
{
    bool_t state[4]; //auto-maunal state & G-C state
    func_t * func_ptr;
    gimbal_ctrl_t * gimbal_ctrl_ptr;
    chassis_ctrl_t * chassis_ctrl_ptr;
} ctrl_info_t;

typedef struct
{
    uint8_t S : 1;
    uint8_t M : 1;
    uint8_t B : 1;
    uint8_t S1: 1;
    uint8_t S2: 1;
}OneBit;

void chassisGimbalInit(ctrl_info_t *); 

void rcDealler(ctrl_info_t *, const int16_t *, rc_info_t *, int16_t*); //STEP1: store the given signal into the rc_info_t struct
void refCalc(rc_info_t *, ctrl_info_t *, int16_t); //STEP2: calculate the ref
//void funcCtrl(rc_info_t *, func_t *); //STEP3: conduct robot's functions, including shooting, rescuring and raising gimbal

void speed_calc(rc_info_t *, ctrl_info_t *, int16_t); // calculate the f/b/l/f speed
                                             // not const since the keyState is going to be updated
void angle_cala(const rc_info_t *, ctrl_info_t *,int16_t); // calculate the g_v_r, g_h_r and c_h_r
void state_transfer(const rc_info_t *, ctrl_info_t *, int16_t); //update the state
void angle_determ(bool_t, ctrl_info_t *, int16_t, int16_t, int16_t, int16_t); //determine the angles basing on the state

int16_t chassis_speed_key(keysignal_t *, keysignal_t *, bool_t *, int);
                                       // not const since the keyState is going to be updated
                                       //bool_t: the target signal, i.e. W/A/S/D
                                       //bool_t []: the current state of the target signal
int16_t chassis_angle_key(const KbCtrl_t *); 
void gimbal_angle_mouse(const rc_info_t *, gimbal_ctrl_t *);
// above is for key-mouse control and PC control
void chassis_speed_rc(const rc_info_t *, chassis_ctrl_t *);
void gimbal_angle_rc(const rc_info_t *, gimbal_ctrl_t *);
#endif
