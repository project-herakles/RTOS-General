#include "Control.h"
#include "bool.h"
#include "keyboard_def.h"
#include "SinCosList.h"

//#define CONTROL_DEBUG
//#define KEYSTATE_DEBUG

#ifdef CONTROL_DEBUG
    #include <stdio.h>
#endif

extern int8_t ERROR_REPORTER;


#define RCMOVE_INACTIVE         rc->ch3==0 && rc->ch4 == 0
#define RCROTATE_INACTIVE       rc->ch1==0 && rc->ch2 == 0

#define MODULE(x,y)             (x+y*(x>0?0:1-x/y))%y
#define ANGLE_LIMIT(a)          MODULE((a+180),360)-180

#define WHETHER_MOVE            ctrl->chassis_ctrl_ptr->forward_back_speed_ref!=0 &&ctrl->chassis_ctrl_ptr->left_right_speed_ref!=0 &&ctrl->chassis_ctrl_ptr->rotation_speed_ref!=0


/**degree to COS
 * @brief: calculate the cos value given the angle in degree in the look-up table
 * @param: a----given angle, from -180 to 180 in degree
 * @reval: the cos value
 */ 
double toCos(short signed int a)
{
	if(a<0)
	{
		if(a<-90)	return -1*cosList[180+a];
		else			return cosList[-1*a];
	}
	else
	{
		if(a>90)	return -1*cosList[180-a];
		else			return cosList[a];
	}
}

/**degree to SIN
 * @brief: calculate the sin value given the angle in degree in the look-up table
 * @param: a----given angle, from -180 to 180 in degree
 * @reval: the sin value
 */ 
double toSin(short signed int a)
{
	if(a<0)
	{
		if(a<-90)	return -1*sinList[180+a];
		else			return -1*sinList[-1*a];
	}
	else
	{
		if(a>90)	return sinList[180-a];
		else			return sinList[a];
	}
}


/**gimbal angle calculation basing on rc
 * @brief: calculate the gimbal horizontal and vertical angle with rc data
 * @param: rc----structure for remote controller signals
 *         gimbal_ref----structure for gimbal control
 * @reval: structure for gimble horizontal angle and vertical angle
 */ 
void gimbal_angle_rc(const rc_info_t * rc, gimbal_ctrl_t * gimbal_ref)
{
    //from the last-year code
    gimbal_ref->horizontal_angle_ref = (int16_t)((float)rc->ch2*ROTATION_CONST_PITCH);
    gimbal_ref->vertical_angle_ref   = (int16_t)((float)rc->ch1*ROTATION_CONST_YAW);
}

/**chassis speed calculation basing on rc
 * @brief: calculate the chassis speed (forward/backward/left/right) with rc data
 * @param: rc----structure for remote controller signals
 *         chassis_ref----structure for chassis control
 * @reval: 
 */ 
void chassis_speed_rc(const rc_info_t * rc, chassis_ctrl_t * chassis_ref)
{
    int16_t ch3_abs;
	int16_t ch4_abs;

	ch3_abs = rc->ch3<0 ? -(rc->ch3):rc->ch3;
	ch4_abs = rc->ch4<0 ? -(rc->ch4):rc->ch4;

    chassis_ref->forward_back_speed_ref = ch4_abs > 10? rc->ch4 * ch4_abs * SPEED_CONST   : 0;
    chassis_ref->left_right_speed_ref = ch3_abs > 10? rc->ch3 * ch3_abs * SPEED_CONST : 0;

}

/**gimbal angle calculation basing on mouse/PC
 * @brief: calculate the gimbal horizontal and vertical angle with mouse signal
 *         in rc data; called whenever the mouse signal is not 0
 * @param: rc----structure for remote controller signals
 *         gimbal----structure for gimbal control
 * @reval: structure for gimble horizontal angle and vertical angle
 */ 
void gimbal_angle_mouse(const rc_info_t * rc, gimbal_ctrl_t * gimbal)
{
    //from the last year code
    gimbal->vertical_angle_ref   = (int16_t)(rc->ch5 * ANGLE_CONST);
    gimbal->horizontal_angle_ref = (int16_t)(rc->ch6 * ANGLE_CONST);
}

/**chassis angle calculation basing on key/PC
 * @brief: calculate the chassis rotation angle with the signal of key Q and key E
 * @param: kb----structure for keyboard data, including the signal of keys and keystate
 * @reval: angle ref
 */ 
int16_t chassis_angle_key(const KbCtrl_t * kb)
{
    int16_t direction = kb->E * CW + kb->Q * CCW;
    return direction * ROTATION_SPEED; 
}

/**chassis speed calculation basing on key/PC
 * @brief: calculate the chassis speed with the signal of key W, A, S and D
 * @param: key----one key signal in the kbCtrl struct, one of the W, A, S and D
 *         state----an array of length two, storing the state of the given key
 *         speed----current speed
 * @reval: the speed in that direction
 */ 
int16_t chassis_speed_key(keysignal_t * key,keysignal_t * high, bool_t * state, int speed)
{
    speed = kb_keyControl(key,high, state, speed);
    key->p = key->n;
    high->p = high->n;
    return speed;
}

/**angle determination
 * @brief: determine the angles for gimbal horizontal rotation and chassis rotation 
 * @param: rc----the structure storing the control signals
 *         ctrl----the structure storing the state, speed and angle ref
 *         delta_gh----delta ref for horizontal rotation of gimbal
 *         delta_gv----delta ref for vertical rotation of gimbal
 *         delta_cr----delta ref for chassis rotation
 *         angle_indet: extern global variable for anlge between chassis direction and gimbal direction
 * @reval:
 */ 
void angle_determ(bool_t swing, ctrl_info_t * ctrl, int16_t delta_gh, int16_t delta_gv, int16_t delta_cr, int16_t angle_inbet)
{
    if(ctrl->state[2]==False && ctrl->state[3]==False)//C follow G mode
    {
        // in this mode, gimbal will not rotate relatively, angle of chassis rotation is either its orignal value or the gimbal ref
        ctrl->gimbal_ctrl_ptr->horizontal_angle_ref += (delta_cr + delta_gh);
        ctrl->chassis_ctrl_ptr->rotation_speed_ref  =  (int16_t)(-1*ANGLE_INBET*angle_inbet);
        //if(angle_inbet < ANGLE_ERROR && angle_inbet > -1*ANGLE_ERROR)           ctrl->gimbal_ctrl_ptr->horizontal_angle_ref -= angle_inbet;
    }
    else if(ctrl->state[2]==False && ctrl->state[3]==True)//chassis move, gimbal free
    {
        ctrl->chassis_ctrl_ptr->rotation_speed_ref  =  delta_cr;
        ctrl->gimbal_ctrl_ptr->horizontal_angle_ref += (int16_t)(delta_gh-ANGLE_INBET*angle_inbet);
    }
    else if(ctrl->state[2]==True && ctrl->state[3]==False)//chassis rotate and gimbal stick
    {
        if(swing)
        {
            ctrl->chassis_ctrl_ptr->rotation_speed_ref = SWING_SPEED;
        }
        else
        {
            ctrl->chassis_ctrl_ptr->rotation_speed_ref = (int16_t)(-1*BACK_CONST*angle_inbet);
        }
        ctrl->gimbal_ctrl_ptr->horizontal_angle_ref += delta_gh;
    }
    ctrl->gimbal_ctrl_ptr->vertical_angle_ref += delta_gv;
}

/**state transfer
 * @brief: update the machine state basing on the received messages
 * @param: rc----structure for remote controller signals
 *         ctrl----the structure storing the state, speed and angle ref
 *         angle_indet: extern global variable for anlge between chassis direction and gimbal direction
 * @reval:
 */ 
void state_transfer(const rc_info_t * rc, ctrl_info_t * ctrl, int16_t angle_inbet)
{
    OneBit one_bit;
    one_bit.S = rc->kb_ctrl.Swing == True?1:0;
    one_bit.M = WHETHER_MOVE?1:0;
    one_bit.B = angle_inbet > ANGLE_ERROR || angle_inbet < -1*ANGLE_ERROR ? 0:1;
    one_bit.S1 = ctrl->state[2];
    one_bit.S2 = ctrl->state[3];

    one_bit.S1 = (one_bit.S2 & ~one_bit.M) | (one_bit.S1 & ~one_bit.B) | (~one_bit.S1 & ~one_bit.S2 & one_bit.S);
    one_bit.S2 = (~one_bit.S1 & one_bit.M);

    #ifdef CONTROL_DEBUG
        printf("STATE: %d%d | S: %d M: %d B: %d -> STATE: %d%d",ctrl->state[2],ctrl->state[3],
                                                                                    one_bit.S,one_bit.M,one_bit.B, one_bit.S1,one_bit.S2);
    #endif

		ctrl->state[2] = one_bit.S1?True:False;
		ctrl->state[3] = one_bit.S2?True:False;
	#ifndef ENGINEER
    if(rc->sw2 == 3) // stop and initial
    {
        if(ctrl->state[0]!=False || ctrl->state[1]!=False)        chassisGimbalInit(ctrl);
        ctrl->state[0] = False;
        ctrl->state[1] = False;
    }
    else if(rc->sw2 == 1) // manual
    {
        ctrl->state[0] = False;
        ctrl->state[1] = True;
    }
    else if(rc->sw2 == 2) // auto aiming
    {
        ctrl->state[0] = True;
        ctrl->state[1] = False;          
    }
    /*else if(rc->sw1 == 2 && rc->sw2 != 3) // auto
    {
        ctrl->state[0] = 1;
        ctrl->state[1] = 1;
    }*/
    else                                                return; //not-in-used
	#else
		ctrl->state[0] = False;
		ctrl->state[1] = True;
		//as for the Engineer, it should always be in MANUAL mode
	#endif
}

/**angle calculation
 * @brief: calculate the angle ref for chassis and gimbal
 *         involving gimbal_angle_rc, gimbal_angle_mouse and chassis_angle_key
 *         NOTE: before calling each function, it detects what the state is and whether the 
 *               keyboard or mouse sends signals and then call the corresponding functions. 
 *         NOTE: the values here are not the final result; they are going to be modified according
 *               to the G-C state after the state is updated 
 * @param: rc----structure for remote controller signals
 *         ctrl----the structure storing the state, speed and angle ref
 *         delta_chassis_angle----delta ref for chassis rotation
 *         delta_gimbal_hAngle----delta ref for relative horizontal angle of gimbal rotation
 *         delta_gimbal_vAngle----delta ref for relative vertical angle of gimbal rotation
 * @reval:
 */ 

void angle_cala(const rc_info_t * rc, ctrl_info_t * ctrl, int16_t angle_inbet)
{
    if(ctrl->state[0]==False && ctrl->state[1]==False)                 //stop mode
    {
        state_transfer(rc,ctrl,angle_inbet);
        return;
    }
    int16_t delta_chassis_angle = chassis_angle_key(&rc->kb_ctrl);
    int16_t delta_gimbal_hAngle;
    int16_t delta_gimbal_vAngle;
    if(ctrl->state[0]==False && ctrl->state[1]==True)                 //manual mode
    {
        //independent for mouse-control-calculation
        gimbal_ctrl_t gimbal_mouse;
        gimbal_angle_mouse(rc,&gimbal_mouse);
        //independent for rc-control-calculation
        gimbal_ctrl_t gimbal_rc;
        gimbal_angle_rc(rc, &gimbal_rc);
        //mouse has a higher priority
        delta_gimbal_vAngle = gimbal_mouse.vertical_angle_ref==0  ? gimbal_rc.vertical_angle_ref  :gimbal_mouse.vertical_angle_ref;
        delta_gimbal_hAngle = gimbal_mouse.horizontal_angle_ref==0? gimbal_rc.horizontal_angle_ref:gimbal_mouse.horizontal_angle_ref;
    }
    else                                                      //auto-aiming or auto
    {
        //only mouse can control the gimbal
        gimbal_ctrl_t gimbal_mouse;
        gimbal_angle_mouse(rc,&gimbal_mouse);
        delta_gimbal_vAngle = gimbal_mouse.vertical_angle_ref;
        delta_gimbal_hAngle = gimbal_mouse.horizontal_angle_ref;
    }
    delta_gimbal_hAngle = ANGLE_LIMIT(delta_gimbal_hAngle);
    delta_gimbal_vAngle = delta_gimbal_vAngle>25 ? 25 : delta_gimbal_vAngle;
    delta_gimbal_vAngle = delta_gimbal_vAngle<-25?-25 : delta_gimbal_vAngle;

    state_transfer(rc,ctrl,angle_inbet);
    angle_determ(rc->kb_ctrl.Swing, ctrl, delta_gimbal_hAngle, delta_gimbal_vAngle, delta_chassis_angle,angle_inbet);
    ctrl->gimbal_ctrl_ptr->horizontal_angle_ref=ANGLE_LIMIT(ctrl->gimbal_ctrl_ptr->horizontal_angle_ref);
}

/**speed calculation
 * @brief: calculate the chassis speed
 *         involving chassis_speed_rc (for 1 time) or chassis_speed_key (for 4 times)
 *         NOTE: before calling each function, it detects what the state is and whether the 
 *               keyboard or mouse sends signals and then call the corresponding functions. 
 * @param: rc----structure for remote controller signals
 *         ctrl----the structure storing the state, speed and angle ref
 *         fSpeed----the speed ref in forward direction
 *         bSpeed----the speed ref in backward direction
 *         lSpeed----the speed ref to shift left
 *         rSpeed----the speed ref to shift right
 * @reval:
 */ 
void speed_calc(rc_info_t * rc, ctrl_info_t * ctrl, int16_t angle_inbet)
{
    if(ctrl->state[0]==False && ctrl->state[1]==False)
    {
        *(ctrl->chassis_ctrl_ptr) = (chassis_ctrl_t)chassisInit;
        return;
    }// stop & initial mode
    if(ctrl->state[0]==True || (RCMOVE_INACTIVE))                // auto-aiming or auto or remote-controller gives no sigals
    {
        int16_t fSpeed = ctrl->chassis_ctrl_ptr->forward_back_speed_ref>0?ctrl->chassis_ctrl_ptr->forward_back_speed_ref:0;
        fSpeed = chassis_speed_key(&rc->kb_ctrl.W, &rc->kb_ctrl.High, rc->kb_ctrl.keyState.w_state, fSpeed);

        int16_t bSpeed = ctrl->chassis_ctrl_ptr->forward_back_speed_ref<0?-1*ctrl->chassis_ctrl_ptr->forward_back_speed_ref:0;
        bSpeed = chassis_speed_key(&rc->kb_ctrl.S, &rc->kb_ctrl.High, rc->kb_ctrl.keyState.s_state, bSpeed);
        
        int16_t rSpeed = ctrl->chassis_ctrl_ptr->left_right_speed_ref>0?ctrl->chassis_ctrl_ptr->left_right_speed_ref:0;
        rSpeed = chassis_speed_key(&rc->kb_ctrl.D, &rc->kb_ctrl.High, rc->kb_ctrl.keyState.d_state, rSpeed);
        
        int16_t lSpeed = ctrl->chassis_ctrl_ptr->left_right_speed_ref<0?-1*ctrl->chassis_ctrl_ptr->left_right_speed_ref:0;
        lSpeed = chassis_speed_key(&rc->kb_ctrl.A, &rc->kb_ctrl.High, rc->kb_ctrl.keyState.a_state, lSpeed);

        ctrl->chassis_ctrl_ptr->forward_back_speed_ref = fSpeed-bSpeed;
        ctrl->chassis_ctrl_ptr->left_right_speed_ref = rSpeed-lSpeed;
        #ifdef KEYSTATE_DEBUG
            kb_printState('W', rc->kb_ctrl.keyState.w_state);
            kb_printState('S', rc->kb_ctrl.keyState.s_state);
            kb_printState('A', rc->kb_ctrl.keyState.a_state);
            kb_printState('D', rc->kb_ctrl.keyState.d_state);
        #endif
    }
    else
    {
        chassis_speed_rc(rc,ctrl->chassis_ctrl_ptr); 
    }

    //derive the actual speed reference from the control ref and angle in between
    int16_t vf_ref = ctrl->chassis_ctrl_ptr->forward_back_speed_ref;
    ctrl->chassis_ctrl_ptr->forward_back_speed_ref = (int16_t)(vf_ref * toCos(angle_inbet)
                       + ctrl->chassis_ctrl_ptr->left_right_speed_ref * toSin(angle_inbet));
    ctrl->chassis_ctrl_ptr->left_right_speed_ref   = (int16_t)(vf_ref * toSin(angle_inbet)
                       - ctrl->chassis_ctrl_ptr->left_right_speed_ref * toCos(angle_inbet));
}

/**referance calculation
 * @brief: this function will calculate the ref for both chassis and gimbal
 *         involving speed_calc, angle_calc, state_transfer and angle_determ
 * @param: rc----structure for remote controller signals 
 *         ctrl----the structure storing the state, speed and angle ref
 * @reval:
 */
void refCalc(rc_info_t * rc, ctrl_info_t * ctrl, int16_t angle_inbet)
{
    speed_calc(rc, ctrl, angle_inbet);
    angle_cala(rc, ctrl, angle_inbet);
    //state_transfer(rc, ctrl);angle_determ(ctrl); inside angle_cala()
}

/**remote control data dealler
 * @brief: store the collected data (communication buffer and motor feedback) in rc
 *         NOTE: this is merely for testing
 * @param: buff----an array of lenght 14 for remote control data
 *         feed----an array of length 02 for feedback
 *         rc----structure for remote controller signals 
 *         angle_indet: extern global variable for anlge between chassis direction and gimbal direction
 * @reval:
 */
void rcDealler(ctrl_info_t * ctrl, const int16_t * feed, rc_info_t * rc, int16_t * angle_inbet)
{
  /*
	remote control signals are handled by interrupt directly
	this function is going to handle the control signal for special func
	*/
	#ifndef ENGINEER
	if(rc->sw1.n==1&&rc->sw1.p!=1)//friction wheel in standard and hero, fetch bullets in Engineer
	{
		ctrl->func_ptr->fWheel = ctrl->func_ptr->fWheel?0:1; //flip if sw1 is at position 1
	}
	if(rc->sw1.n==2&&rc->sw1.p!=2)//magazine lid in standard and hero, fetch damaged robots in Engineer
	{
		ctrl->func_ptr->magazineLid = ctrl->func_ptr->magazineLid?0:1; //flip if sw1 is at position 1
	}
	//ctrl->func_ptr->raising
	if(rc->kb_ctrl.othKey & Z && feed[1]!=RAISING_HEAD)
	{
		ctrl->func_ptr->raising = 2;//10----raising
	}
	else if(!(rc->kb_ctrl.othKey & Z) && feed[1]!=RAISING_BOTTOM)
	{
		ctrl->func_ptr->raising = 1;//01----declining
	}
	else
	{
		ctrl->func_ptr->raising = 0;//00----holding position
	}
	//ctrl->func_ptr->shoot
	//ctrl->func_ptr->shootMode

  *angle_inbet = feed[0];
	#else
	if(rc->sw2==1) //robot rescuring mode
	{
		if(rc->sw1.n==1 && rc->sw1.p!=1)			ctrl->func_ptr->grabRobot = 1;
		if(rc->sw1.n==2 && rc->sw1.p!=2)			ctrl->func_ptr->releaseRobot = 1;
	}
	if(rc->sw2==3) //bullet fetching mode
	{
		if(rc->sw1.n==1 && rc->sw1.p!=1)			ctrl->func_ptr->grabBullet = 1;
		if(rc->sw1.n==2 && rc->sw1.p!=2)			ctrl->func_ptr->releaseBullet = 1;
	}
	#endif
}

/**chassis and gimbal initialization
 * @brief: initialize the ctrl data when the machine is (re)started or switch back to the 
 *         stop/init state
 * @param: ctrl----the structure storing the state, speed and angle ref
 * @reval:
 */
void chassisGimbalInit(ctrl_info_t * ctrl)
{
    *(ctrl->chassis_ctrl_ptr) = (chassis_ctrl_t)chassisInit;
    *(ctrl->gimbal_ctrl_ptr)  = (gimbal_ctrl_t)gimbalInit;
    *(ctrl->func_ptr)         = (func_t)FuncInit;
    ctrl->state[0] = False;
    ctrl->state[1] = False;
    ctrl->state[2] = False;
    ctrl->state[3] = False;
}
