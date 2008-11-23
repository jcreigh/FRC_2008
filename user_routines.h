/*******************************************************************************
* FILE NAME: user_routines.h
*
* DESCRIPTION: 
*  This is the include file which corresponds to user_routines.c and
*  user_routines_fast.c
*  It contains some aliases and function prototypes used in those files.
*
* USAGE:
*  If you add your own routines to those files, this is a good place to add
*  your custom macros (aliases), type definitions, and function prototypes.
*******************************************************************************/

#ifndef __user_program_h_
#define __user_program_h_


/*******************************************************************************
                            MACRO DECLARATIONS
*******************************************************************************/
/* Add your macros (aliases and constants) here.                              */
/* Do not edit the ones in ifi_aliases.h                                      */
/* Macros are substituted in at compile time and make your code more readable */
/* as well as making it easy to change a constant value in one place, rather  */
/* than at every place it is used in your code.                               */
/*
 EXAMPLE CONSTANTS:
#define MAXIMUM_LOOPS   5
#define THE_ANSWER      42
#define TRUE            1
#define FALSE           0
#define PI_VAL          3.1415

 EXAMPLE ALIASES:
#define LIMIT_SWITCH_1  rc_dig_int1  (Points to another macro in ifi_aliases.h)
#define MAIN_SOLENOID   solenoid1    (Points to another macro in ifi_aliases.h)
*/

/* Used in limit switch routines in user_routines.c */
#define OPEN        1     /* Limit switch is open (input is floating high). */
#define CLOSED      0     /* Limit switch is closed (input connected to ground). */
#define MAX_INTEGRAL_ERROR 	100

#define Light1	Pwm1_red
#define Light2	Pwm1_green
#define Light3  Pwm2_red
#define Light4  Pwm2_green
#define Light5  Relay1_red
#define Light6  Relay1_green
#define Light7  Relay2_red
#define Light8  Relay2_green


//Trolley Positions - Encoder Click Count Relative to zero - (Zero Position is Max Stop)
#define Trolley_Top 		0 // (zero position); MK-0
#define Trolley_Capture		1441	// ok; MK-1956
#define Trolley_Capture_Move_Arm 75	// ok
#define Trolley_Place_Hurdle	75	// ok; MK-0
#define Trolley_Place_Hurdle_Move_Arm 75	// ok
#define Trolley_Place_Hurdle_Lower 100 	// ok (same as Trolley_Poke_Ready)
#define Trolley_Poke_Ready	1441  //450	// ok; MK-1426 - Version 1
#define Trolley_Poke_Up		288 //75	// ok; MK-163 - Version 1
#define Trolley_Poke		288 //300	// Version 2
#define Trolley_Poke_Move_Arm	100	// Version 2
#define Trolley_Path1_Tuck	75	// ok
#define Trolley_Path2_Tuck	100	// ok; MK-204
#define Trolley_Path1_Home	75	// ok
#define Trolley_Path2_Home	50	// ok; MK-1956
#define Trolley_Path1_Ball_Release 75	// ok
#define Trolley_Path2_Ball_Release 75	// ok
#define Trolley_Bottom 		1370 // (Max encoder click - end of range)

#define Trolley_Max_Waitcount	15 // Number of user code loops to wait

//Arm Positions - Encoder Click Count Relative to zero - (Zero Position is Max down)
#define Arm_Top		 	0 // (Zero position); MK-0
#define Arm_Out			208	// ok (same as Arm_Capture)
#define Arm_Capture		208	// ok; MK-313
#define Arm_Capture_Move_Gripper 50	// ok
#define Arm_Place_Hurdle	63	// ok; MK-157
#define Arm_Place_Hurdle_Retract 0	// ok (same as Arm_Poke_Ready); MK-0
#define Arm_Poke_Ready		(-7)	// ok; MK-0	Version 1
#define Arm_Poke_Up		(-7)	// okl MK-157	Version 1 (63)
#define Arm_Poke		(-7)	// Version 2
#define Arm_Tuck		300	// ok; MK-504
#define Arm_Ball_Release	63	// ok; MK-157
#define Arm_Home		420	// ok; MK-87
#define Arm_Bottom		420 // (Max encoder click - end of range); MK-556

//MK-87 - Hurdle Detect ????

#define Bot_Straight_Long	3900	// 39.5 feet (9 clicks per inch)
#define Bot_Turn_90		500	// 4 clicks per degree
#define Bot_Straight_Short	1080	// 10 feet (9 clicks per inch)


#define Arm_Target_Range_Threshold 10	// This is the number of encoder clicks above and below the target position that will be considered to be 'at target

#define Trolley_Target_Range_Threshold 10 // This is the number of encoder clicks above and below the target position that will be considered to be 'at target

#define Bot_Target_Range_Threshold 5 // This is the number of encoder clicks above and below the target position that will be considered to be 'at target

//Bot Move in Poke Mode
#define Bot_Poke_Move_To_Ready_Position 0 // # of clicks from Place_Hurdle to Poke_Ready

// Set Power/Speed for Arm/Trolley for different conditions
//Arm Power
#define Arm_Detect_pwm_Out	90  //when it is detecting position (only in detect mode) - relative pwm value to add/subtract from idle
#define Arm_Top_Out_pwm_Out	75  //when it is being moved to Top/Out position - relative pwm value to add/subtract from idle

#define Arm_Nudge_Click_Change	50  // MK-244; plus/minus the current arm encoder value

//Trolley Power
#define Trolley_Detect_pwm_Out	0  //when it is detecting position - relative pwm value to add/subtract from idle
#define Trolley_Top_Bottom_pwm_Out	75  //when it is being moved to Top/Bottom position - relative pwm value to add/subtract from idle

//Bot Drive Power (Override for Place_Hurdle and Poke)
#define Bot_Creep_State_pwm_out			170 	//Creep up to the Overpass bar
#define Bot_Poke_Ready_State_pwm_out		150
#define Bot_Driver_Scale_Factor 		3 //Allow driver to manipulate bot up to specific scale factor of driver input

//Automatic shifting
// Maximum low gear speed = 5 feet per second (MK)
// Encloder clicks per inch = 9 CPI (MK)
// Slow code loops per second = 38.16794 (B)
//
// Clicks per second at maximum low gear = 540 = 5FPS * 60Inches * EncoderClicksPerInch (A)
//
// The number of encoder clicks per slow code loop at MAximum Low gear speed = (A) / (B) = 14.148
//
// Usable factor (including the fudge factor) = 13
//
#define Max_Encoder_CPI_Low_Gear 		13 //Shift to high gear when this value is met or exceeded
							
/*

#define RC_ana_1			Get_Analog_Value(rc_ana_in01)
#define Button1	p1_sw_top==1
#define Button2	p1_sw_aux1==1
#define Button3	p1_sw_aux2==1
#define Button4	p1_sw_trig==1
#define Button5	p2_sw_top==1
#define Button6	p2_sw_aux1==1
#define Button7	p2_sw_aux2==1
#define Button8	p2_sw_trig==1
*/

#define LED_Driver_Hybrid_CMD2		Pwm1_red
#define LED_Driver_Tilt			Pwm1_red
#define LED_Driver_Hybrid_CMD1		Pwm1_green
#define LED_Driver_Capture		Pwm1_green
#define LED_Driver_Shift_Override	Pwm2_green
#define LED_Driver_Shift_Funct_Override Pwm2_red

#define LED_Manip_Hybrid_CMD2		Relay_1_red
#define LED_Manip_Tilt			Relay_1_red
#define LED_Manip_Hybrid_CMD1		Relay_1_green
#define LED_Manip_Capture		Relay_1_green
#define LED_Manip_Shift_Override	Relay_2_green
#define LED_Manip_Funct_Override	Relay_2_red

/*
#define OI_Drive_x p1_x
#define OI_Drive_y p1_y
*/


#define joy_deadzone				7
#define joy_max					127

#define Encoder_Left_Count				(0-Get_Encoder_1_Count())
//#define Encoder_2_Count				(signed long)(4294967295-(unsigned long)Get_Encoder_2_Count())+1
#define Encoder_Right_Count				(Get_Encoder_2_Count())
#define f_Encoder_Trolley_Count				(0-Get_Encoder_3_Count())
#define f_Encoder_Arm_Count				(Get_Encoder_4_Count())

#define Bar_Detect_Count_Max		5 	// Number of loops the bar detect switch must be pressed before being considered a true
						// Experience may tell us a better loopcount


// OI Input
#define OI_Button_Place_Hurdle 		p1_sw_top  	//ok
#define OI_Button_Poke			!p1_sw_trig 	//ok
#define OI_Button_Tuck			!p1_sw_aux1 	//ok
#define OI_Button_Capture		!p2_sw_top	//ok
#define OI_Button_Ball_Release          p3_sw_trig	//ok

#define OI_Button_Home			!p2_sw_aux1	//ok
#define OI_Button_Function_Override	p1_sw_aux2	//ok
#define OI_Button_Arm_Top		p3_sw_top	//ok
#define OI_Button_Arm_Out		p3_sw_aux1	//ok
#define OI_Button_Grip_OpenClose	p3_sw_aux2	//ok
#define OI_Button_Trolley_Top		p4_sw_trig	//ok
#define OI_Button_Trolley_Down		p4_sw_top	//ok
#define OI_Button_Nudge_Arm_Up		p4_sw_aux1	//ok
#define OI_Button_Nudge_Arm_Down	p4_sw_aux2	//ok

//Port 2 Driver
#define OI_Button_Trans			p2_sw_trig	//ok
#define OI_Button_Shift_Override	!p2_sw_aux2	//ok
#define OI_Drive_x			p2_x		//ok
#define OI_Drive_y			p2_y		//ok

//PWM Motors
#define RC_Motor_Drive_Left		pwm01
#define RC_Motor_Drive_Right		pwm02
#define RC_Motor_Trolley		pwm03
#define RC_Motor_Arm			pwm04

//Analog In
#define RC_Accel_x			Get_Analog_Value(rc_ana_in01)
#define RC_Accel_y			Get_Analog_Value(rc_ana_in02)
#define RC_Utrasonic			Get_Analog_Value(rc_ana_in03)
#define RC_Gyro_Twist			Get_Analog_Value(rc_ana_in04)
#define RC_Gyro_Temp			Get_Analog_Value(rc_ana_in05)
#define RC_IRBoard_CMD1			Get_Analog_Value(rc_ana_in06)!=0
#define RC_IRBoard_CMD2			Get_Analog_Value(rc_ana_in07)!=0
#define RC_IRBoard_CMD3			Get_Analog_Value(rc_ana_in08)!=0
#define RC_IRBoard_CMD4			Get_Analog_Value(rc_ana_in09)!=0
#define RC_Hybrid_Move			Get_Analog_Value(rc_ana_in10)!=0
#define RC_Hybrid_Poke_1		Get_Analog_Value(rc_ana_in11)!=0
#define RC_Hybrid_Poke_2		Get_Analog_Value(rc_ana_in12)!=0
#define RC_Hybrid_Dflt_Poke_Position	Get_Analog_Value(rc_ana_in13)!=0
#define RC_Hybrid_Starting_Position_1	Get_Analog_Value(rc_ana_in14)!=0
#define RC_Hybrid_Starting_Position_2	Get_Analog_Value(rc_ana_in15)!=0

#define RC_Hybrid_On			Get_Analog_Value(rc_ana_in15)!=0
#define RC_Hybrid_sw1			Get_Analog_Value(rc_ana_in13)!=0
#define RC_Hybrid_sw2			Get_Analog_Value(rc_ana_in14)!=0


//Digital In (non-encoder)
#define RC_Bar_Detect			!rc_dig_in09
#define RC_Grip_Open			!rc_dig_in10
#define RC_Grip_Close			!rc_dig_in11
#define RC_Grip_Left			!rc_dig_in12
#define RC_Grip_Right			!rc_dig_in13
#define RC_Arm_Max_Up			rc_dig_in14
#define RC_Arm_Home			rc_dig_in15


//Relay
#define RC_Pneum_Grip_Left_Open		relay1_fwd
#define RC_Pneum_Grip_Left_Close	relay1_rev
//#define RC_Pneum_Grip_Right_Open	relay2_fwd
//#define RC_Pneum_Grip_Right_Close	relay2_rev
#define RC_Pneum_Trans_Low		relay3_fwd
#define RC_Pneum_Trans_High		relay3_rev
#define RC_Pneum_Compressor_Start	relay4_fwd
//#define RC_Pneum_Compressor		relay4_rev


//State_Arm_Position
#define Arm_Max_Bottom_State 			1
#define Arm_Capture_State 			2
#define Arm_Tuck_State 				3
#define Arm_Place_Hurdle_State 			4
#define Arm_Poke_State 				5
#define Arm_Home_State				6
#define Arm_Max_Top_State			7
#define Arm_Ball_Release_State			8
#define Arm_Searching_State			9

//State_Nudge
#define Arm_Nudge_No_State			10
#define Arm_Nudge_Yes_State			11

//Bot State
#define Bot_Creep_State				12
#define Bot_Poke_Ready_State			13
#define Bot_Poke_Move_To_Ready_State		14
#define Bot_Poke_Moving_To_Ready_State		15


//State_Trolley_Position
#define Trolley_Max_Bottom_State 		16
#define Trolley_Capture_State 			17
#define Trolley_Tuck_State 			18
#define Trolley_Place_Hurdle_State		19
#define Trolley_Poke_State			20
#define Trolley_Home_State			21
#define Trolley_Max_Top_State			22
#define Trolley_Ball_Release_State		23
#define Trolley_Searching_State			24
#define Trolley_Move_Arm_Position_State		25

//State_Mode
#define Mode_Capture_State			26 // No ball; Positioning to capture (transient state)
#define Mode_Place_Hurdle_State			27 // Ball; positioning to be hurdled or placed (transient state)
#define Mode_Poke_State				28 // No ball; the bot is positioning to poke (transient)
#define Mode_Poke_Done_State			29 // No ball; the bot is positioning to poke (an end statr)
#define Mode_Ball_Release_State			30 // Ball; Moving to position to release the ball (transient)
#define Mode_Home_State				31 // No ball; arm/troller/gripper are going home (transient)
#define Mode_Home_Done_State			32 // No ball; Arm/trolley/gripper are home (an end state)
#define Mode_Tuck_State				33 // Ball;  process of being tucked (transient)
#define Mode_Tuck_Done_State			34 // Ball has been tucked (an end state)
#define Mode_Arm_Trolley_Joystick_Control	35 // Used at startup only to maneuver arm and trolley to a position

//State_Poke
#define Poke_Place_Hurdle_Position		36	// ok
#define Poke_Overpass_Detected			37	
#define Poke_Retracting_Arm			38
#define Poke_Lowering_Trolley			39
#define Poke_Arm_Retracted			40	// ok
#define Poke_Up					41	// ok
#define Poke_Ready				42
#define Poke_Arm_Positioning			43	// ok
#define Poke_Trolley_Position			44	// ok
#define Poke_Up_Arm_Done			45	// ok
#define Poke_Up_Trolley_Done			46	// ok
#define Poke_Arm_Done				47	// ok
#define Poke_Trolley_Done			48

//State_Hurdle
#define Place_Hurdle_Position			49
#define Place_Hurdle_Ready			50
#define Place_Hurdle_Arm_Retracted		51
#define Place_Hurdle_Arm_Done			52
#define Place_Hurdle_Trolley_Lowered		53
#define Place_Hurdle_Trolley_Done		54
#define Place_Hurdle_Done			55

//State_Home
#define Home_Position				56
#define Home_Ready				57

//State_Tuck
#define Tuck_Position				58	// ok
#define Tuck_Ready				59	// ok


//State_Ball_Release
#define Ball_Release_Position			60
#define Ball_Release_Ready			61

//State_Capture
#define Capture_Ball_Position			62
#define Capture_Ready				63
#define Capture_Done				64

//State_Grip
#define Grip_Open_State				65
#define Grip_Close_State			66
#define Grip_Ball_State				67
#define Gripper_Opening_State			68
#define Gripper_Closing_State			69
#define Gripper_Manual_Control_State		70

//State_Overpass
#define Overpass_Not_Detected			71
#define Overpass_Detected			72

//State_Back_Tilt
#define Back_Tilt_Not_Detected			73
#define Back_Tilt_Detected			74

//State_Side_Tilt
#define Side_Tilt_Not_Detected			75
#define Side_Tilt_Detected			76

//State_Trans
#define Mode_Trans_Low_State			77
#define Mode_Trans_High_State			78

//State_Error
#define No_Error				79
#define Error					80

//State_Move
#define Mode_Move_OI_State			81
#define Mode_Move_Auto_State			82

//Other States
#define Unknown_State 				95	// ok
#define Action_Lowering_Trolley			96	// ok
#define Action_Retracting_Arm			97	// ok
#define Wait_State				98	// ok
#define Move_To_Position_State			100	// ok


/*******************************************************************************
                            TYPEDEF DECLARATIONS
*******************************************************************************/

typedef struct{
	int *value; 		// Current value
	signed int prev_err;
	signed int integral_err;
	unsigned int hold_value;  // Target value
	unsigned float KP;
	unsigned float KI;
	unsigned float KD;
	unsigned char *motor;	//Actual PWM Output
} motor_structure;

typedef struct{
	unsigned float KP;
	unsigned float KI;
	unsigned float KD;	
} pid_structure;
/*******************************************************************************
                           FUNCTION PROTOTYPES
*******************************************************************************/

/* These routines reside in user_routines.c */
void User_Initialization(void);
void Process_Data_From_Master_uP(void);
void Default_Routine(void);

/* These routines reside in user_routines_fast.c */
void InterruptHandlerLow (void);  /* DO NOT CHANGE! */
void User_Autonomous_Code(void);  /* Only in full-size FRC system. */
void Process_Data_From_Local_IO(void);
unsigned char joy_condition(unsigned char in);
void debug(unsigned int v);
void debug_long(unsigned long v);
signed int PID(motor_structure pid);
long abs(long x);
int absint(int x);
void Handle_Sensors(void);
void Handle_OI(void);
void Handle_States(void);
void Move_Bot(void);
void Move_Arm(void);
void Move_Trolley(void);
void Move_Gripper(void);
void LabView(void);
void LabView_Out(void);
void Capture_Ball_Request(void);
void Home_Request(void);
void Place_Hurdle_Request(void);
void Poke_Ball_Request(void);
void Tuck_Request(void);
void Ball_Release_Request(void);
void PID_Flop(motor_structure *mot, pid_structure pid);
void Auto_Out(unsigned char left, unsigned char right);

#endif
/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
