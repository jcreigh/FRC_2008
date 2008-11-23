/*******************************************************************************
* FILE NAME: user_routines.c <FRC VERSION>
*
* DESCRIPTION:
*  This file contains the default mappings of inputs  
*  (like switches, joysticks, and buttons) to outputs on the RC.  
*
* USAGE:
*  You can either modify this file to fit your needs, or remove it from your 
*  project and replace it with a modified copy. 
*
*******************************************************************************/

#include <stdio.h>

#include "ifi_aliases.h"
#include "ifi_default.h"
#include "ifi_utilities.h"
#include "user_routines.h"
#include "serial_ports.h"
#include "gyro.h"
#include "adc.h"
#include "encoder.h"


extern unsigned char aBreakerWasTripped;
//unsigned char State_Switch1;
//unsigned char State_Switch2;
//unsigned char State_Switch3;
//unsigned char State_Switch4;
//unsigned char State_Switch5;
//unsigned char State_Switch6;
//unsigned char State_Switch7;
//unsigned char State_Switch8;
int Encoder_Left_Speed=0;
int Encoder_Right_Speed=0;
unsigned char pollingtime=1;

int Encoder_Arm_Count=0;
int Encoder_Trolley_Count=0;

unsigned char Arm_Position_State=Unknown_State;
unsigned char Arm_Nudge_State=Arm_Nudge_No_State;
unsigned char Trolley_Position_State=Unknown_State;
unsigned char Bot_Position_State=Unknown_State;
unsigned char Gripper_Position_State=Unknown_State;
unsigned char Mode_State=Unknown_State;
unsigned char Gripper_State=Unknown_State;
unsigned char State_Overpass_Detected=Unknown_State;
unsigned char Back_Tilt_State;
unsigned char Side_Tilt_State;
unsigned char Arm_Override_State;
unsigned char State_Error=Unknown_State;
unsigned char State_Move=Unknown_State;
signed int Arm_Start_Position_Adj=0; //If Arm starts at TOP or searches then 0; If Arm starts at Home then need to proide adjustment so targets will be correct on an absolute position basis
unsigned char Arm_Trolley_Joystick_Control=0;

unsigned char State_Capture=Unknown_State;
unsigned char State_Poke=Unknown_State;
unsigned char State_Home=Unknown_State;
unsigned char State_Ball_Release=Unknown_State;
unsigned char State_Tuck=Unknown_State;
unsigned char State_Place_Hurdle=Unknown_State;

unsigned char OI_Request=Unknown_State;

signed long turncount=0;

signed int Tmp_Left_Speed=0;
signed int Tmp_Right_Speed=0;

unsigned char status; // Unknown use

int global1;
int global2;

motor_structure right;
motor_structure left;
motor_structure arm;
motor_structure trolley;
pid_structure trolleyup;
pid_structure trolleydown;
pid_structure armup;
pid_structure armdown;

/*** DEFINE USER VARIABLES AND INITIALIZE THEM HERE ***/
/* EXAMPLES: (see MPLAB C18 User's Guide, p.9 for all types)
unsigned char wheel_revolutions = 0; (can vary from 0 to 255)
unsigned int  delay_count = 7;       (can vary from 0 to 65,535)
int           angle_deviation = 142; (can vary from -32,768 to 32,767)
unsigned long very_big_counter = 0;  (can vary from 0 to 4,294,967,295)
*/

/*******************************************************************************
* FUNCTION NAME: Limit_Switch_Max
* PURPOSE:       Sets a PWM value to neutral (127) if it exceeds 127 and the
*                limit switch is on.
* CALLED FROM:   this file
* ARGUMENTS:     
*     Argument       Type             IO   Description
*     --------       -------------    --   -----------
*     switch_state   unsigned char    I    limit switch state
*     *input_value   pointer           O   points to PWM byte value to be limited
* RETURNS:       void
*******************************************************************************/
void Limit_Switch_Max(unsigned char switch_state, unsigned char *input_value)
{
  if (switch_state == CLOSED)
  { 
    if(*input_value > 127)
      *input_value = 127;
  }
}


/*******************************************************************************
* FUNCTION NAME: Limit_Switch_Min
* PURPOSE:       Sets a PWM value to neutral (127) if it's less than 127 and the
*                limit switch is on.
* CALLED FROM:   this file
* ARGUMENTS:     
*     Argument       Type             IO   Description
*     --------       -------------    --   -----------
*     switch_state   unsigned char    I    limit switch state
*     *input_value   pointer           O   points to PWM byte value to be limited
* RETURNS:       void
*******************************************************************************/
void Limit_Switch_Min(unsigned char switch_state, unsigned char *input_value)
{
  if (switch_state == CLOSED)
  { 
    if(*input_value < 127)
      *input_value = 127;
  }
}


/*******************************************************************************
* FUNCTION NAME: Limit_Mix
* PURPOSE:       Limits the mixed value for one joystick drive.
* CALLED FROM:   Default_Routine, this file
* ARGUMENTS:     
*     Argument             Type    IO   Description
*     --------             ----    --   -----------
*     intermediate_value    int    I    
* RETURNS:       unsigned char
*******************************************************************************/
unsigned char Limit_Mix (int intermediate_value)
{
  static int limited_value;
  
  if (intermediate_value < 2000)
  {
    limited_value = 2000;
  }
  else if (intermediate_value > 2254)
  {
    limited_value = 2254;
  }
  else
  {
    limited_value = intermediate_value;
  }
  return (unsigned char) (limited_value - 2000);
}


/*******************************************************************************
* FUNCTION NAME: User_Initialization
* PURPOSE:       This routine is called first (and only once) in the Main function.  
*                You may modify and add to this function.
* CALLED FROM:   main.c
* ARGUMENTS:     none
* RETURNS:       void
*******************************************************************************/
void User_Initialization (void)
{
  Set_Number_of_Analog_Channels(SIXTEEN_ANALOG);    /* DO NOT CHANGE! */

/* FIRST: Set up the I/O pins you want to use as digital INPUTS. */
  digital_io_01 = digital_io_02 = digital_io_03 = digital_io_04 = INPUT;
  digital_io_05 = digital_io_06 = digital_io_07 = digital_io_08 = INPUT;
  digital_io_09 = digital_io_10 = digital_io_11 = digital_io_12 = INPUT;
  digital_io_13 = digital_io_14 = digital_io_15 = digital_io_16 = INPUT;
    /* 
     Note: digital_io_01 = digital_io_02 = ... digital_io_04 = INPUT; 
           is the same as the following:

           digital_io_01 = INPUT;
           digital_io_02 = INPUT;
           ...
           digital_io_04 = INPUT;
    */

/* SECOND: Set up the I/O pins you want to use as digital OUTPUTS. */
  digital_io_17 = digital_io_18 = OUTPUT;    /* Example - Not used in Default Code. */

/* THIRD: Initialize the values on the digital outputs. */
// Use by LabView
  rc_dig_out17 = 1;
  rc_dig_out18 = 1;

/* FOURTH: Set your initial PWM values.  Neutral is 127. */
  pwm01 = pwm02 = pwm03 = pwm04 = pwm05 = pwm06 = pwm07 = pwm08 = 127;
  pwm09 = pwm10 = pwm11 = pwm12 = pwm13 = pwm14 = pwm15 = pwm16 = 127;

/* FIFTH: Set your PWM output types for PWM OUTPUTS 13-16.                    */
  /*   Choose from these parameters for PWM 13-16 respectively:               */
  /*     IFI_PWM  - Standard IFI PWM output generated with Generate_Pwms(...) */
  /*     USER_CCP - User can use PWM pin as digital I/O or CCP pin.           */
  Setup_PWM_Output_Type(IFI_PWM,IFI_PWM,IFI_PWM,IFI_PWM);

  /* 
     Example: The following would generate a 40KHz PWM with a 50% duty cycle on the CCP2 pin:

         CCP2CON = 0x3C;
         PR2 = 0xF9;
         CCPR2L = 0x7F;
         T2CON = 0;
         T2CONbits.TMR2ON = 1;

         Setup_PWM_Output_Type(USER_CCP,IFI_PWM,IFI_PWM,IFI_PWM);
  */

  /* Add any other initialization code here. */
  Init_Serial_Port_One();
  Init_Serial_Port_Two();

  stdout_serial_port = SERIAL_PORT_ONE;

  Initialize_Gyro();
  Initialize_ADC();
  
  Initialize_Encoders();

  /* Motor initialization */
  left.value=&Encoder_Left_Speed;
  left.motor=&RC_Motor_Drive_Left;
  right.value=&Encoder_Right_Speed;
  right.motor=&RC_Motor_Drive_Right;

  armup.KP=0;
  armup.KI=0;
  armup.KD=0;
  armdown.KP=0;
  armdown.KI=0;
  armdown.KD=0;

  /* Arm Motor initialization */
  arm.motor=&RC_Motor_Arm;
  arm.value=&Encoder_Arm_Count;
  PID_Flop(&arm, armup);
/*  arm.KP=0;//616.0/1000.0;
  arm.KI=0;//1770.0/1000.0;
  arm.KD=0;//744.0/1000.0; */


  trolleyup.KP=0;
  trolleyup.KI=0;
  trolleyup.KD=0;
  trolleydown.KP=0;
  trolleydown.KI=0;
  trolleydown.KD=0;

  /* Trolley Motor initialization */
  trolley.motor=&RC_Motor_Trolley;
  trolley.value=&Encoder_Trolley_Count;
  PID_Flop(&trolley,trolleyup);
  /*trolley.KP=0;//616.0/1000.0;
  trolley.KI=0;//1770.0/1000.0;
  trolley.KD=0;//744.0/1000.0;*/






  Putdata(&txdata);             /* DO NOT CHANGE! */


//  printf("IFI 2006 User Processor Initialized ...\r");  /* Optional - Print initialization message. */

  User_Proc_Is_Ready();         /* DO NOT CHANGE! - last line of User_Initialization */
}

/*******************************************************************************
* FUNCTION NAME: Process_Data_From_Master_uP
* PURPOSE:       Executes every 26.2ms when it gets new data from the master 
*                microprocessor.
* CALLED FROM:   main.c
* ARGUMENTS:     none
* RETURNS:       void
*******************************************************************************/
void Process_Data_From_Master_uP(void){

	Getdata(&rxdata);   /* Get fresh data from the master microprocessor. */

//	Encoder_Arm_Count=(int)f_Encoder_Arm_Count;
	Encoder_Arm_Count=(int)f_Encoder_Arm_Count + Arm_Start_Position_Adj;

	Encoder_Trolley_Count=(int)f_Encoder_Trolley_Count;


	LabView();		// Read LabView Input
//	Get_EEPROM_Data();	// Get data from EEPROM
	Handle_Sensors();	// Get Specific Sensor Input
	//*trolley.value=Encoder_Trolley_Count;
	Handle_OI();		// Get OI Requests
	Handle_States();	// Set States
	if (!Arm_Trolley_Joystick_Control){ //Make sure 
		Move_Bot();		// Move Bot based on States
	}
	Move_Trolley();		// Move Trolley based on States
	Move_Arm();		// Move Arm based on States
	Move_Gripper();		// Move Gripper based on States

	LabView_Out();		// Write Output to LabView or Data loggr
	
	Putdata(&txdata);             /* DO NOT CHANGE! */
}

void LabView_Out(){

	debug(32896);	
	debug(32896);	
	debug(50);
	debug(OI_Button_Place_Hurdle);		//0
	debug(OI_Button_Poke);
	debug(OI_Button_Tuck);
	debug(OI_Button_Capture);
	debug(OI_Button_Ball_Release);
	debug(OI_Button_Home);			//5
	debug(OI_Button_Function_Override);
	debug(OI_Button_Arm_Top);
	debug(OI_Button_Arm_Out);
	debug(OI_Button_Grip_OpenClose);
	debug(OI_Button_Trolley_Top);		//10
	debug(OI_Button_Trolley_Down);
	debug(OI_Button_Nudge_Arm_Up);
	debug(OI_Button_Nudge_Arm_Down);
	debug(OI_Button_Trans);
	debug(OI_Button_Shift_Override);	//15
	debug(OI_Drive_x);
	debug(OI_Drive_y);
	debug(RC_Motor_Drive_Left);
	debug(RC_Motor_Drive_Right);
	debug(RC_Motor_Trolley);		//20
	debug(RC_Motor_Arm);
	debug(RC_Bar_Detect);
	debug(RC_Grip_Open);
	debug(RC_Grip_Close);
	debug(RC_Grip_Left);			//25
	debug(RC_Grip_Right);
	debug(RC_Arm_Home);
	debug(RC_Arm_Max_Up);
	debug(RC_Pneum_Grip_Left_Open);
	debug(RC_Pneum_Grip_Left_Close);	//30
	debug(RC_Pneum_Trans_Low);
	debug(RC_Pneum_Trans_High);
	debug(RC_Pneum_Compressor_Start);
	debug_long(Encoder_Left_Count);		//34,35
	debug_long(Encoder_Right_Count);	
	debug_long(Encoder_Trolley_Count);
	debug_long(Encoder_Arm_Count);		//40
	debug(Mode_State);
	debug(Arm_Position_State);
	debug(Trolley_Position_State);
	debug((int)(arm.KP*10000));			//45
	debug((int)(arm.KI*10000));
	debug((int)(arm.KD*10000));
//	global1=trolley.hold_value;
	global1=Arm_Home;
	debug(global1);
//	global2=arm.hold_value;
	global2=Arm_Start_Position_Adj;
	debug(global2);


	/*
	debug(OI_Drive_x);						//0
	debug(OI_Drive_y);
	debug(Light1);
	debug(Light2);
	debug(Light3);
	debug(Light4);							//5
	debug(Light5);							
	debug(Light6);
	debug(Light7);
	debug(Light8);
	debug((unsigned int)((battery_voltage)*100));			//10
	debug(status);							
	debug_long(Encoder_1_Count);
	debug_long(Encoder_2_Count);				
	debug(Encoder_1_Speed);
	debug(Encoder_2_Speed);
	debug(pollingtime);
	debug(pwm02);
	debug((int)(right.KP*10000));					//20
	debug((int)(right.KI*10000));
	debug((int)(right.KD*10000));
	debug(Get_Analog_Value(rc_ana_in14));
	debug(Get_Analog_Value(rc_ana_in15));
	debug(Get_Analog_Value(rc_ana_in16));				//25
	debug(inputmagics);
	debug(bytecount);
	debug_long(turncount);
	debug(turningzero1);						//30
	debug_long(abs(((signed long)(Encoder_1_Count-turningzero1))));			
	debug(Get_Gyro_Bias());
	debug(Get_Gyro_Rate());
	debug(Get_Gyro_Angle());					//35
	debug(turningzero2);
	debug_long(abs(((signed long)(Encoder_2_Count-turningzero2))));			
	debug(tmp2);*/
	/*debug(Get_Gyro_Bias());
	debug(Get_Gyro_Rate());
	debug((unsigned int)Get_Gyro_Angle());
	debug(65535*Light1);						//20*/
}


void LabView(void)
{
// LabView Input
// *** Put EEPROM writes here ***
//
// Need Left Motor values
//
	unsigned char bytecount=0;
	unsigned char data=0;
	unsigned char data2=0;
	unsigned int variablecount=0;
	unsigned char currentvar=0;
	unsigned char inputpktstate=0;
	static int inputmagics=0;
	static unsigned int i = 0;
	static unsigned int j = 0;

	/* LabView Interface - Get variables */
	bytecount=Serial_Port_One_Byte_Count();
	if(bytecount>0){
		for (j=0;j<bytecount;j++){
			data=Read_Serial_Port_One();
			if (data!=128 && inputpktstate<4){
					inputpktstate=0;
			}else if (data==128 && inputpktstate<4){	
				inputpktstate++;
			}else if(inputpktstate==4 && variablecount==0){
				data2=Read_Serial_Port_One();
				variablecount=data2;
				j++;
				currentvar=0;
			}else if (variablecount>0){
				data2=Read_Serial_Port_One();
				j++;
				switch (currentvar){
					case 0:				// Program Mode
						if (data2==1){
							rc_dig_out18=0;
						}
						break;
					case 1:				// P Value - Right Motor
						inputmagics=data;inputmagics=inputmagics<<8;inputmagics+=data2;
						right.KP=((float)inputmagics)/10000;
						break;	
					case 2:				// I Value - Right Motor
						inputmagics=data;inputmagics=inputmagics<<8;inputmagics+=data2;
						right.KI=((float)inputmagics)/10000;
						break;	
					case 3:				// D Value - Right Motor
						inputmagics=data;inputmagics=inputmagics<<8;inputmagics+=data2;
						right.KD=((float)inputmagics)/10000;
						break;	
					case 4:				// P Value - Left Motor 
						inputmagics=data;inputmagics=inputmagics<<8;inputmagics+=data2;
						left.KP=((float)inputmagics)/10000;
						break;	
					case 5:				// I Value - Left Motor
						inputmagics=data;inputmagics=inputmagics<<8;inputmagics+=data2;
						left.KI=((float)inputmagics)/10000;
						break;	
					case 6:				// D Value - Left Motor
						inputmagics=data;inputmagics=inputmagics<<8;inputmagics+=data2;
						left.KD=((float)inputmagics)/10000;
						break;	
					case 7:				// P Value - Arm Motor Up
						inputmagics=data;inputmagics=inputmagics<<8;inputmagics+=data2;
						armup.KP=((float)inputmagics)/10000;
						break;	
					case 8:				// I Value - Arm Motor Up
						inputmagics=data;inputmagics=inputmagics<<8;inputmagics+=data2;
						armup.KI=((float)inputmagics)/10000;
						break;	
					case 9:				// D Value - Arm Motor Up
						inputmagics=data;inputmagics=inputmagics<<8;inputmagics+=data2;
						armup.KD=((float)inputmagics)/10000;
						break;	
					case 10:			// P Value - Arm Motor Down
						inputmagics=data;inputmagics=inputmagics<<8;inputmagics+=data2;
						armdown.KP=((float)inputmagics)/10000;
						break;	
					case 11:			// I Value - Arm Motor Down
						inputmagics=data;inputmagics=inputmagics<<8;inputmagics+=data2;
						armdown.KI=((float)inputmagics)/10000;
						break;	
					case 12:			// D Value - Arm Motor Down
						inputmagics=data;inputmagics=inputmagics<<8;inputmagics+=data2;
						armdown.KD=((float)inputmagics)/10000;
						break;
					case 13:			// P Value - Trolley Motor Up
						inputmagics=data;inputmagics=inputmagics<<8;inputmagics+=data2;
						trolleyup.KP=((float)inputmagics)/10000;
						break;	
					case 14:			// I Value - Trolley Motor Up
						inputmagics=data;inputmagics=inputmagics<<8;inputmagics+=data2;
						trolleyup.KI=((float)inputmagics)/10000;
						break;	
					case 15:			// D Value - Trolley Motor Up
						inputmagics=data;inputmagics=inputmagics<<8;inputmagics+=data2;
						trolleyup.KD=((float)inputmagics)/10000;
						break;	
					case 16:			// P Value - Trolley Motor Down
						inputmagics=data;inputmagics=inputmagics<<8;inputmagics+=data2;
						trolleydown.KP=((float)inputmagics)/10000;
						break;	
					case 17:			// I Value - Trolley Motor Down
						inputmagics=data;inputmagics=inputmagics<<8;inputmagics+=data2;
						trolleydown.KI=((float)inputmagics)/10000;
						break;	
					case 18:			// D Value - Trolley Motor Down
						inputmagics=data;inputmagics=inputmagics<<8;inputmagics+=data2;
						trolleydown.KD=((float)inputmagics)/10000;
						break;

				}
				currentvar++;
			}
			if (currentvar>variablecount){
				variablecount=0; inputpktstate=0;
			}
		}
	}
}

void Handle_Sensors(void)
{
	static char Bar_Detect_Count=0;

// Am I tilting?
/* --> Backward tilt 	*/
/* --> Side tilt	*/

// Has overpass bar been detected?  (Need to check for switch bounce....needs to be TRUE for 5 cycles to be a valid Detect)
// Need to ensure that a passing robot does not cause a premature poke, hurdle or place
	if (RC_Bar_Detect){
		if (Bar_Detect_Count > Bar_Detect_Count_Max){
			State_Overpass_Detected=Overpass_Detected;
			Bar_Detect_Count = 0;
		} else {
			Bar_Detect_Count += 1;
		}
	} else {
		State_Overpass_Detected=Overpass_Not_Detected;
	}
}
void Handle_OI(void)
{

// OI Functions (non-override...and one override)
// Certain requests can not be performed if the bot is not in a state to execute the request
// e.g. Can't hurdle if a ball is not captured
//
// Initial call to request processor to set the initial states  All subsequent loops will be handled by the Handle_States routine

	if (OI_Button_Capture && Gripper_State!=Grip_Ball_State){
		/* --> Move arm/trolley/open gripper to capture the ball */
		OI_Request = Mode_Capture_State;

	}else if (OI_Button_Tuck && Gripper_State==Grip_Ball_State){
		/* --> Move arm/trolley to tuck position */
		OI_Request = Mode_Tuck_State;

	}else if (OI_Button_Place_Hurdle && Gripper_State==Grip_Ball_State){
		/* --> Move arm/trolley to hurdle/place position */
		OI_Request = Mode_Place_Hurdle_State;

	}else if (OI_Button_Poke && Gripper_State!=Grip_Ball_State){ //can't poke with a ball in possession
		/* --> Move arm/trolley/close gripper to Poke_Ready position and do the poke*/
		OI_Request = Mode_Poke_State;

	}else if (OI_Button_Home && Gripper_State!=Grip_Ball_State){
		/* --> Move arm/trolley/close gripper to Home position */
		OI_Request = Mode_Home_State;

	}else if (OI_Button_Ball_Release && Gripper_State==Grip_Ball_State){
			/* --> Position trolley/arm to preset release position and release ball	*/
			OI_Request = Mode_Ball_Release_State;
	} else {
			OI_Request = Mode_State; // No request from OI (assign current state)
	}
}

// Use at your own risk
// Use at your own risk
// Use at your own risk
//
// OI Functions (Function Override) - Manual Actions
// The following will fall through to the arm, trolley and gripper move routines.  Use at your own risk.
// - OI_Button_Nudge_Arm_Up
// - OI_Button_Nudge_Arm_Down
// - OI_Button_Trolley_Up
// - OI_Button_Trolley_Down
// - OI_Button_Arm_Up
// - OI_Button_Arm_Out
// - OI_Button_Grip_OpenClose
//
// Use at your own risk
// Use at your own risk
// Use at your own risk


void Handle_States(void)
{
// Handle calling the routines to set the states based on what happened since last loop

//	if (Arm_Position_State!=Unknown_State && Arm_Position_State!=Arm_Searching_State && Trolley_Position_State!=Unknown_State && Trolley_Position_State!=Trolley_Searching_State){
	if (Arm_Position_State!=Unknown_State && Arm_Position_State!=Arm_Searching_State){
		if (Mode_State==Unknown_State){
			Mode_State = OI_Request;
		}
		if (Mode_State!=OI_Request){		// Did the OI request a change in function? Do we know where the arm and trolley are?
			switch (Mode_State){		// Yes..perform clean-up from previous function that was not allowed to complete
				case Mode_Capture_State:
					Mode_State = OI_Request;
					Capture_Ball_Request();
					break;
				case Mode_Tuck_State:
					Mode_State = OI_Request;
					Tuck_Request();
					break;
				case Mode_Tuck_Done_State: // Do nothing as it is done....nothing to clean-up
					Mode_State = OI_Request;
					break;	
				case Mode_Place_Hurdle_State:
					Mode_State = OI_Request;
					Place_Hurdle_Request();
					break;	
				case Mode_Poke_State:
					Mode_State = OI_Request;
					Poke_Ball_Request();
					break;
				case Mode_Poke_Done_State: // Do nothing as it is done...nothing to clean-up
					Mode_State = OI_Request;
					break;
				case Mode_Home_State:
					Mode_State = OI_Request;
					Home_Request();
					break;
				case Mode_Home_Done_State: // Do nothing as it is done...nothing to clean-up
					Mode_State = OI_Request;
					break;
				case Mode_Ball_Release_State:
					Mode_State = OI_Request;
					Ball_Release_Request();
					break;
					}
		}else {	
			switch (Mode_State){		// Are we in one of the below states...and did we lose the ball???
				case Mode_Tuck_State:
					if (RC_Grip_Close){
						Mode_State = Mode_Home_State;
						Tuck_Request();
					}
					break;
				case Mode_Tuck_Done_State: // Nothing to clean-up....just go home.
					if (RC_Grip_Close){
						Mode_State = Mode_Home_State;
					}
					break;	
				case Mode_Place_Hurdle_State:
					if (RC_Grip_Close){
						Mode_State = Mode_Home_State;
						Place_Hurdle_Request();
					}
					break;	
				}
	}	
	
	
		switch (Mode_State){	// Process request - routine will determine if this the first call and process accordingly
			case Mode_Capture_State:
				Capture_Ball_Request();	
				break;
			case Mode_Tuck_State:
				Tuck_Request();
				break;
			case Mode_Tuck_Done_State: // Do nothing as it is done....
				break;
			case Mode_Place_Hurdle_State:
				Place_Hurdle_Request();
				break;
			case Mode_Poke_State:
				Poke_Ball_Request();
				break;
			case Mode_Poke_Done_State: // Do nothing as it is done...
				break;
			case Mode_Home_State:
				Home_Request();
				break;
			case Mode_Home_Done_State: // Do nothing as it is done...
				break;
			case Mode_Ball_Release_State:
				Ball_Release_Request();
				break;
		}
	}
}
void Capture_Ball_Request(void)
{
	//The sequence of events for capture
	// a) Move arm and trolley to capture position; Open gripper
	// b) Gripper detects ball and closes
	// c) Move to Tuck State

	static char Capture_Ball_Request = 0;

	if (Capture_Ball_Request==0){	// Capture set up
		Capture_Ball_Request = 1;  // Prevent going in to 'set up' every time the button is pushed and the first request is being executed
		State_Capture = Capture_Ball_Position;  // Indicate to move arm and trolley to position
		Arm_Position_State = Trolley_Position_State = Move_To_Position_State;
		Gripper_Position_State = Bot_Position_State = 0; // Ensure these are clear of previous states from other requests
	}
	if (Mode_State==Mode_Capture_State){
		if (State_Capture==Capture_Ball_Position && Arm_Position_State==Capture_Ready && Trolley_Position_State==Capture_Ready && Gripper_State==Grip_Open_State){
			State_Capture = Capture_Ready; // Now ready to capture ball
		}
		if (State_Capture==Capture_Ready && Gripper_State==Grip_Ball_State){
			Mode_State = Mode_Tuck_State; // Got the ball...now protect it...go to tuck
			Arm_Position_State = Trolley_Position_State = Bot_Position_State = Gripper_Position_State = State_Capture = Capture_Ball_Request = 0; // Reset Capture states
		}
	}else { // The Mode_Sate changed in the middle of executing Capture_Ball; Clean-up all in-process states
		Arm_Position_State = Trolley_Position_State = Bot_Position_State = Gripper_Position_State = State_Capture = Capture_Ball_Request = 0;
	}
}


void Home_Request(void)
{
	//The sequence of events for Home
	// a) move trolley up (close gripper)
	// b) move arm down
	// c) move trolley down
	// d) set state to be home_done
	static char Home_Request = 0;


	if (Home_Request==0){	// Home set up
		Home_Request = 1;  // Prevent going into 'set up' every time the button is pushed and the first request is being executed
		State_Home = Home_Position;  // Indicate to move arm and trolley to position
		Trolley_Position_State = Move_To_Position_State;
		Arm_Position_State = Wait_State;
		Gripper_Position_State = Bot_Position_State = 0; // Ensure these are clear of previous states from other requests
	}
	if (Mode_State==Mode_Home_State){
		if (State_Home==Home_Position && Arm_Position_State==Home_Ready && Trolley_Position_State==Home_Ready && Gripper_State==Grip_Close_State){
			Mode_State = Mode_Home_Done_State; // Home!
			Arm_Position_State = Trolley_Position_State = Bot_Position_State = Gripper_Position_State = State_Home = Home_Request = 0; // Reset Capture states
		}
	}else { // The Mode_State changed in the middle of executing Capture_Ball; Clean-up all in-process states
		Arm_Position_State = Trolley_Position_State = Bot_Position_State = Gripper_Position_State = State_Home = Home_Request = 0;
	}
}

void Ball_Release_Request(void)
{
	//The sequence of events for Ball Release
	// a) Move trolley up
	// b) Initiate move arm up at defined trolley point
	// b) as soon as arm/trolley are in position then open gripper
	// c) Go to Home State
	static char Ball_Release_Request = 0;
			
	if (Ball_Release_Request==0){	// Ball release set up
		Ball_Release_Request = 1;  // Prevent going into 'set up' every time the button is pushed and the first request is being executed
		State_Ball_Release = Ball_Release_Position;  // Indicate to move arm and trolley to position
		Trolley_Position_State = Move_To_Position_State;
		Arm_Position_State = Wait_State;
		Gripper_Position_State = Bot_Position_State = 0; // Ensure these are clear of previous states from other requests
	}
	if (Mode_State==Mode_Ball_Release_State){
		if (State_Ball_Release==Ball_Release_Position && Arm_Position_State==Ball_Release_Ready && Trolley_Position_State==Ball_Release_Ready){
			Gripper_State = Gripper_Opening_State;
		}
		if (State_Ball_Release==Ball_Release_Position && Arm_Position_State==Ball_Release_Ready && Trolley_Position_State==Ball_Release_Ready && Gripper_State==Grip_Open_State){
			Mode_State = Mode_Home_State; // The ball has been release...go Home!
			Arm_Position_State = Trolley_Position_State = Bot_Position_State = Gripper_Position_State = State_Ball_Release = Ball_Release_Request = 0; // Reset Capture states
		}
	}else { // The Mode_State changed in the middle of executing Ball_Release; Clean-up all in-process states
		Arm_Position_State = Trolley_Position_State = Bot_Position_State = Gripper_Position_State = State_Ball_Release = Ball_Release_Request = 0;}

}

void Tuck_Request(void)
{
	//The sequence of events for Tuck
	// a) move trolley up
	// b) move arm down
	// c) move trolley down
	// d) set state to be tuck_done
	static char Tuck_Ball_Request = 0;
	
	if (Tuck_Ball_Request==0){	// Tuck set up
		Tuck_Ball_Request = 1;  // Prevent going into 'set up' every time the button is pushed and the first request is being executed
		State_Tuck = Tuck_Position;  // Indicate to move arm and trolley to position
		Trolley_Position_State = Move_To_Position_State;
		Arm_Position_State = Wait_State;
		Gripper_Position_State = Bot_Position_State = 0; // Ensure these are clear of previous states from other requests
	}
	if (Mode_State==Mode_Tuck_State){
		if (State_Tuck==Tuck_Position && Arm_Position_State==Tuck_Ready && Trolley_Position_State==Tuck_Ready && Gripper_State==Grip_Ball_State){
			Mode_State = Mode_Tuck_Done_State; // The ball has been tucked
			Arm_Position_State = Trolley_Position_State = Bot_Position_State = Gripper_Position_State = State_Tuck = Tuck_Ball_Request = 0; // Reset Capture states
		}
	}else { // The Mode_State changed in the middle of executing Capture_Ball; Clean-up all in-process states
		Arm_Position_State = Trolley_Position_State = Bot_Position_State = Gripper_Position_State = State_Tuck = Tuck_Ball_Request = 0;}

}

void Place_Hurdle_Request(void)
{
	//The sequence of events for Place/Hurdle
	// a) Move arm to Hurdle/Place position and wait for bar to be detected
	// ??? need to slow the bot so as not to get in to a tilt situation
	// b) Bar detect - Ignore driver input (unless shift override is pressed) open gripper and stop bot
	// c) Backup bot (auto)
	// e) Bot stops, retract arm, lower trolley (return control to driver); go to Tuck State
	static char Place_Hurdle_Request = 0;

	if (Place_Hurdle_Request==0){	// Tuck set up
		Place_Hurdle_Request = 1;  // Prevent going into 'set up' every time the button is pushed and the first request is being executed
		State_Place_Hurdle = Place_Hurdle_Position;  // Indicate to move arm and trolley to position
		Trolley_Position_State = Move_To_Position_State;
		Arm_Position_State = Wait_State;
		Bot_Position_State = Bot_Creep_State;
		Gripper_Position_State = 0;
	}
	if (Mode_State==Mode_Place_Hurdle_State){
		if (State_Place_Hurdle==Place_Hurdle_Position && Arm_Position_State==Place_Hurdle_Ready && Trolley_Position_State==Place_Hurdle_Ready && State_Overpass_Detected==Overpass_Detected){
			Bot_Position_State = Wait_State;
		} 
		if (State_Place_Hurdle==Place_Hurdle_Position && Arm_Position_State==Place_Hurdle_Ready && Trolley_Position_State==Place_Hurdle_Ready && Gripper_State==Grip_Open_State){
			State_Place_Hurdle = Place_Hurdle_Done; //Arm is up, Trolley is up, ..ball has launched
			Arm_Position_State = Action_Retracting_Arm;
			Trolley_Position_State = Wait_State;
		}
 		if (State_Place_Hurdle==Place_Hurdle_Done && Arm_Position_State==Place_Hurdle_Arm_Retracted && Trolley_Position_State==Place_Hurdle_Trolley_Lowered){	
			Mode_State = Mode_Home_State;
			Arm_Position_State = Trolley_Position_State = Bot_Position_State = Gripper_Position_State = State_Place_Hurdle = Place_Hurdle_Request = 0; // Reset Capture states
		}
	}else { // The Mode_State changed in the middle of executing Capture_Ball; Clean-up all in-process states
		Arm_Position_State = Trolley_Position_State = Bot_Position_State = Gripper_Position_State = State_Place_Hurdle = Place_Hurdle_Request = 0;}

}

void Poke_Ball_Request(void)
{
	//The sequence of events for Poking a ball (version 1) - Detect Bar - Untested
	// a) Move arm to Hurdle/Place position and wait for bar to be detected (gripper will ensure it closes itself)
	// b) Bar detect - Ignore driver input (unless shift override is pressed) stop bot, retract arm then retract trolley
	// c) Move bot forward
	// d) Bot continuous to move forward, arm raises, trolley raises (pokes ball)
	// e) Bot stops, retract arm, lower trolley (return control to driver); go to Home State
	//
	// The following supercedes Version 1.  version 1 is commented below so as not to lose the logic during testing and
	// and determination of Version 2
	//
	// The sequence of events for Poking a ball (version 2 ) - Drive underneath
	// a) Move trolley to Poke position
	// b) Move arm to Poke position
	// c) Ensure gripper is closed
	// Note: Bot under joystick control

	static char Poke_Ball_Request = 0;

//Version 1 - Untested
//	if (Poke_Ball_Request==0){	// Poke set up
//		Poke_Ball_Request = 1;  // Prevent going in to 'set up' every time the button is pushed and the first request is being executed
//		State_Poke = Poke_Place_Hurdle_Position;  // Indicate to move arm and trolley to position
//		Trolley_Position_State = Move_To_Position_State;
//		Arm_Position_State = Wait_State;
//		Bot_Position_State = Bot_Creep_State; // Ensure these are clear of previous states from other requests
//	}
//	if (Mode_State==Mode_Poke_State){
//		if (State_Poke==Poke_Place_Hurdle_Position && State_Overpass_Detected==Overpass_Detected){
//			State_Poke = Poke_Overpass_Detected; // Causes arm to lower in Move_Arm routine
//		}
//		if (Arm_Position_State==Poke_Arm_Retracted && Trolley_Position_State==Poke_Trolley_Lowered){
//			State_Poke = Poke_Ready; // Arm and trolley down...indicates to move Bot in Move_Bot routine
//			Bot_Position_State = Bot_Poke_Move_To_Ready_State;
//		}
//		if (Bot_Position_State==Bot_Poke_Ready_State){
//			State_Poke = Poke_Up; // Arm and trolley down and bot has moved to position ... so now do the Poke
//		}
//		if (Arm_Position_State==Poke_Up_Arm_Done && Trolley_Position_State==Poke_Up_Trolley_Done){
//			State_Poke = Poke_Up_Done; //Arm is up, Trolley is up, Bot has moved forward and stopped
//		}
//		if (State_Poke==Poke_Up_Done && Arm_Position_State==Poke_Arm_Retracted && Trolley_Position_State==Poke_Trolley_Lowered){
//			Arm_Position_State = Trolley_Position_State = Bot_Position_State = State_Poke = Poke_Ball_Request = 0; // Reset poke states
//			Mode_State = Mode_Home_State;  // Protect the gripper and arm by going 'home'
//		}
//	}else { // The Mode_State changed in the middle of executing Capture_Ball; Clean-up all in-process states
//		Arm_Position_State = Trolley_Position_State = Bot_Position_State = Gripper_Position_State = State_Poke = Poke_Ball_Request = 0;}

//Version 2
	if (Poke_Ball_Request==0){	// Poke set up
		Poke_Ball_Request = 1;  // Prevent going in to 'set up' every time the button is pushed and the first request is being executed
		State_Poke = Poke_Up;  // Indicate to move arm and trolley to position
		Arm_Position_State = Trolley_Position_State = Move_To_Position_State;
		Gripper_Position_State = Wait_State;
		Bot_Position_State = 0; // Ensure these are clear of previous states from other requests
	}
	if (Mode_State==Mode_Poke_State){
		if (Arm_Position_State==Poke_Ready && Trolley_Position_State==Poke_Ready){
			Mode_State=Mode_Poke_Done_State;
			Arm_Position_State = Trolley_Position_State = Bot_Position_State = State_Poke = Poke_Ball_Request = 0; // Reset poke states
		}
	}else { // The Mode_State changed in the middle of executing Poke_Ball; Clean-up all in-process states
		Arm_Position_State = Trolley_Position_State = Bot_Position_State = Gripper_Position_State = State_Poke = Poke_Ball_Request = 0;}

}

void Move_Gripper(void){
	static char Gripper_Position_Dont_Change=0;
	static char Gripper_Start_Up=1;
	static char Gripper_Manual_Control_Button_Pushed=0;
	char tmp=0;

// should gripper be prevented from being opened in home or tuck state? 


// (Move_Gripper) Ball Capture
// Open gripper for ball capture at the last 'second' before arm/trolley are in position 
	if(State_Capture==Capture_Ball_Position){
		if (Gripper_Position_State==Gripper_Opening_State){ // Opening
	    		if (RC_Grip_Open){
				RC_Pneum_Grip_Left_Open = 0;
				RC_Pneum_Grip_Left_Close = 0;
				Gripper_State = Grip_Open_State;
				Gripper_Position_State = 0;
			}else {
				if (RC_Grip_Close){
					RC_Pneum_Grip_Left_Open = 1; // Missed the Ball....open gripper
					RC_Pneum_Grip_Left_Close = 0;
					Gripper_State = Grip_Close_State;
				}
			}
		}else {
			if (Gripper_State==Grip_Open_State){
				if (RC_Grip_Left || RC_Grip_Right){	// Ball Capture!
					RC_Pneum_Grip_Left_Open = 0;
					RC_Pneum_Grip_Left_Close = 1;
				}
			} else {
				if ((!(RC_Grip_Open || RC_Grip_Close))){
				Gripper_State = Grip_Ball_State; // Got the ball!
				RC_Pneum_Grip_Left_Open = 0;
				RC_Pneum_Grip_Left_Close = 0;
				}
			}
		}
	}

	
// (Move_Gripper) Place_Hurdle
	if(State_Place_Hurdle==Place_Hurdle_Position){
		if (State_Overpass_Detected==Overpass_Detected){ // Launch ball
			if (!(RC_Grip_Open || RC_Grip_Close)){
				RC_Pneum_Grip_Left_Open = 1;
				RC_Pneum_Grip_Left_Close = 0;
				Gripper_Position_State = Gripper_Opening_State;
			} else {
				if (RC_Grip_Open){
					RC_Pneum_Grip_Left_Open = 0;
					RC_Pneum_Grip_Left_Close = 0;
					Gripper_State = Grip_Open_State;
					Gripper_Position_State = 0;
				}
			}
		}
	}


// (Move_Gripper) Ball Release
//open gripper for ball release as soon as arm/trolley are in position 
	if(State_Ball_Release==Ball_Release_Position){
		if (Gripper_State==Gripper_Opening_State){ // Opening
	    		if ((!(RC_Grip_Open || RC_Grip_Close))){
				RC_Pneum_Grip_Left_Open = 1;
				RC_Pneum_Grip_Left_Close = 0;
				Gripper_State = Grip_Open_State;
			}else {
				if (RC_Grip_Open){
					RC_Pneum_Grip_Left_Open = 0; // Turn off spike
					RC_Pneum_Grip_Left_Close = 0;
					Gripper_State = Grip_Open_State;
					Gripper_Position_State = 0;
				}
			}
		}
	}


// (Move_Gripper) Poke
//version 1 - Untested
//	if (State_Poke==Poke_Place_Hurdle_Position){
//	       if (RC_Grip_Open){
//			RC_Pneum_Grip_Left_Close = 1;
//			RC_Pneum_Grip_Left_Close = 0;
//			Gripper_State = Grip_Open_State;
//		} else {
//			if (RC_Grip_Close){
//				RC_Pneum_Grip_Left_Close = 0;
//				RC_Pneum_Grip_Left_Open = 0;
//				Gripper_State = Grip_Close_State;
//			}
//		}
//	}
// Version 2
//	if (State_Poke==Poke_Up && Gripper_State==Move_To_Position_State){
//	       if (RC_Grip_Open){
//			RC_Pneum_Grip_Left_Close = 1;
//			RC_Pneum_Grip_Left_Close = 0;
//			Gripper_State = Grip_Open_State;
//		} else {
//			if (RC_Grip_Close){
//				RC_Pneum_Grip_Left_Close = 0;
//				RC_Pneum_Grip_Left_Open = 0;
//				Gripper_State = Grip_Close_State;
//			}
//		}
//	}

// (Move_Gripper) Home
	if (State_Home==Home_Position){
		if (Gripper_Position_State==Gripper_Closing_State){
			if (RC_Grip_Open){
				RC_Pneum_Grip_Left_Close = 1;
				RC_Pneum_Grip_Left_Open = 0;
				Gripper_State = Grip_Close_State;
			} else {
				if (RC_Grip_Close){
					RC_Pneum_Grip_Left_Close = 0;
					RC_Pneum_Grip_Left_Open = 0;
					Gripper_Position_State = 0;
				}
			       	else {
					RC_Pneum_Grip_Left_Close = 0;
					RC_Pneum_Grip_Left_Open = 1; // if not open and not close...reopen so to try to close again
				}
			}
		}
	}

// (Move_Gripper) Manually open/close gripper.  It changes to the opposite state.  If it has the ball...it will drop it
//	if (OI_Button_Function_Override && OI_Button_Grip_OpenClose){
	if (OI_Button_Grip_OpenClose){
		Gripper_Position_State = Gripper_Manual_Control_State;
		if (Gripper_Manual_Control_Button_Pushed==0){
			Gripper_Manual_Control_Button_Pushed = 1;
			if (RC_Grip_Open){
				RC_Pneum_Grip_Left_Close = 1;
				RC_Pneum_Grip_Left_Open = 0;
				Gripper_State = Grip_Open_State;
			} else {
				if (RC_Grip_Close){
					RC_Pneum_Grip_Left_Open = 1;
					RC_Pneum_Grip_Left_Close = 0;
					Gripper_State = Grip_Close_State;
				} else {
					RC_Pneum_Grip_Left_Open = 1;
					RC_Pneum_Grip_Left_Close = 0;
					Gripper_State = Grip_Ball_State;
					}
				}
		} else { // I am here...means the button has been pushed once and now I need to turn off spikes
			if (RC_Grip_Open){
				RC_Pneum_Grip_Left_Open = 0;
				Gripper_State = Grip_Open_State;
			} else {
				if (RC_Grip_Close){
					RC_Pneum_Grip_Left_Close = 0;
					Gripper_State = Grip_Close_State;
					}
				}
			}
	}else {
		Gripper_Manual_Control_Button_Pushed = 0;
		}


// (Move_Gripper) Start Up
// Determine: should I open/close, override of automatic Open/close at startup or gripper open/close button push.  If not changing gripper then only the gripper open/close button can be used to move gripper 

// Never start the robot with a ball in the gripper.  There is no code to to try to figure out what to do.

	if ((!(RC_Grip_Open || RC_Grip_Close)) && Gripper_Start_Up==1){
		Gripper_State=Unknown_State;
		Gripper_Start_Up = 0;
	}else {
		if (RC_Grip_Open){
			Gripper_State = Grip_Open_State;
			Gripper_Start_Up = 0;
		}else {
			if (RC_Grip_Close){
				Gripper_State = Grip_Close_State;
				Gripper_Start_Up = 0;
			} else {
				Gripper_State = Grip_Ball_State;
				}
			}
	}
	if (Gripper_State==Unknown_State){
	       if ((!(OI_Button_Home && OI_Button_Capture) && Gripper_Position_Dont_Change==0) || (OI_Button_Function_Override && OI_Button_Grip_OpenClose)){
			Gripper_State=Gripper_Closing_State;
	       }else{
		       if (Gripper_State==Unknown_State){
			       Gripper_Position_Dont_Change=1;
		       }
	       }
		if (Gripper_State==Gripper_Closing_State){
		       // initiate spike_forward
			if (RC_Grip_Open){
				Gripper_State = Grip_Open_State;
				//initiate spike_reverse so that we can close the gripper
			}else{
				if (RC_Grip_Close){
					Gripper_State = Grip_Close_State;
				}
			}
		}			
	}


}

void Move_Arm(){
	static char Arm_Nudge_Button_State=0;
	static char Arm_Position_Dont_Search=0;
	signed int tmp=0;

//	
/* Add code to prevent arm movement if the trolley is too low so as not to get caught on bot/floor. Asked Mrk K. for approx. encoder count of trolley and arm encoder clicks. */
//
//	


// (Move_Arm) Capture Ball
	if (Mode_State==Mode_Capture_State){
		if (Arm_Position_State==Move_To_Position_State){
			arm.hold_value=Arm_Capture;
			if (Arm_Capture>Encoder_Arm_Count){
				PID_Flop(&arm, armdown);
			}else {
				PID_Flop(&arm, armup);
			}
		       if (abs(Arm_Capture-Encoder_Arm_Count)<=Arm_Target_Range_Threshold){
				Arm_Position_State = Capture_Ready;
			}
			if (abs(Arm_Capture_Move_Gripper-Encoder_Arm_Count)<=Arm_Target_Range_Threshold){
				Gripper_Position_State = Gripper_Opening_State;
			}
		}
	}

// (Move_Arm) Place_Hurdle
	if (Mode_State==Mode_Place_Hurdle_State){
		if (Arm_Position_State==Move_To_Position_State){
			arm.hold_value=Arm_Place_Hurdle;
			if (Arm_Place_Hurdle>absint(Encoder_Arm_Count-Arm_Start_Position_Adj)){
				PID_Flop(&arm, armdown);
			}else {
				PID_Flop(&arm, armup);
			}
		       if (abs(Arm_Place_Hurdle-Encoder_Arm_Count)<=Arm_Target_Range_Threshold){
				Arm_Position_State = Place_Hurdle_Ready;
			}
		} else {
			if (Arm_Position_State==Action_Retracting_Arm){
				arm.hold_value=Arm_Place_Hurdle_Retract;
				if (Arm_Place_Hurdle_Retract>Encoder_Arm_Count){
					PID_Flop(&arm, armdown);
				}else {
					PID_Flop(&arm, armup);
				}
			       if (abs(Arm_Place_Hurdle_Retract-Encoder_Arm_Count)<=Arm_Target_Range_Threshold){
					Arm_Position_State = Place_Hurdle_Arm_Retracted;
					Trolley_Position_State = Action_Lowering_Trolley;
				}
			}
		}
	}	



// (Move_Arm) Ball Release
	if (Mode_State==Mode_Ball_Release_State){
		if (Arm_Position_State==Move_To_Position_State){
			arm.hold_value=Arm_Ball_Release;	
			if (Arm_Ball_Release>Encoder_Arm_Count){
				PID_Flop(&arm, armdown);
			}else {
				PID_Flop(&arm, armup);
			}
		       if (abs(Arm_Ball_Release-Encoder_Arm_Count)<=Arm_Target_Range_Threshold){
				Arm_Position_State = Ball_Release_Ready;
			}
		}
	}

// (Move_Arm) Tuck
	if (Mode_State==Mode_Tuck_State){
		if (Arm_Position_State==Move_To_Position_State){
			arm.hold_value=Arm_Tuck;
			if (Arm_Tuck>Encoder_Arm_Count){
				PID_Flop(&arm, armdown);
			}else {
				PID_Flop(&arm, armup);
			}
		       if (abs(Arm_Tuck-Encoder_Arm_Count)<=Arm_Target_Range_Threshold){
				Arm_Position_State = Tuck_Ready;
				Trolley_Position_State=Action_Lowering_Trolley;
			}
		}
	}

// (Move_Arm) Home
	if (Mode_State==Mode_Home_State){
		if (Arm_Position_State==Move_To_Position_State){
			arm.hold_value=Arm_Home;
			if (Arm_Home>Encoder_Arm_Count){
				PID_Flop(&arm, armdown);
			}else {
				PID_Flop(&arm, armup);
			}
		       if ((abs(Arm_Home-Encoder_Arm_Count))<=Arm_Target_Range_Threshold){
				Arm_Position_State = Home_Ready;
				Trolley_Position_State=Action_Lowering_Trolley;
			}
		}
	}


// (Move_Arm) Poke
// Version 1 - Untested
//	if (Mode_State==Mode_Poke_State){
//		if (State_Poke==Poke_Place_Hurdle_Position && Arm_Position_State==Move_To_Position_State){ // Place_Hurdle position
//			arm.hold_value=Arm_Place_Hurdle;
//			if (Arm_Place_Hurdle>Encoder_Arm_Count){
//				PID_Flop(&arm, armdown);
//			}else {
//				PID_Flop(&arm, armup);
//			}
//		       if (abs(Arm_Place_Hurdle-Encoder_Arm_Count)<=Arm_Target_Range_Threshold){
//				Arm_Position_State = Wait_State;
//			}
//		}
//		if (State_Poke==Poke_Overpass_Detected){
//			Arm_Position_State=Action_Retracting_Arm;
//			arm.hold_value=Arm_Poke_Ready;
//			if (Arm_Poke_Ready>Encoder_Arm_Count){
//				PID_Flop(&arm, armdown);
//			}else {
//				PID_Flop(&arm, armup);
//			}
//		       if (abs(Arm_Poke_Ready-Encoder_Arm_Count)<=Arm_Target_Range_Threshold){
//				Arm_Position_State = Poke_Arm_Retracted;
//				Trolley_Position_State = Action_Lowering_Trolley; // Once arm is retracted then cause trolley to lower in Move_Trolley routine
//			}
//		}else {
//			if (State_Poke==Poke_Up){
//				arm.hold_value=Arm_Poke_Up;
//				if (Arm_Poke_Up>Encoder_Arm_Count){
//					PID_Flop(&arm, armdown);
//				}else {
//					PID_Flop(&arm, armup);
//				}
//			       if (abs(Arm_Poke_Up-Encoder_Arm_Count)<=Arm_Target_Range_Threshold){
//					Arm_Position_State = Poke_Up_Arm_Done;
//				}
//			}else {
//				if (State_Poke==Poke_Up_Done){
//					Arm_Position_State = Action_Retracting_Arm;
//					arm.hold_value=Arm_Poke_Ready;
//					if (Arm_Poke_Ready>Encoder_Arm_Count){
//						PID_Flop(&arm, armdown);
//					}else {
//						PID_Flop(&arm, armup);
//					}
//				       if (abs(Arm_Poke_Ready-Encoder_Arm_Count)<=Arm_Target_Range_Threshold){
//						Arm_Position_State = Poke_Arm_Retracted;
//					}
//				}
//			}
//		}	
//	}
//
// Version 2
	if (Mode_State==Mode_Poke_State){
		if (Arm_Position_State==Move_To_Position_State){
			arm.hold_value=Arm_Poke;
			if (Arm_Poke>Encoder_Arm_Count){
				PID_Flop(&arm, armdown);
			}else {
				PID_Flop(&arm, armup);
			}
		      	if (abs(Arm_Poke-Encoder_Arm_Count)<=Arm_Target_Range_Threshold && Arm_Position_State != Poke_Up_Arm_Done){
			 	Arm_Position_State = Poke_Up_Arm_Done;
				PID_Flop(&trolley,trolleyup);
				trolley.hold_value=Trolley_Poke;
			}
		}else if(Arm_Position_State == Poke_Up_Arm_Done && Trolley_Position_State!=Poke_Up_Trolley_Done){
			if (abs(Trolley_Poke-Encoder_Trolley_Count)<=Trolley_Target_Range_Threshold){
				Trolley_Position_State=Poke_Up_Trolley_Done;
			}
		}else if(Trolley_Position_State==Poke_Up_Trolley_Done){
			trolley.hold_value=Trolley_Poke_Ready;
			PID_Flop(&trolley,trolleydown);
			Arm_Position_State = Poke_Arm_Done;
		}else if(Trolley_Position_State!=Poke_Trolley_Done){
			if (abs(Trolley_Poke-Encoder_Trolley_Count)<=Trolley_Target_Range_Threshold){
				Trolley_Position_State=Poke_Trolley_Done;
			}
		}
	}	


// (Move_Arm) Nudge
// Set Nudge adjustments to target location. Zero is at top so the arm moving down will increment the encoder click count
// Ensure that holding the button down will not alow continuous arm movement.  Requires button to be released for at least one cycle.
	if ((OI_Button_Nudge_Arm_Up || OI_Button_Nudge_Arm_Down) && Arm_Nudge_Button_State==0){
		if (OI_Button_Nudge_Arm_Up){
			arm.hold_value-=Arm_Nudge_Click_Change; //Adjust target by Nudge amount
			PID_Flop(&arm, armup);
		}else if (OI_Button_Nudge_Arm_Down){
			arm.hold_value+=Arm_Nudge_Click_Change; //Adjust target by Nudge amount
			PID_Flop(&arm, armdown);
		}
		Arm_Nudge_Button_State=1;
	}else if (!(OI_Button_Nudge_Arm_Up || OI_Button_Nudge_Arm_Down) && Arm_Nudge_Button_State==1){
		Arm_Nudge_Button_State=0;
	}

// (Move_Arm) Manually move Arm to top
	if (OI_Button_Function_Override && OI_Button_Arm_Top){
		PID_Flop(&arm, armup);
		arm.hold_value=Arm_Top;	
	}


// (Move_Arm) Manually move Arm to Out
	if (OI_Button_Function_Override && OI_Button_Arm_Out){
		PID_Flop(&arm, armdown);
		arm.hold_value=Arm_Out;	
	}

// (Move_Arm) Start Up
/* Determine: searching, override of search at startup or Arm Top button push.  If not searching at position is unknown then only the Arm Top button can be used to move arm */
	if (Arm_Position_State==Unknown_State || Arm_Position_State==Arm_Searching_State){
	       if ((!(OI_Button_Home && OI_Button_Capture && OI_Button_Tuck) && Arm_Position_Dont_Search==0) || OI_Button_Arm_Top){
			Arm_Position_State=Arm_Searching_State;
	       }else{
		       if (Arm_Position_State==Unknown_State){
			       Arm_Position_Dont_Search=1;
		       }
		       if (OI_Button_Home && OI_Button_Capture && OI_Button_Tuck){ //Joystick for Arm/Trolley Only
			       Arm_Trolley_Joystick_Control = 1;
			       Mode_State = Mode_Arm_Trolley_Joystick_Control;
		       }
		       if (Arm_Trolley_Joystick_Control){
				OI_Drive_x = 255 - OI_Drive_x; //Reverse Inputs (forward/reverse)
				if (absint((int)OI_Drive_x-127) >= absint((int)OI_Drive_y - 127)){ // Only drive Arm or Trolley at one time
					OI_Drive_y = 127;
				}else{
					OI_Drive_x = 127;
				}
				OI_Drive_x=joy_condition(OI_Drive_x);
				*arm.motor=Limit_Mix(2000 + (int)OI_Drive_x);
			}
	       }
	       if (Arm_Position_State==Arm_Searching_State){
		       if (RC_Arm_Max_Up){
				Reset_Encoder_4_Count(); // Sets encoder count to zero at MAX UP position
				Arm_Position_State=Arm_Max_Top_State;
				*arm.motor=127;
				arm.hold_value=0;
			}else{
				*arm.motor=Arm_Detect_pwm_Out;  // No PID is used for Arm Search i
			}		 
	       }
	       if (RC_Arm_Home){
			Reset_Encoder_4_Count(); // Sets encoder count to zero at Home position
			Arm_Position_State=Arm_Home_State;
			Arm_Start_Position_Adj = Arm_Home; // Adjustment to arm Target calculations to adjust for Home being encoder count 0 instead of Top being encoder count 0
			*arm.motor=127;
		}
	}else{
		tmp=127-PID(arm);
		if (tmp<0){
			*arm.motor=0;
		}else if(tmp>255){
			*arm.motor=255;
		}else{
			*arm.motor=tmp;
		}
	}

}


void Move_Trolley(void){
	static char Trolley_Position_Dont_Search=0;
	static char Trolley_Search_Loopcount=0;
	static signed long Trolley_Last_Count=0;
	signed int tmp=0;

/* Add code to prevent trolley movement if the arm is angled too low so as to not jam the arm/gripper in to the floor or electronics board.  Asked Mrk K. for approx. encoder count of trolley and arm encoder clicks. */

// (Move_Trolley) Ball Release
	if (Mode_State==Mode_Ball_Release_State){
		if (Trolley_Position_State==Move_To_Position_State){
			trolley.hold_value=Trolley_Path1_Ball_Release;
			if (Trolley_Path1_Ball_Release>Encoder_Trolley_Count){
				PID_Flop(&trolley, trolleydown);
			} else {
				PID_Flop(&trolley, trolleyup);
			}
		       if (abs(Trolley_Path1_Ball_Release-Encoder_Trolley_Count)<=Trolley_Target_Range_Threshold){
				Trolley_Position_State = Trolley_Move_Arm_Position_State;
				Arm_Position_State = Move_To_Position_State;
			} 
		} else {
			if (Trolley_Position_State==Trolley_Move_Arm_Position_State){
				trolley.hold_value=Trolley_Path2_Ball_Release;
				if (Trolley_Path2_Ball_Release>Encoder_Trolley_Count){
				PID_Flop(&trolley, trolleydown);
			} else {
				PID_Flop(&trolley, trolleyup);
			}
			       if (abs(Trolley_Path2_Ball_Release-Encoder_Trolley_Count)<=Trolley_Target_Range_Threshold){
					Trolley_Position_State = Ball_Release_Ready;
				}
			}
		}
	}

// (Move_Trolley) Home
	if (Mode_State==Mode_Home_State){
		if (Trolley_Position_State==Move_To_Position_State){
			trolley.hold_value=Trolley_Path1_Home;
			if (Trolley_Path1_Home>Encoder_Trolley_Count){
				PID_Flop(&trolley, trolleydown);
			} else {
				PID_Flop(&trolley, trolleyup);
			}
		       if (abs(Trolley_Path1_Home-Encoder_Trolley_Count)<=Trolley_Target_Range_Threshold){
				Trolley_Position_State = Wait_State;
				Arm_Position_State = Move_To_Position_State;
				Gripper_Position_State = Gripper_Closing_State;
			} 
		}else {
			if (Trolley_Position_State==Action_Lowering_Trolley){
				trolley.hold_value=Trolley_Path2_Home;
				if (Trolley_Path2_Home>Encoder_Trolley_Count){
					PID_Flop(&trolley, trolleydown);
				} else {
					PID_Flop(&trolley, trolleyup);
				}
		       if (abs(Trolley_Path2_Home-Encoder_Trolley_Count)<=Trolley_Target_Range_Threshold){
				Trolley_Position_State = Home_Ready;
				}

			}
		}
	}

// (Move_Trolley) Capture Ball
	if (Mode_State==Mode_Capture_State){
		if (Trolley_Position_State==Move_To_Position_State){
			trolley.hold_value=Trolley_Capture;
			if (Trolley_Capture>Encoder_Trolley_Count){
				PID_Flop(&trolley, trolleydown);
			} else {
				PID_Flop(&trolley, trolleyup);
			}
		       if (abs(Trolley_Capture-Encoder_Trolley_Count)<=Trolley_Target_Range_Threshold){
				Trolley_Position_State = Capture_Ready;
			}
		       if (abs(Trolley_Capture_Move_Arm-Encoder_Trolley_Count)<=Trolley_Target_Range_Threshold){
				Arm_Position_State = Move_To_Position_State;
			}			
		} 
	}
// (Move_Trolley) Tuck
	if (Mode_State==Mode_Tuck_State){
		if (Trolley_Position_State==Move_To_Position_State){
			trolley.hold_value=Trolley_Path1_Tuck;
			if (Trolley_Path1_Tuck>Encoder_Trolley_Count){
				PID_Flop(&trolley, trolleydown);
			} else {
				PID_Flop(&trolley, trolleyup);
			}
		       if (abs(Trolley_Path1_Tuck-Encoder_Trolley_Count)<=Trolley_Target_Range_Threshold){
				Trolley_Position_State = Wait_State;
				Arm_Position_State = Move_To_Position_State;
			} 
		}else {
			if (Trolley_Position_State==Action_Lowering_Trolley){
				trolley.hold_value=Trolley_Path2_Tuck;
				if (Trolley_Path2_Tuck>Encoder_Trolley_Count){
					PID_Flop(&trolley, trolleydown);
				} else {
					PID_Flop(&trolley, trolleyup);
				}
			       if (abs(Trolley_Path2_Tuck-Encoder_Trolley_Count)<=Trolley_Target_Range_Threshold){
				Trolley_Position_State = Tuck_Ready;
				}

			}
		}
	}

// (Move_Trolley) Place_Hurdle
	if (Mode_State==Mode_Place_Hurdle_State){
		if (Trolley_Position_State==Move_To_Position_State){
			trolley.hold_value=Trolley_Place_Hurdle;
			if (Trolley_Place_Hurdle>Encoder_Trolley_Count){
				PID_Flop(&trolley, trolleydown);
			} else {
				PID_Flop(&trolley, trolleyup);
			}
		       if (abs(Trolley_Place_Hurdle-Encoder_Trolley_Count)<=Trolley_Target_Range_Threshold){
				Trolley_Position_State = Place_Hurdle_Ready;
			}
		       if (abs(Trolley_Place_Hurdle_Move_Arm-Encoder_Trolley_Count)<=Trolley_Target_Range_Threshold){
				Arm_Position_State = Move_To_Position_State;
			}
		} else {
			if (Trolley_Position_State==Action_Lowering_Trolley){
				trolley.hold_value=Trolley_Place_Hurdle_Lower;
				if (Trolley_Place_Hurdle_Lower>Encoder_Trolley_Count){
					PID_Flop(&trolley, trolleydown);
				} else {
					PID_Flop(&trolley, trolleyup);
				}
				if (abs(Trolley_Place_Hurdle_Lower-Encoder_Trolley_Count)<=Trolley_Target_Range_Threshold){
					Trolley_Position_State = Place_Hurdle_Trolley_Lowered;
				}
			}
		} 
	}


// (Move_Trolley) Home
	if (Mode_State==Mode_Home_State){
		if (Trolley_Position_State==Move_To_Position_State){
			trolley.hold_value=Trolley_Path1_Home;
			if (Trolley_Path1_Home>Encoder_Trolley_Count){
				PID_Flop(&trolley, trolleydown);
			} else {
				PID_Flop(&trolley, trolleyup);
			}
		       if (abs(Trolley_Path1_Home-Encoder_Trolley_Count)<=Trolley_Target_Range_Threshold){
				Trolley_Position_State = Wait_State;
				Arm_Position_State = Move_To_Position_State;
				Gripper_Position_State = Gripper_Closing_State;
			} 
		}else {
			if (Trolley_Position_State==Action_Lowering_Trolley){
				trolley.hold_value=Trolley_Path2_Home;
			       if (abs(Trolley_Path2_Home-Encoder_Trolley_Count)<=Trolley_Target_Range_Threshold){
				Trolley_Position_State = Home_Ready;
				}

			}
		}
	}
	
// (Move_Trolley) Poke
//Version 1 - Untested
//	if (Mode_State==Mode_Poke_State){
//		if (Trolley_Position_State==Poke_Place_Hurdle_Position && Trolley_Position_State==Move_To_Position_State){
//			trolley.hold_value=Trolley_Place_Hurdle;
//			if (Trolley_Place_Hurdle>Encoder_Trolley_Count){
//				PID_Flop(&trolley, trolleydown);
//			} else {
//				PID_Flop(&trolley, trolleyup);
//			}
//		       if (abs(Trolley_Place_Hurdle-Encoder_Trolley_Count)<=Trolley_Target_Range_Threshold){
//				Trolley_Position_State = Wait_State;
//			}
//		       if (abs(Trolley_Place_Hurdle_Move_Arm-Encoder_Trolley_Count)<=Trolley_Target_Range_Threshold){
//				Arm_Position_State = Move_To_Position_State;
//			}
//		}
//		if (Trolley_Position_State == Action_Lowering_Trolley){
//			trolley.hold_value=Trolley_Poke_Ready;
//			if (Trolley_Poke_Ready>Encoder_Trolley_Count){
//				PID_Flop(&trolley, trolleydown);
//			} else {
//				PID_Flop(&trolley, trolleyup);
//			}
//		       if (abs(Trolley_Poke_Ready-Encoder_Trolley_Count)<=Trolley_Target_Range_Threshold){
//				Trolley_Position_State = Poke_Trolley_Lowered;
//			}
//		}else {
//			if (State_Poke==Poke_Up){
//				trolley.hold_value=Trolley_Poke_Up;
//				if (Trolley_Poke_Up>Encoder_Trolley_Count){
//					PID_Flop(&trolley, trolleydown);
//				} else {
//					PID_Flop(&trolley, trolleyup);
//				}
//			       if (abs(Trolley_Poke_Up-Encoder_Trolley_Count)<=Trolley_Target_Range_Threshold){
//					Trolley_Position_State = Poke_Up_Trolley_Done;
//				}
//			}else {
//				if (State_Poke==Poke_Up_Done){
//					Trolley_Position_State = Action_Lowering_Trolley;
//					trolley.hold_value=Trolley_Poke_Ready;
//					if (Trolley_Poke_Ready>Encoder_Trolley_Count){
//						PID_Flop(&trolley, trolleydown);
//					} else {
//						PID_Flop(&trolley, trolleyup);
//					}
//				       if (abs(Trolley_Poke_Ready-Encoder_Trolley_Count)<=Trolley_Target_Range_Threshold){
//						Trolley_Position_State = Poke_Trolley_Lowered;
//					}
//				}
//			}
//		}	
//	}

//Version 2
	if (Mode_State==Mode_Poke_State){
		if (Trolley_Position_State==Move_To_Position_State){
			trolley.hold_value=Trolley_Poke;
			if (Trolley_Poke>Encoder_Trolley_Count){
				PID_Flop(&trolley, trolleydown);
			} else {
				PID_Flop(&trolley, trolleyup);
			}
		       if (abs(Trolley_Poke_Move_Arm-Encoder_Trolley_Count)<=Trolley_Target_Range_Threshold){
				Gripper_Position_State = Move_To_Position_State;
			}
		       if (abs(Trolley_Poke-Encoder_Trolley_Count)<=Trolley_Target_Range_Threshold){
				Trolley_Position_State = Poke_Ready;
			}
		}
	}

//Version 3
//	if (Mode_State==Mode_Poke_State){
//		Trolley_Position_State = Poke_Ready;
//	}


// (Move_Trolley) Manually move trolley to top
	if (OI_Button_Function_Override && OI_Button_Trolley_Top){
		trolley.hold_value=Trolley_Top;	
		PID_Flop(&trolley, trolleyup);
	}

// (Move_Trolley) Manually move trolley to bottom
	if (OI_Button_Function_Override && OI_Button_Trolley_Down){
		trolley.hold_value=Trolley_Bottom;
		PID_Flop(&trolley, trolleydown);
	}

// (Move_Trolley) Start Up
	if (Trolley_Position_State==Unknown_State || Trolley_Position_State==Trolley_Searching_State){
		/*
	       if ((!(OI_Button_Home && OI_Button_Capture) && Trolley_Position_Dont_Search==0) || OI_Button_Trolley_Top){
			Trolley_Position_State=Trolley_Searching_State;
	       }else{
		       if (Trolley_Position_State==Unknown_State){
			       Trolley_Position_Dont_Search=1;
		       }
       		       */if (OI_Button_Home && OI_Button_Capture && OI_Button_Tuck){ //Joystick for Arm/Trolley Only
			       Arm_Trolley_Joystick_Control = 1;
       			       Mode_State = Mode_Arm_Trolley_Joystick_Control;
		       }else if(Mode_State!=Mode_Arm_Trolley_Joystick_Control){
			       	Trolley_Position_State=Trolley_Max_Top_State;
				*trolley.motor=127;
				trolley.hold_value=0;
		       }
	       		if (Arm_Trolley_Joystick_Control){
				OI_Drive_y = 255 - OI_Drive_y; //Reverse Inputs (forward/reverse)
				if (absint((int)OI_Drive_y-127) >= absint((int)OI_Drive_x - 127)){ // Only drive Arm or Trolley at one time
					OI_Drive_x = 127;
				}else{
					OI_Drive_y = 127;
				}
				OI_Drive_y=joy_condition(OI_Drive_y);
				*trolley.motor=Limit_Mix(2000 + (int)OI_Drive_y);
			}/*
	       }
	       if (Trolley_Position_State==Trolley_Searching_State){
		       if (abs(Trolley_Last_Count-Encoder_Trolley_Count)<10 && !disabled_mode){
			       Trolley_Search_Loopcount++;
		       }else{
			       Trolley_Search_Loopcount=0;
		       }
		       if (Trolley_Search_Loopcount==Trolley_Max_Waitcount){
				Reset_Encoder_3_Count();
				Trolley_Position_State=Trolley_Max_Top_State;
				*trolley.motor=127;
				trolley.hold_value=0;
			}else{
				*trolley.motor=Trolley_Detect_pwm_Out;
			}		 
	       }*/
	}else{
		tmp=127-PID(trolley);
		if (tmp<0){
			*trolley.motor=0;
		}else if(tmp>255){
			*trolley.motor=255;
		}else{
			*trolley.motor=tmp;
		}
		Trolley_Last_Count = Encoder_Trolley_Count;
	}
}

void Move_Bot(void){
	int temp_gyro_rate;
	long temp_gyro_angle;
	int temp_gyro_bias;
	char Speed_Override=0;
	static signed long Encoder_Left_Last=0;
	static signed long Encoder_Right_Last=0;
	static unsigned char encoderloop=0;
	static unsigned char Encoder_Left_Start_Poke_Ready=0;


//need to - ramp up/down from shifting and OI speed input (prevent killing the transmission)


/*
	i++;
	k++; // this will rollover every ~1000 seconds

	if(k == 10)
	{
//		printf("\rCalculating Gyro Bias...");
	}

	if(k == 60)
	{
		// start a gyro bias calculation
		Start_Gyro_Bias_Calc();
	}

	if(k == 300)
	{
		// terminate the gyro bias calculation
		Stop_Gyro_Bias_Calc();

		// reset the gyro heading angle
		Reset_Gyro_Angle();

//		printf("Done\r");
	}


	if(i >= 30 && k >= 300)
	{
		temp_gyro_bias = Get_Gyro_Bias();
		temp_gyro_rate = Get_Gyro_Rate();
		temp_gyro_angle = Get_Gyro_Angle();

		i = 0;
	}
	


	right.KP=Get_Analog_Value(rc_ana_in14)*.0003;
	right.KI=Get_Analog_Value(rc_ana_in15)*.0003;
	right.KD=Get_Analog_Value(rc_ana_in16)*.0003;

*/
	
// Basic drive code - Joystick to wheel

//	OI_Drive_x = 255 - OI_Drive_x; //Reverse Inputs (turning)
	OI_Drive_x = OI_Drive_x; //
	OI_Drive_y = 255 - OI_Drive_y; //Reverse Inputs (forward/reverse)

	OI_Drive_x=joy_condition(OI_Drive_x);
	OI_Drive_y=joy_condition(OI_Drive_y);

	//Override Programmed Limits on OI Speed Control
	//Requires Function Override and Shift Override and no other Function buttons to be pushed..Home/Poke, etc. are OK
//	if (OI_Button_Function_Override && OI_Button_Shift_Override && !(OI_Button_Arm_Top || OI_Button_Arm_Out || OI_Button_Trolley_Top || OI_Button_Trolley_Down || OI_Button_Nudge_Arm_Up || OI_Button_Nudge_Arm_Down || OI_Button_Grip_OpenClose)){
//		Speed_Override = 1;
//	}

//Override speed control if Poke or Place_Hurdle
// Version 1 - Untested
//	if (Mode_State==Mode_Place_Hurdle_State || Mode_State==Mode_Poke_State){
//		if (Bot_Position_State==Wait_State || Bot_Position_State==Bot_Poke_Ready_State){
//			OI_Drive_x = OI_Drive_y = 127;
//		}else {
//			if (Bot_Position_State==Bot_Creep_State){
//				//Use a scaling factor to limit driver input
//				if (OI_Drive_x==127 && OI_Drive_y==127){
//					OI_Drive_y = Bot_Creep_State_pwm_out;
//				}else {
//					if (OI_Button_Home && OI_Button_Capture && OI_Button_Shift_Override){
//					} else {
//						OI_Drive_x = (((OI_Drive_x-127)*Bot_Driver_Scale_Factor)+127);
//						OI_Drive_y = (((OI_Drive_y-127)*Bot_Driver_Scale_Factor)+127);
//					}
//				}
//			} else {
//				if (Bot_Position_State==Bot_Poke_Move_To_Ready_State){
//					//Use a scaling factor to limit driver input
//					if (OI_Drive_x==127 && OI_Drive_y==127){
//						OI_Drive_y = Bot_Poke_Ready_State_pwm_out;
//					} else {
//						if (Speed_Override){
//						} else {
//							OI_Drive_x = (((OI_Drive_x-127)*Bot_Driver_Scale_Factor)+127);
//							OI_Drive_y = (((OI_Drive_y-127)*Bot_Driver_Scale_Factor)+127);
//						}
//					}
//					Encoder_Left_Start_Poke_Ready = Encoder_Left_Count;
//					Bot_Position_State=Bot_Poke_Moving_To_Ready_State;
//				} else {
//					if (Bot_Position_State==Bot_Poke_Moving_To_Ready_State){
//					//Use a scaling factor to limit driver input
//						if (OI_Drive_x==127 && OI_Drive_y==127){
//							OI_Drive_y = Bot_Poke_Ready_State_pwm_out;
//						} else {
//							if (Speed_Override){
//							} else {
//								OI_Drive_x = (((OI_Drive_x-127)*Bot_Driver_Scale_Factor)+127);
//								OI_Drive_y = (((OI_Drive_y-127)*Bot_Driver_Scale_Factor)+127);
//								}
//						}
//						if ((abs(Bot_Poke_Move_To_Ready_Position-(Encoder_Left_Count-Encoder_Left_Start_Poke_Ready)))>=Bot_Target_Range_Threshold){
//							Bot_Position_State = Bot_Poke_Ready_State;
//						}
//					}
//				}
//			}
//		}
//	}

// Version 2 (Nothing to do with Poke Mode)
//	if (Mode_State==Mode_Place_Hurdle_State){
//		if (Bot_Position_State==Wait_State){
//			OI_Drive_x = OI_Drive_y = 127;
//		}else {
//			if (Bot_Position_State==Bot_Creep_State){
//				//Use a scaling factor to limit driver input
//				if (OI_Drive_x==127 && OI_Drive_y==127){
//					OI_Drive_y = Bot_Creep_State_pwm_out;
//				}else {
//					if (Speed_Override){
//					} else {
//						OI_Drive_x = (((OI_Drive_x-127)*Bot_Driver_Scale_Factor)+127);
//						OI_Drive_y = (((OI_Drive_y-127)*Bot_Driver_Scale_Factor)+127);
//					}
//				}
//			}
//		}
//	}

//Determine speed
	if (encoderloop>=pollingtime){
		encoderloop=0;
		Encoder_Left_Speed=Tmp_Left_Speed/pollingtime;
		Encoder_Right_Speed=Tmp_Right_Speed/pollingtime;
		Tmp_Left_Speed=Tmp_Right_Speed=0;
	}else{
		Tmp_Left_Speed+=(signed int)(Encoder_Left_Count-Encoder_Left_Last);
		Tmp_Right_Speed+=(signed int)(Encoder_Right_Count-Encoder_Right_Last);
		Encoder_Left_Last=Encoder_Left_Count;
		Encoder_Right_Last=Encoder_Right_Count;
		encoderloop++;
	}


//speed shifting
//if speed >= Bot_Speed_To_Shift then set trans to high else it is low
//	if (Encoder_Right_Speed>=Max_Encoder_CPI_Low_Gear){ //Using Right side as it will be turning faster around corners
//		RC_Pneum_Trans_High = 1;
//		RC_Pneum_Trans_Low = 0;
//	} else {
//		RC_Pneum_Trans_High = 0;
//		RC_Pneum_Trans_Low = 1;
//	}

//Manual Shifting
	if (OI_Button_Trans){
		RC_Pneum_Trans_High = 1;
		RC_Pneum_Trans_Low = 0;
	} else {
		RC_Pneum_Trans_High = 0;
		RC_Pneum_Trans_Low = 1;
	}


//Make it go zoom!
	*left.motor=Limit_Mix(2000 + (int)OI_Drive_y + (int)OI_Drive_x - 127);
	*right.motor=Limit_Mix(2000 - (int)OI_Drive_y + (int)OI_Drive_x + 127); 


	/*if (abs((signed long)(Encoder_1_Count-turningzero1))<abs(turncount*700)){
		pwm02=127+(turncount<0 ? -1 : 1)*53; 
	}else{
		pwm02=127;
	}

	if (abs((signed long)(Encoder_2_Count-turningzero2))<abs(turncount*700)){
		pwm01=127+(turncount<0 ? 1 : -1)*53;
	}else{
		pwm01=127;
	}*/

	/*OI_Drive_y = Limit_Mix((int)OI_Drive_y-127+2000);
	right.hold_value=OI_Drive_y;
	pwm02 = Limit_Mix(2000+(int)pwm02+PID(right));*/

	//printf("1: %d\t2: %d\t3: %d\t4: %d\r\n5: %d\t6: %d\t7: %d\t8: %d\r\n", Button1, Button2, Button3, Button4, Button5, Button6, Button7, Button8);
	
/*	if (*left.value>*right.value){
		tmp2=1; 
		*left.motor--;
	}else if(*left.value<*right.value){
		tmp2=2;
		*
	}*/


}

	

unsigned char joy_condition(unsigned char in) {
	signed int out = ((signed int)in)-127;			
	if (out > joy_deadzone)	{
		if (out > joy_max){
			return (unsigned char)(joy_max+127);	
		}else{
 			return (unsigned char)(out+127);		
 		}
	}else if (out < -joy_deadzone){					// joystick is negative
		if (out < -joy_max){
			return (unsigned char)(-joy_max+127);	
		}else{
			return (unsigned char)(out+127);	
		}
	}
	return 127;
}



void debug(unsigned int v){
	unsigned char r = v&255;
	unsigned char l = (v>>8)&255;
	//stdout_serial_port = SERIAL_PORT_TWO;
	printf("%c%c",l,r);
	//stdout_serial_port = SERIAL_PORT_ONE;

}

void debug_long (unsigned long v){
	unsigned char lr = v&255;
	unsigned char ll = (v>>8)&255;
	unsigned char ur = (v>>16)&255;
	unsigned char ul = (v>>24)&255;
	//stdout_serial_port = SERIAL_PORT_TWO;
	printf("%c%c%c%c",ul,ur,ll,lr);
	//stdout_serial_port = SERIAL_PORT_ONE;
}


int PID(motor_structure pid){
	signed int error;
	signed int delta_err;
	signed int integral_err;
	signed int p_out;
	signed int i_out;
	signed int d_out;
	signed int output;

	error=*pid.value-pid.hold_value;
	delta_err=pid.prev_err-error;
	pid.integral_err+=error;

	pid.prev_err = error;

	if (pid.integral_err>MAX_INTEGRAL_ERROR){
		pid.integral_err = MAX_INTEGRAL_ERROR;
	}else if (pid.integral_err<-MAX_INTEGRAL_ERROR){
		pid.integral_err = -MAX_INTEGRAL_ERROR;
	}

    	if (error<5 && error>-5) {pid.integral_err=0;}

	p_out = (int)(error * pid.KP);
	i_out = (int)(pid.integral_err * pid.KI);
	d_out = (int)(delta_err * pid.KD);

	output = p_out + i_out + d_out;

	pid.prev_err = error;

	return output;	
}

long abs(long x){
	if (x<0){return -x;}else{return x;}
}

int absint(int x){
	if (x<0){return -x;}else{return x;}
}


void PID_Flop(motor_structure *mot, pid_structure pid){
	mot.KP = pid.KP;
	mot.KI = pid.KI;
	mot.KD = pid.KD;
}

