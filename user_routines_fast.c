/*******************************************************************************
* FILE NAME: user_routines_fast.c <FRC VERSION>
*
* DESCRIPTION:
*  This file is where the user can add their custom code within the framework
*  of the routines below. 
*
* USAGE:
*  You can either modify this file to fit your needs, or remove it from your 
*  project and replace it with a modified copy. 
*
* OPTIONS:  Interrupts are disabled and not used by default.
*
*******************************************************************************/

#include "ifi_aliases.h"
#include "ifi_default.h"
#include "ifi_utilities.h"
#include "user_routines.h"
//#include "user_Serialdrv.h"
#include "serial_ports.h"
#include "adc.h"
#include "gyro.h"
#include "encoder.h"

/*** DEFINE USER VARIABLES AND INITIALIZE THEM HERE ***/


/*******************************************************************************
* FUNCTION NAME: InterruptVectorLow
* PURPOSE:       Low priority interrupt vector
* CALLED FROM:   nowhere by default
* ARGUMENTS:     none
* RETURNS:       void
* DO NOT MODIFY OR DELETE THIS FUNCTION 
*******************************************************************************/
#pragma code InterruptVectorLow = LOW_INT_VECTOR
void InterruptVectorLow (void)
{
  _asm
    goto InterruptHandlerLow  /*jump to interrupt routine*/
  _endasm
}


/*******************************************************************************
* FUNCTION NAME: InterruptHandlerLow
* PURPOSE:       Low priority interrupt handler
* If you want to use these external low priority interrupts or any of the
* peripheral interrupts then you must enable them in your initialization
* routine.  Innovation First, Inc. will not provide support for using these
* interrupts, so be careful.  There is great potential for glitchy code if good
* interrupt programming practices are not followed.  Especially read p. 28 of
* the "MPLAB(R) C18 C Compiler User's Guide" for information on context saving.
* CALLED FROM:   this file, InterruptVectorLow routine
* ARGUMENTS:     none
* RETURNS:       void
*******************************************************************************/
#pragma code
#pragma interruptlow InterruptHandlerLow save=PROD,section(".tmpdata")

void InterruptHandlerLow ()     
{                               
	unsigned char Port_B;
	unsigned char Port_B_Delta;

	if (PIR1bits.RC1IF && PIE1bits.RC1IE) // rx1 interrupt?
	{
		#ifdef ENABLE_SERIAL_PORT_ONE_RX
		Rx_1_Int_Handler(); // call the rx1 interrupt handler (in serial_ports.c)
		#endif
	}                              
	else if (PIR3bits.RC2IF && PIE3bits.RC2IE) // rx2 interrupt?
	{
		#ifdef ENABLE_SERIAL_PORT_TWO_RX
		Rx_2_Int_Handler(); // call the rx2 interrupt handler (in serial_ports.c)
		#endif
	} 
	else if (PIR1bits.TX1IF && PIE1bits.TX1IE) // tx1 interrupt?
	{
		#ifdef ENABLE_SERIAL_PORT_ONE_TX
		Tx_1_Int_Handler(); // call the tx1 interrupt handler (in serial_ports.c)
		#endif
	}                              
	else if (PIR3bits.TX2IF && PIE3bits.TX2IE) // tx2 interrupt?
	{
		#ifdef ENABLE_SERIAL_PORT_TWO_TX
		Tx_2_Int_Handler(); // call the tx2 interrupt handler (in serial_ports.c)
		#endif
	}
	else if (INTCON3bits.INT2IF && INTCON3bits.INT2IE) // encoder 1 interrupt?
	{ 
		INTCON3bits.INT2IF = 0; // clear the interrupt flag
		#ifdef ENABLE_ENCODER_1
		Encoder_1_Int_Handler(); // call the left encoder interrupt handler (in encoder.c)
		#endif
	}
	else if (INTCON3bits.INT3IF && INTCON3bits.INT3IE) // encoder 2 interrupt?
	{
		INTCON3bits.INT3IF = 0; // clear the interrupt flag
		#ifdef ENABLE_ENCODER_2
		Encoder_2_Int_Handler(); // call right encoder interrupt handler (in encoder.c)
		#endif
	}
	else if (INTCONbits.RBIF && INTCONbits.RBIE) // encoder 3-6 interrupt?
	{
		Port_B = PORTB; // remove the "mismatch condition" by reading port b            
		INTCONbits.RBIF = 0; // clear the interrupt flag
		Port_B_Delta = Port_B ^ Old_Port_B; // determine which bits have changed
		Old_Port_B = Port_B; // save a copy of port b for next time around
	 
		if(Port_B_Delta & 0x10) // did external interrupt 3 change state?
		{
			#ifdef ENABLE_ENCODER_3
			Encoder_3_Int_Handler(Port_B & 0x10 ? 1 : 0); // call the encoder 3 interrupt handler (in encoder.c)
			#endif
		}
		if(Port_B_Delta & 0x20) // did external interrupt 4 change state?
		{
			#ifdef ENABLE_ENCODER_4
			Encoder_4_Int_Handler(Port_B & 0x20 ? 1 : 0); // call the encoder 4 interrupt handler (in encoder.c)
			#endif
		}
		if(Port_B_Delta & 0x40) // did external interrupt 5 change state?
		{
			#ifdef ENABLE_ENCODER_5
			Encoder_5_Int_Handler(Port_B & 0x40 ? 1 : 0); // call the encoder 5 interrupt handler (in encoder.c)
			#endif
		}
		if(Port_B_Delta & 0x80) // did external interrupt 6 change state?
		{
			#ifdef ENABLE_ENCODER_6
			Encoder_6_Int_Handler(Port_B & 0x80 ? 1 : 0); // call the encoder 6 interrupt handler (in encoder.c)
			#endif
		}
	}
	else if(PIR1bits.TMR2IF && PIE1bits.TMR2IE) // timer 2 interrupt?
	{
		PIR1bits.TMR2IF = 0; // clear the timer 2 interrupt flag [92]
		Timer_2_Int_Handler(); // call the timer 2 interrupt handler (in adc.c)
	}                     
	else if(PIR1bits.ADIF && PIE1bits.ADIE) // ADC interrupt
	{
		PIR1bits.ADIF = 0; // clear the ADC interrupt flag
		ADC_Int_Handler(); // call the ADC interrupt handler (in adc.c)
	}       
	
	


	
}


/*******************************************************************************
* FUNCTION NAME: User_Autonomous_Code
* PURPOSE:       Execute user's code during autonomous robot operation.
* You should modify this routine by adding code which you wish to run in
* autonomous mode.  It will be executed every program loop, and not
* wait for or use any data from the Operator Interface.
* CALLED FROM:   main.c file, main() routine when in Autonomous mode
* ARGUMENTS:     none
* RETURNS:       void
*******************************************************************************/
void User_Autonomous_Code(void)
{
	static unsigned int Hybrid_Starting_Position=0;
	static char Straight_Long_1=1;
	static char Straight_Long_2=0;
	static char Straight_Long_3=0;
	static char Straight_Short_1=0;
	static char Straight_Short_2=0;
	static char Turn_90_1=0;
	static char Turn_90_2=0;
	static char Turn_90_3=0;
	static char Turn_90_4=0;
	static char Stop_0=0;
	static char Stop_1=0;
	static char Stop_2=0;
	static char Stop_3=0;
	static char Stop_4=0;
	static int Loop=0;
  /* Initialize all PWMs and Relays when entering Autonomous mode, or else it
     will be stuck with the last values mapped from the joysticks.  Remember, 
     even when Disabled it is reading inputs from the Operator Interface. 
  */
  pwm01 = pwm02 = pwm03 = pwm04 = pwm05 = pwm06 = pwm07 = pwm08 = 127;
  pwm09 = pwm10 = pwm11 = pwm12 = pwm13 = pwm14 = pwm15 = pwm16 = 127;
  relay1_fwd = relay1_rev = relay2_fwd = relay2_rev = 0;
  relay3_fwd = relay3_rev = relay4_fwd = relay4_rev = 0;
  relay5_fwd = relay5_rev = relay6_fwd = relay6_rev = 0;
  relay7_fwd = relay7_rev = relay8_fwd = relay8_rev = 0;

  Hybrid_Starting_Position=RC_Hybrid_Starting_Position_2;
  Hybrid_Starting_Position=(Hybrid_Starting_Position << 1) | RC_Hybrid_Starting_Position_1; 
  RC_Pneum_Trans_High = 0;
  RC_Pneum_Trans_Low = 1;

  while (autonomous_mode)   /* DO NOT CHANGE! */
  {
    if (statusflag.NEW_SPI_DATA)      /* 26.2ms loop area */
    {
        Getdata(&rxdata);   /* DO NOT DELETE, or you will be stuck here forever! */
        /* Add your own autonomous code here. */
        Generate_Pwms(pwm13,pwm14,pwm15,pwm16);
//	if (RC_Hybrid_Move){
//	if (Stop_0){
//			if (Loop<=80){
//				Auto_Out (127,127);
//				Loop++;
//			} else {
//				Reset_Encoder_1_Count();
//				Reset_Encoder_2_Count();
//				Loop=0;
//				Stop_0 = 0;
//				Straight_Long_1 = 1;
//			}
//		}

	if (Straight_Long_1){
			if (Encoder_Right_Count<Bot_Straight_Long){
				Auto_Out (60,187);
			} else {
				Reset_Encoder_1_Count();
				Reset_Encoder_2_Count();
				Straight_Long_1 = 0;
				Stop_1 = 1;
			}
		}
		if (Stop_1){
			if (Loop<=25){
				Auto_Out (117,135);
				Loop++;
			} else {
				Reset_Encoder_1_Count();
				Reset_Encoder_2_Count();
				Loop=0;
				Stop_1 = 0;
				Turn_90_1 = 1;
			}
		}
		if (Turn_90_1){
			if (Encoder_Right_Count<Bot_Turn_90){
				Auto_Out (127,255);
			} else {
				Reset_Encoder_1_Count();
				Reset_Encoder_2_Count();
				Turn_90_1 = 0;
				Straight_Short_1 = 1;
			}
		}
		if (Straight_Short_1){
			if (Encoder_Right_Count<Bot_Straight_Short){
				Auto_Out (60,187);
			} else {
				Reset_Encoder_1_Count();
				Reset_Encoder_2_Count();
				Stop_2 = 1;
				Straight_Short_1 = 0;
			}
		}
		if (Stop_2){
			if (Loop<=25){
				Auto_Out (117,135);
				Loop++;
			} else {
				Reset_Encoder_1_Count();
				Reset_Encoder_2_Count();
				Loop=0;
				Stop_2 = 0;
				Turn_90_2 = 1;
			}
		}
		if (Turn_90_2){
			if (Encoder_Right_Count<Bot_Turn_90){
				Auto_Out (70,187);
			} else {
				Reset_Encoder_1_Count();
				Reset_Encoder_2_Count();
				Turn_90_2 = 0;
				Straight_Long_2 = 1;
			}
		}
		if (Straight_Long_2){
			if (Encoder_Right_Count<Bot_Straight_Long){
				Auto_Out (60,187);
			} else {
				Reset_Encoder_1_Count();
				Reset_Encoder_2_Count();
				Straight_Long_2 = 0;
				Stop_3 = 1;
			}
		}
/*		if (Stop_3){
			if (Loop<=20){
				Auto_Out (127,127);
				Loop++;
			} else {
				Reset_Encoder_1_Count();
				Reset_Encoder_2_Count();
				Loop=0;
				Stop_3 = 0;
				Turn_90_3 = 1;
			}
		}
		if (Turn_90_3){
			if (Encoder_Right_Count<Bot_Turn_90){
				Auto_Out (127,187);
			} else {
				Reset_Encoder_1_Count();
				Reset_Encoder_2_Count();
				Turn_90_3 = 0;
				Straight_Short_2 = 1;
			}
		}
		if (Straight_Short_2){
			if (Encoder_Right_Count<Bot_Straight_Short){
				Auto_Out (60,187);
			} else {
				Reset_Encoder_1_Count();
				Reset_Encoder_2_Count();
				Stop_4 = 1;
				Straight_Short_2 = 0;
			}
		}
		if (Stop_4){
			if (Loop<=20){
				Auto_Out (127,127);
				Loop++;
			} else {
				Reset_Encoder_1_Count();
				Reset_Encoder_2_Count();
				Loop=0;
				Stop_4 = 0;
				Turn_90_4 = 1;
			}
		}

		if (Turn_90_4){
			if (Encoder_Right_Count<Bot_Turn_90){
				Auto_Out (127,187);
			} else {
				Reset_Encoder_1_Count();
				Reset_Encoder_2_Count();
				Turn_90_4 = 0;
				Straight_Long_3 = 1;
			}
		}
		if (Straight_Long_3){
			if (Encoder_Right_Count<Bot_Straight_Long){
				Auto_Out (60,187);
			} else {
				Reset_Encoder_1_Count();
				Reset_Encoder_2_Count();
				Straight_Long_3 = 0;
			}
		}
*/
//	}

//	if (RC_Hybrid_Move){//		if (RC_Hybrid_Poke_1){//		}//		if (RC_Hybrid_Poke_2){//		}
//		RC_Hybrid_Dflt_Poke_Position 0=Inside; 1=Middle//		Hybrid_Starting_Position 1=Inside ; 2=Middle ; 3=Outside; 0=Assume No Autonomous...if Hybrod_Move...will only drive straight	}
//Process commands from IR Board//Button 1	Overpass 1 - 1st open position from inside//Button 2	Overpass 1 - 1st open position from outside//Button 3	Overpass 2 - 1st open position from inside//Button 4	Overpass 2 - 1st open position from outside//	if (RC_IRBoard_CMD1){//	}//	if (RC_IRBoard_CMD2){//	}//	if (RC_IRBoard_CMD3){//	}//	if (RC_IRBoard_CMD4){//	}


	LabView_Out();		// Write Output to LabView or Data loggr

        Putdata(&txdata);   /* DO NOT DELETE, or you will get no PWM outputs! */
    }
  }
}


void Auto_Out(unsigned char left, unsigned char right){
	RC_Motor_Drive_Left=left;
	RC_Motor_Drive_Right=right;
}	


/*******************************************************************************
* FUNCTION NAME: Process_Data_From_Local_IO
* PURPOSE:       Execute user's realtime code.
* You should modify this routine by adding code which you wish to run fast.
* It will be executed every program loop, and not wait for fresh data 
* from the Operator Interface.
* CALLED FROM:   main.c
* ARGUMENTS:     none
* RETURNS:       void
*******************************************************************************/
void Process_Data_From_Local_IO(void)
{
  /* Add code here that you want to be executed every program loop. */
  if(Get_ADC_Result_Count())
  {
    Process_Gyro_Data();
	
    Reset_ADC_Result_Count();
  }	

//Turn Compressor on
  RC_Pneum_Compressor_Start=1;


}

/*******************************************************************************
* FUNCTION NAME: Serial_Char_Callback
* PURPOSE:       Interrupt handler for the TTL_PORT.
* CALLED FROM:   user_SerialDrv.c
* ARGUMENTS:     
*     Argument             Type    IO   Description
*     --------             ----    --   -----------
*     data        unsigned char    I    Data received from the TTL_PORT
* RETURNS:       void
*******************************************************************************/

void Serial_Char_Callback(unsigned char data)
{
  /* Add code to handle incomming data (remember, interrupts are still active) */
}


/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
