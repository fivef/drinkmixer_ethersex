/*
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 * For more information on the GPL, please go to:
 * http://www.gnu.org/copyleft/gpl.html
 */

/*
OCR1A and OCR2 pins are used for pwm!!!!

Check hardware's pinning *.m4 (e.g. netio.m4):
*/

#include <avr/pgmspace.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "config.h"
#include "hbridge.h"

//for comparator
#include <avr/io.h> 

//for experimental analog voltage of pressure sensor
//#include "../adc/adc.h"


//init data structures

//encoder
encoder encoder1;
encoder *encoder1_ptr = &encoder1;

//set point for pid control
int pid_set_point = 0;

//pid
pid_f_t pid_constants;
pid_f_t *pid_constants_ptr = &pid_constants;

//for acceleration control
int ramped_set_point = 0;
int acceleration_limiter_switch_off_threshold = ACCELERATION_LIMITED_SWITCH_OFF_THRESHOLD;
int acceleration = ACCELERATION;
int error = 0;

//for check stop threshold to stop the motor if close to setpoint
int position_tolerance = POSITION_TOLERANCE;


//set PWM values to STOP
uint8_t enable1_pwm=HBRIDGE_PWM_STOP;
uint8_t enable2_pwm=HBRIDGE_PWM_STOP;

//for left end switch debouncing
int left_end_switch_enabled = 1;


/****************************

INITIALIZATION FUNCTIONS

****************************/

/*! Initializes the H bridge module including pid controller, linear encoder and end witch */
void
init_hbridge(){

	HBRIDGEDEBUG ("Initializing HBridge module");

	//init pid
	pid_init_f(pid_constants_ptr, PID_PWM_MIN, PID_PWM_MAX); 
  	init_linear_encoder();
	init_end_switch();


	//OC1A/B Output Compare Pin: Pin for PWM signals. Has to be defined in pinning. (e.g. netio.m4)


	DDR_CONFIG_OUT(HBRIDGE_1_ENABLE); // set HBRIDGE_1_ENABLE Pin as defined in pinning (e.g. netio.m4) as output
	DDR_CONFIG_OUT(HBRIDGE_2_ENABLE);

	OCR1A=enable1_pwm;  //Output Compare Registers (OCR1A/B)
	OCR2=enable2_pwm;

	TC1_COUNTER_CURRENT=0x00FF; //set the timer counter
	TC2_COUNTER_CURRENT=0x00FF;

	/////////////////////////////////////////////////
	//Setup first timer
	/////////////////////////////////////////////////

	/*
	TCCR1A/B Timer0/Counter0 Control Registers

	Set OC1A/OC1B on compare match (Set
	output to high level)
	*/
	  TCCR1A|=_BV(COM1A1)|_BV(COM1A0);

	// Set PWM, Phase Correct, 8 bit (see Table 47 Atmega 32 Manual
	  TCCR1A|=_BV(WGM10); 
	  //TCCR1B|=_BV(WGM12); 

	// clockselect: (clkI/O)/1 (No prescaling)
	  TCCR1B|=_BV(CS10); 


	/////////////////////////////////////////////////
	//same for second timer (Timer/Counter2) (caution: different Registers!)
	/////////////////////////////////////////////////

	
	  TCCR2|=_BV(WGM20)|_BV(WGM21); // Set Fast PWM, 8 bit (see Table 52 Atmega 32 Manual)
	  TCCR2|=_BV(COM21)|_BV(COM20); // set OC2 on compare match (inverting mode) (see Table 52 Atmega 32 Manual)

	//TODO try non inverting mode: TCCR2|=_BV(COM21); // clear OC2 on compare match (non-inverting mode) (see Table 52 Atmega 32 Manual)

	// clockselect: (clkI/O)/1 (No prescaling)

		//TCCR2|=_BV(CS20); // no prescaling; //no noise but not working good
	  TCCR2|=_BV(CS20)|_BV(CS21)|_BV(CS22);  //clk/1024 working good but loud
																//60Hz on Fast PWM

	
		//TCCR2|=_BV(CS21)|_BV(CS22); //clk/256  working not good and very loud  3 122Hz
		//TCCR2|=_BV(CS20)|_BV(CS22);  //clk/128  movement quite good  2
		//TCCR2|=_BV(CS22);   //clk/64  high frequency noise but good driving  1
		//TCCR2|=_BV(CS21)|_BV(CS20);  //clk/32 high frequency very loud bad movement 4
		//TCCR2|=_BV(CS21); //clk/8  very high frequency very bad movement 5



	//move tray to 0 position
	//move_tray_to_init_position();
}

/*! Initializes the analog comparator for linear encoder reading */

void init_linear_encoder(){

	//init analog comparator for phase A input

	DDRA&=~(1<<HBRIDGE_ENCODER_PHASE_A_PIN);//as input
	PORTA&=~(1<<HBRIDGE_ENCODER_PHASE_A_PIN);//no Pull-up
		
	//SFIOR|=(1<<ACME);//enable multiplexer
	SFIOR&=~(1<<ACME);//disable multiplexer
	
	//ADCSRA&=~(1<<ADEN);//make sure ADC is OFF 
	ADCSRA|=(1<<ADEN);//make sure ADC is ON	

	ADMUX|=(0<<MUX2)|(1<<MUX1)|(0<<MUX0); //select ADC2 as negative AIN
	ACSR|=
	(0<<ACD)|	//Comparator ON
	(1<<ACBG)|	//Connect 1.23V reference to AIN0
	(1<<ACIE)|	//Comparator Interrupt enable
	(0<<ACIC)|	//input capture disabled
	(0<<ACIS1)| //set Comparator Interrupt on Output Toggle
	(0<<ACIS0);
	sei();//enable global interrupts

	//init phase B input

	DDRA&=~(1<<HBRIDGE_ENCODER_PHASE_B_PIN);//as input
	PORTA&=~(1<<HBRIDGE_ENCODER_PHASE_B_PIN);//no Pull-up

	encoder1_ptr->count = 0;
	
}

/*! Initializes the horizontal left end switch */
void init_end_switch(){

	DDRA&=~(1<<HBRIDGE_SWITCH_PIN); //as input
	PORTA|=(1<<HBRIDGE_SWITCH_PIN); //enable pull-up

}

/*! \details This function initializes the data in a PID structure.
 *
 */
void pid_init_f(pid_f_t * ptr /*! A pointer to the PID data structure */,
    float min /*! The manipulated variable's minimum value */,
    float max /*! The manipulated variable's maximum value */){
	memset(ptr, 0, sizeof(pid_f_t));
	ptr->min = min;
	ptr->max = max;

	ptr->e = 0.0;
	ptr->i = 0.0;

	ptr->kp = KP;
	ptr->ki = KI;
	ptr->kd = KD;

	
}

/****************************

PERIODICALLY CALLED FUNCTIONS

****************************/

/*! Main loop called every 20ms (see META at the bottom of the file) */
void main_loop(){

	//check if stop switch was pressed
	check_switch_state();


	//calculate real error to determine direction of movement
	error = pid_set_point - encoder1_ptr->count;

	//stop if in range of position_tolerance
	//commented out because not needed if the I component of the PID controler is set != 0.
/*
	if(abs(error) <= position_tolerance){ 

		stop_all();

		//reset integral of PID
		pid_constants_ptr->i = 0;


	}
*/

	//determine direction of movement
	//if the direction changes or the error is 0 reset the PIDs integral variable
	//to avoid noise and oscillation around the setpoint
	if(error > 0){ 


		//reset integral of PID if direction changed
		if(pid_constants_ptr->direction == HBRIDGE_ACTION_LEFT){
			pid_constants_ptr->i = 0;
		}


		pid_constants_ptr->direction = HBRIDGE_ACTION_RIGHT;

		

	}else if(error < 0){


		//reset integral of PID if direction changed
		if(pid_constants_ptr->direction == HBRIDGE_ACTION_RIGHT){
			pid_constants_ptr->i = 0;
		}


		pid_constants_ptr->direction = HBRIDGE_ACTION_LEFT;

		

	}else if(error == 0){

		//keep direction

		//reset integral
		pid_constants_ptr->i = 0;

	}

	//calculate ramped error to limit acceleration
	int ramped_error = pid_set_point - ramped_set_point;


	//limit acceleration
	if(abs(ramped_error) > acceleration_limiter_switch_off_threshold){

		
		if(pid_constants_ptr->direction == HBRIDGE_ACTION_RIGHT){
			ramped_set_point += acceleration;
			
		}else if (pid_constants_ptr->direction == HBRIDGE_ACTION_LEFT){

			ramped_set_point -= acceleration;
		}
	


	}else{//if the ramped set point is very close to the real setpoint 
			//closer than acceleration_limiter_switch_off_threshold the ramping is disabled

		ramped_set_point = pid_set_point;

		

	}

	enable2_pwm = pid_update_f((float)ramped_set_point, (float)encoder1_ptr->count, pid_constants_ptr); 


	//invert pwm from 0 = stop, 255 = max speed to 0 = max speed, 255 = stop
	enable2_pwm = HBRIDGE_PWM_STOP - enable2_pwm;

	hbridge(HBRIDGE_2_SELECT, pid_constants_ptr->direction);



}


/*! Determines direction and updates encoder count from the logic 
		levels of the encoder's A and B outputs. 
	/param e encoder data structure
	/param A logic level phase A
	/param B logic level phase B
*/

void encoder_update(encoder *e, int A, int B){
	
	if(A == 1){
		// Rising edge of A.
		if(B == 1){
			
			e->count ++;
			
		}
		else{
			
			e->count --;

		}
	}
	else{
		// Falling edge of A.
		if(B == 1){
	
			e->count --;
		
		}
		else{

			e->count ++;
		
		}
	}
}

/*! used for debug information and left_end_switch dead time
		called by timer every 50*20ms (see META at the bottom of the file) */
void check_periodic(){
	
	
	HBRIDGEDEBUG ("ramped sp: %d pwm: %d pos: %d error: %f \n", ramped_set_point, enable2_pwm, 
			 encoder1_ptr->count, pid_constants_ptr->e);


	//reenable left end switch which was disabled by activating the switch

	left_end_switch_enabled = 1;


	//experimental preassure control loop
	/*
	int voltage = adc_get_voltage_setref(5,4); //Ref Voltage, Analog Input PD4

	HBRIDGEDEBUG ("pressure: %f", voltage);
	*/


}


/*! ISR for ANA_COMP_vect created by encoder flanks */

ISR(ANA_COMP_vect) {

	//toggle led for each interrupt
	PIN_TOGGLE(HBRIDGE_DEBUG_LED);

	//count encoder pulses

	int A = bit_is_clear(ACSR, ACO);
	int B = PIN_HIGH(HBRIDGE_ENCODER_PHASE_B) == 2; //true is 2?

	//HBRIDGEDEBUG ("A: %d B: %d Count: %d\n",A,B,encoder1_ptr->count);


	// encoder pointer, phase A, phase B
	encoder_update(encoder1_ptr, A, B);

	//call check switch state here to avoid polling the switch
	check_switch_state();	
}


/*! Called by main_loop() to check if the left end switch was pressed and to stop motor */
void check_switch_state(){

	//if the stop switch is activated stop motor
	
	 if (PIN_HIGH(HBRIDGE_SWITCH)==0 && left_end_switch_enabled == 1){

			left_end_switch_enabled = 0;
			
		
			HBRIDGEDEBUG ("counted steps: %d \n",encoder1_ptr->count);
		
			stop_all();
			
			//reset encoder count
			ramped_set_point = pid_set_point = encoder1_ptr->count = 0;

			//drive 100 steps right for init position
			//set_set_point(100);


	}
}


/*! \details This function updates the value of the manipulated variable (MV)
 * based on the current state of the PID loop.
 */
float pid_update_f(float sp /*! The set point */,
    float pv /*! The process variable */,
    pid_f_t * ptr /*! A pointer to the PID constants */){
 
  float e;
  float manp;
  float tmpi;

  e = ptr->e;

  ptr->e = abs(sp - pv);

  tmpi = ptr->i + ptr->e;
  //bound the integral
  manp = ptr->kp * ptr->e + ptr->ki * tmpi + ptr->kd * (e - ptr->e);



  if ( (manp < ptr->max) && (manp > ptr->min) ){
    ptr->i = tmpi;


  } else if ( manp > ptr->max ){
    manp = ptr->max;
  } else if ( manp < ptr->min ){
    manp = ptr->min;
  }

  return manp;
}

/****************************

INTERFACE FUNCTIONS

****************************/

/*! Sets the desired poisition of the tray.
	\param set_point as integer encoder steps. (one step = 0,083 mm)
*/
void set_set_point(int set_point){

	 pid_set_point = set_point;
	 
}

/*! Sets PID's propotional value (default see KP)
	\param kp as float.
*/
void set_kp(float kp){
	pid_constants_ptr->kp = kp;

	HBRIDGEDEBUG ("setkp: %f\n", pid_constants_ptr->kp);
}

/*! Sets PID's integral value (default see KI)
	\param ki as float.
*/
void set_ki(float ki){
	pid_constants_ptr->ki = ki;

	HBRIDGEDEBUG ("setki: %f\n", pid_constants_ptr->ki);
}

/*! Sets acceleration for ramping (default see ACCELERATION)
	\param acc an integer acceleration.
*/
void set_acceleration(int acc){
	acceleration = acc;

	HBRIDGEDEBUG ("set acceleration: %f\n", pid_constants_ptr->kd);
}

/*! Moves tray to the initial position */
void move_tray_to_init_position(){

	//drive to left until the stop switch is hit
	set_set_point(-5000);
	

}

/****************************

HELPER FUNCTIONS

****************************/

/*! stops all motor movements */
void stop_all(){
	
	ramped_set_point = pid_set_point = encoder1_ptr->count;

}

void
hbridge_disable(uint8_t selection){

  if (selection==HBRIDGE_1_SELECT){
	
	OCR1A=HBRIDGE_PWM_STOP;
	
  } else {
  	
	OCR2=HBRIDGE_PWM_STOP; 

  }

}

/*!
	Enables the selection H-Bridge by setting the speed of enable*_pwm
*/
void
hbridge_enable(uint8_t selection){

  if (selection==HBRIDGE_1_SELECT){
    OCR1A=enable1_pwm;
  } else {
    OCR2=enable2_pwm; 
  }

}

void
hbridge(uint8_t selection, uint8_t action)
{
  if (selection==HBRIDGE_1_SELECT){

		  PIN_CLEAR(HBRIDGE_I1);

		  switch (action){
			case HBRIDGE_ACTION_BRAKE: 
				hbridge_disable(selection);
				break;
			case HBRIDGE_ACTION_RIGHT: 

			 	PIN_CLEAR(HBRIDGE_I1);

				hbridge_enable(selection);
				break;
			case HBRIDGE_ACTION_LEFT: 

			  	PIN_SET(HBRIDGE_I1);

				hbridge_enable(selection);
				break;
		  }

  }else{

	 	 PIN_CLEAR(HBRIDGE_I2);

		  switch (action){
			case HBRIDGE_ACTION_BRAKE: 
				hbridge_disable(selection);
				break;
			case HBRIDGE_ACTION_LEFT: 

			  	PIN_CLEAR(HBRIDGE_I2);
				hbridge_enable(selection);
				break;
			case HBRIDGE_ACTION_RIGHT: 

			  	PIN_SET(HBRIDGE_I2);
				hbridge_enable(selection);
				break;
		  }

  } 

}


/*
  -- Ethersex META --
  header(hardware/hbridge/hbridge.h)
  init(init_hbridge)
  timer(1,main_loop())
  timer(50,check_periodic())

*/

/*According to ethersex documentation timer(1,function()) means call function() every 1*20ms
	not sure if this is really the case TODO check 
	

*/


