/*
 * Copyright (c) 2010 by Stefan Riepenhausen <rhn@gmx.net>
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

Check hardwares pinning *.m4:
*/

#include <avr/pgmspace.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "config.h"
#include "hbridge.h"

//comparator
#include <avr/io.h> 


uint8_t hbridge_1_current_speed = HBRIDGE_PWM_STOP; //variable for incremental speed up 255 is stop, 0 full speed
uint8_t hbridge_2_current_speed = HBRIDGE_PWM_STOP; //variable for incremental speed up

//init data structures

//pid
pid_f_t pid_constants;
pid_f_t *pid_constants_ptr = &pid_constants;

//encoder
encoder encoder1;
encoder *encoder1_ptr = &encoder1;

//define set point for pid control
int pid_set_point = 0;

//for acceleration control
int ramped_set_point = 0;
float pwm = 0.0;
int acceleration_limiter_switch_off_threshold = 10;
int acceleration = 5;
int error = 0;

//for check stop threshold to stop the motor if close to setpoint
	int last_error = 0;
	int POSITION_TOLERANCE = 10; //if error is smaller than this value the motor stops
											 //trying to reach setpoint (1 = 0,083 mm)


uint8_t enable1_pwm=HBRIDGE_PWM_STOP;
uint8_t enable2_pwm=HBRIDGE_PWM_STOP;


/*
sets the speed (speed [0,255]) of the selected (selection) H-Bridge
*/
void
hbridge_pwm(uint8_t selection, uint8_t speed){
  if (selection==HBRIDGE_1_SELECT){
    enable1_pwm=speed;
    OCR1A=enable1_pwm;
  } else {
    enable2_pwm=speed;
    OCR2=enable2_pwm; 
  }
}


void
init_hbridge(){

	//init pid
	pid_init_f(pid_constants_ptr, 0.0, 150.0); //150 für 60Hz betrieb


	
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

	// Set PWM, Phase Correct, 8 bit (see. Table 47)
	  TCCR1A|=_BV(WGM10); 
	  //TCCR1B|=_BV(WGM12); 

	// clockselect: (clkI/O)/1 (No prescaling)
	  TCCR1B|=_BV(CS10); 


	/////////////////////////////////////////////////
	//same for second timer (caution: different Registers!)
	/////////////////////////////////////////////////

	// Set Fast PWM?, 8 bit (see. Table 47)
	  TCCR2|=_BV(WGM20)|_BV(WGM21);
	  TCCR2|=_BV(COM21)|_BV(COM20);

	// clockselect: (clkI/O)/1 (No prescaling)

		//TCCR2|=_BV(CS20); // no prescaling; //no noise but not working good
	  TCCR2|=_BV(CS20)|_BV(CS21)|_BV(CS22);  //clk/1024 working good but loud  1  30Hz on PHase Correct PWM
																//60Hz on Fast PWM

	
		//TCCR2|=_BV(CS21)|_BV(CS22); //clk/256  working not good and very loud  3 122Hz
		//TCCR2|=_BV(CS20)|_BV(CS22);  //clk/128  movement quite good  2
		//TCCR2|=_BV(CS22);   //clk/64  high frequency noise but good driving  1
		//TCCR2|=_BV(CS21)|_BV(CS20);  //clk/32 high frequency very loud bad movement 4
		//TCCR2|=_BV(CS21); //clk/8  very high frequency very bad movement 5
}

void
hbridge_disable(uint8_t selection){

  if (selection==HBRIDGE_1_SELECT){
  	OCR1A=HBRIDGE_PWM_STOP;
	hbridge_1_current_speed = HBRIDGE_PWM_STOP;
  } else {
  	OCR2=HBRIDGE_PWM_STOP; 
	hbridge_2_current_speed = HBRIDGE_PWM_STOP;
  }

}

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
				hbridge_disable(selection); // PIN_CLEAR(HBRIDGE_1_ENABLE);
				break;
			case HBRIDGE_ACTION_RIGHT: 

			 	PIN_CLEAR(HBRIDGE_I1);

				hbridge_enable(selection); // PIN_SET(HBRIDGE_1_ENABLE);
				break;
			case HBRIDGE_ACTION_LEFT: 

			  	PIN_SET(HBRIDGE_I1);

				hbridge_enable(selection); // PIN_SET(HBRIDGE_1_ENABLE);
				break;
		  }

  }else{

	 	 PIN_CLEAR(HBRIDGE_I2);

		  switch (action){
			case HBRIDGE_ACTION_BRAKE: 
				hbridge_disable(selection); // PIN_CLEAR(HBRIDGE_2_ENABLE);
				break;
			case HBRIDGE_ACTION_LEFT: 

			  	PIN_CLEAR(HBRIDGE_I2);
				hbridge_enable(selection); // PIN_SET(HBRIDGE_2_ENABLE);
				break;
			case HBRIDGE_ACTION_RIGHT: 

			  	PIN_SET(HBRIDGE_I2);
				hbridge_enable(selection); // PIN_SET(HBRIDGE_2_ENABLE);
				break;
		  }

  } 

}


/*inits the analog comparator for linear encoder reading*/

void init_linear_encoder(){

	//init phase A input

	DDRA&=~(1<<HBRIDGE_ENCODER_PHASE_A_PIN);//as input
	PORTA&=~(1<<HBRIDGE_ENCODER_PHASE_A_PIN);//no Pull-up
		
	SFIOR|=(1<<ACME);//enable multiplexer
	ADCSRA&=~(1<<ADEN);//make sure ADC is OFF 
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


	encoder1_ptr->direction = 0;
	encoder1_ptr->count = 0;
	encoder1_ptr->loop_count = 0;
}

void encoder_update(encoder *e, int A, int B){
	// Determine direction and update encoder count from the logic levels of the encoder's A and B outputs.
	if(A == 1){
		// Rising edge of A.
		if(B == 1){
			e->direction = HBRIDGE_ACTION_RIGHT;
			e->count ++;
			e->loop_count++;
		}
		else{
			e->direction = HBRIDGE_ACTION_LEFT;
			e->count --;
			e->loop_count--;


		}
	}
	else{
		// Falling edge of A.
		if(B == 1){
			e->direction = HBRIDGE_ACTION_LEFT;
			e->count --;
			e->loop_count--;
		}
		else{
			e->direction = HBRIDGE_ACTION_RIGHT;
			e->count ++;
			e->loop_count++;
		}
	}
}

void init_end_switch(){

	//switch
	//DDR_CONFIG_IN(HBRIDGE_SWITCH);

	//PIN_SET(HBRIDGE_SWITCH); 
	DDRA&=~(1<<HBRIDGE_SWITCH_PIN); //as input
	PORTA|=(1<<HBRIDGE_SWITCH_PIN); //enable pull-up

	//interrupt would be nice here but INT0 and INT1 are on Port D (EXT.) and on INT 2 is used for 		//	Networking
	//on atmega32 interrupts only on pins INT0, INT1 and INT2 possible!?!
	
	//so the analog comparator ISR is used to check if the switch is activated
}


// Interrupt handler for ANA_COMP_vect
//
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

void set_set_point(int set_point){

	 pid_set_point = set_point;
}

void set_kp(float kp){
	pid_constants_ptr->kp = kp;

	HBRIDGEDEBUG ("setkp: %f\n", pid_constants_ptr->kp);
}

void set_ki(float ki){
	pid_constants_ptr->ki = ki;

	HBRIDGEDEBUG ("setki: %f\n", pid_constants_ptr->ki);
}

void set_acceleration(float acc){
	acceleration = acc;

	HBRIDGEDEBUG ("set acceleration: %f\n", pid_constants_ptr->kd);
}

void check_switch_state(){

	

	//if the stop switch is activated stop motor
	
	 if (PIN_HIGH(HBRIDGE_SWITCH)==0){
		
			HBRIDGEDEBUG ("counted steps: %d",encoder1_ptr->count);
		
			stop_all();
			
			ramped_set_point = pid_set_point = encoder1_ptr->count = 0;

			hbridge_disable(HBRIDGE_1_SELECT);
			hbridge_disable(HBRIDGE_2_SELECT); //TODO: adapt for second motor


			HBRIDGEDEBUG ("motor stopped");

	}
}

void main_loop(){

	//check if stop switch was pressed
	check_switch_state();

	
	//calculate real error to determine direction of movement
	error = pid_set_point - encoder1_ptr->count;

	//solved problem if carriage is moved manually the motor starts full speed in this direction
	

		//determine direction of movement
	if(error > 0){   //TODO: Doku > oder >= aufeinmal vollgas in die andere richtung???
		pid_constants_ptr->direction = HBRIDGE_ACTION_RIGHT;

	}else if(error < 0){

		pid_constants_ptr->direction = HBRIDGE_ACTION_LEFT;

	}else if(error == 0){

		//keep direction

	}

	//calculate ramped error to limit acceleration
	error = pid_set_point - ramped_set_point;


	//limit acceleration
	if(abs(error) > acceleration_limiter_switch_off_threshold){

		
		if(pid_constants_ptr->direction == HBRIDGE_ACTION_RIGHT){
			ramped_set_point += acceleration;
		}else if (pid_constants_ptr->direction == HBRIDGE_ACTION_LEFT){

			ramped_set_point -= acceleration;
		}


	}else{//if the ramped set point is very close to the real setpoint 
			//closer than acceleration_limiter_switch_off_threshold the ramping is disabled

		ramped_set_point = pid_set_point;

		

	}

	pwm = pid_update_f((float)ramped_set_point, (float)encoder1_ptr->count, pid_constants_ptr); 

	pwm = 255-pwm;



	hbridge(HBRIDGE_2_SELECT, pid_constants_ptr->direction);


	hbridge_pwm(HBRIDGE_2_SELECT, pwm);


}

//stops all motor movements
void stop_all(){
	
	ramped_set_point = pid_set_point = encoder1_ptr->count;

}

void check_stop_threshold(){
	
	
	HBRIDGEDEBUG ("ramped sp: %d pwm: %f %d pos: %d error: %f \n", ramped_set_point, pwm, 
			pid_constants_ptr->direction, encoder1_ptr->count, pid_constants_ptr->e);


		//stop if in range of POSITION_TOLERANCE
	if(abs(pid_constants_ptr->e) < POSITION_TOLERANCE){ // && last_error < POSITION_TOLERANCE

		stop_all();

		HBRIDGEDEBUG ("Stopped because POSITION_TOLERANCE was reached \n");


	}

	//last_error = abs(pid_constants_ptr->e);


}

void move_tray_to_init_position(){

	//drive to left until the stop switch is hit
	set_set_point(-5000);
	

}

void move_tray_test(){



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
	//ptr->kp = 0.4f;
	//ptr->ki = 0.009f;
	//ptr->kd = 0.032f;

	ptr->kp = 1.5;
	ptr->ki = 0.0;
	ptr->kd = 0.0;

	ptr->acceleration = 0.1;
	
}
 
/*! \details This function updates the value of the manipulated variable (MV)
 * based on the current state of the PID loop. Sets the direction of movement in the pid_constants struct
 */
float pid_update_f(float sp /*! The set point */,
    float pv /*! The process variable */,
    pid_f_t * ptr /*! A pointer to the PID constants */){
  float temp;
  float e;
  float p;
  float manp;
  float tmpi;


  e = ptr->e;

  ptr->e = abs(sp - pv);



	


  tmpi = ptr->i + ptr->e;
  //bound the integral
  manp = ptr->kp * ptr->e + ptr->ki * tmpi + ptr->kd * (e - ptr->e);

//lower acceleration
 // manp = ptr->last_speed + ptr->acceleration*manp;

  if ( (manp < ptr->max) && (manp > ptr->min) ){
    ptr->i = tmpi;


  } else if ( manp > ptr->max ){
    manp = ptr->max;
  } else if ( manp < ptr->min ){
    manp = ptr->min;
  }

 

  return manp;
}



/*
//move steps_in_cm cm to right
void move(uint8_t selection, uint8_t direction){

	int real_steps_to_make = steps_in_cm * STEPS_PER_CM;


}
*/


/*
  -- Ethersex META --
  header(hardware/hbridge/hbridge.h)
  init(init_hbridge)
  timer(1,main_loop())
  timer(50,check_stop_threshold())
  
*/
//timer(1,main_loop())
//timer(5,main_loop())
//TODO: adapt timer value (value x 40ms ?)
//put this into meta for periodic call of function timer(1, joystick_digital_periodic())

