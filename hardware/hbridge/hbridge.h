/*	
 * Copyright (c) 2009 by Stefan Riepenhausen <rhn@gmx.net>
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

#ifndef HAVE_HBRIDGE_H
#define HAVE_HBRIDGE_H

/*!if the ramped set point is very close to the real setpoint.
If closer than ACCELERATION_LIMITED_SWITCH_OFF_THRESHOLD the ramping is disabled 
one step = 0,083 mm */
#define ACCELERATION_LIMITED_SWITCH_OFF_THRESHOLD 1

/*!Default acceleration for the ramping curve*/
#define ACCELERATION 1

/*!If the error is smaller than this value the motor stops to avoid noise.
one step = 0,083 mm */
#define POSITION_TOLERANCE 0

/*!pwm values for pid control*/
#define PID_PWM_MIN 0.0
#define PID_PWM_MAX 150.0 //PID_PWM_MAX = 150 good for 60Hz PWM

/*!pid initial values*/
#define KP 0.6
#define KI 0.0007
#define KD 0


/*!pwm defines for hbridge*/
#define HBRIDGE_PWM_STOP 0xFF
#define HBRIDGE_PWM_MAX 0x00

enum {
	HBRIDGE_1_SELECT,
	HBRIDGE_2_SELECT,

// *** SINGLE ENGINE MOVES
	HBRIDGE_ACTION_FREE,
	HBRIDGE_ACTION_BRAKE,
	HBRIDGE_ACTION_RIGHT,
	HBRIDGE_ACTION_LEFT


};

/*!PID datastructure*/
typedef struct{
  float max /*!< Max manipulated value */;
  float min /*!< Miniumum manipulated value */;
  float e /*!< Error value */;
  float i /*!< Integrator value */;
  float kp /*!< Proportional constant */;
  float ki /*!< Integrator constant */;
  float kd /*!< Differential constant */;
  int direction; /*!< Direction of movement */
} pid_f_t;

/*!Encoder datastructure*/
typedef struct{

	int count; /*!< Inpulese Counter*/

} encoder;



void
move_tray_to_init_position();

void 
set_set_point(int set_point);

void 
set_kp(float kp);

void 
set_ki(float ki);

void 
set_acceleration(int acc);


void
init_hbridge();

void
init_linear_encoder();

void
init_end_switch();

void pid_init_f(pid_f_t * ptr /*! A pointer to the PID data structure */,
    float min /*! The manipulated variable's minimum value */,
    float max /*! The manipulated variable's maximum value */);

void main_loop();

void
check_switch_state();

void
hbridge(uint8_t selection, uint8_t action);

void
hbridge_pwm(uint8_t selection, uint8_t speed);

void 
check_periodic();

void 
stop_all();

float 
pid_update_f(float sp /*! The set point */,
    float pv /*! The process variable */,
    pid_f_t * ptr /*! A pointer to the PID constants */);





#include "config.h"

#ifdef DEBUG_HBRIDGE
# include "core/debug.h"
# define HBRIDGEDEBUG(a...)  debug_printf("h-bridge: " a)
#else
# define HBRIDGEDEBUG(a...)
#endif

#endif  /* HAVE_HBRIDGE_H */
