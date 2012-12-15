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

#include <avr/io.h>
#include <avr/pgmspace.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "config.h"
#include "hbridge.h"
#include "protocols/ecmd/ecmd-base.h"


int16_t parse_cmd_hbridge_setpoint_command(char *cmd, char *output, uint16_t len) 
{
  int setpoint = atoi(cmd);

HBRIDGEDEBUG ("setpoint: %i\n", setpoint);

	set_set_point(setpoint);

  
  return ECMD_FINAL_OK;
}

int16_t parse_cmd_hbridge_kp_command(char *cmd, char *output, uint16_t len) 
{
  float kp = (float) atof(cmd);

	set_kp(kp);

  return ECMD_FINAL_OK;
}

int16_t parse_cmd_hbridge_ki_command(char *cmd, char *output, uint16_t len) 
{
  float ki = (float) atof(cmd);

	set_ki(ki);

  return ECMD_FINAL_OK;
}

int16_t parse_cmd_hbridge_acc_command(char *cmd, char *output, uint16_t len) 
{
  float acc = (float) atof(cmd);

	set_acceleration(acc);

  
  return ECMD_FINAL_OK;
}


#ifdef HBRIDGE_SUPPORT
int16_t parse_cmd_hbridge_command(char *cmd, char *output, uint16_t len) 
{

	uint8_t h_bridge_selection = 0;
  	uint8_t h_bridge_direction = 0; //0 = left, 1 = right
	uint8_t h_bridge_amount = 0;

	if(cmd[0] == NULL || cmd[1] == NULL ){
		return ECMD_ERR_PARSE_ERROR;
	}

	//bridge selection

	switch (cmd[0]){

		case '1':
			h_bridge_selection = HBRIDGE_1_SELECT;

		break;

		case '2':
			h_bridge_selection = HBRIDGE_2_SELECT;
		break;


		default:
			return ECMD_ERR_PARSE_ERROR;

	}

	//direction

	switch (cmd[1]){

		case 'l':
			h_bridge_direction = HBRIDGE_ACTION_LEFT;

		break;

		case 'r':
			h_bridge_direction = HBRIDGE_ACTION_RIGHT;
		break;


		case 'i':
			
			move_tray_to_init_position();
		return ECMD_FINAL_OK;
		

		case 't':
			
			move_tray_test();
		return ECMD_FINAL_OK;
		


		default:
			return ECMD_ERR_PARSE_ERROR;

	}

	//amount

	h_bridge_amount = atoi(cmd[2]);


	hbridge(h_bridge_selection,h_bridge_direction);	

	HBRIDGEDEBUG ("received command: hbridge: %i %i %i \n", h_bridge_selection, h_bridge_direction, h_bridge_amount);

  return ECMD_FINAL_OK;
}
#endif /* HBRIDGE_SUPPORT */



/*
  -- Ethersex META --
  block([[H-Bridge]])
  header(hardware/hbridge/hbridge.h)
  ecmd_feature(hbridge_setpoint_command, "hbridge setpoint", int, Set H-Bridge enable line valueeg. speed)
  ecmd_feature(hbridge_command, "hbridge direction ", [h_bridge] [direction] [amount], h_bridge_selection . direction . amount)
	ecmd_feature(hbridge_kp_command, "hbridge kp", float, variable . amount)
	ecmd_feature(hbridge_ki_command, "hbridge ki", float, variable . amount)
	ecmd_feature(hbridge_acc_command, "hbridge acc", float, variable . amount)
*/
