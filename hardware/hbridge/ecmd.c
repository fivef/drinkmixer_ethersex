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
  int acc = atoi(cmd);

	set_acceleration(acc);

  
  return ECMD_FINAL_OK;
}


/*
  -- Ethersex META --
	block([[H-Bridge]])
	header(hardware/hbridge/hbridge.h)
	ecmd_feature(hbridge_setpoint_command, "hbridge setpoint", int, sets the setpoint)
	ecmd_feature(hbridge_kp_command, "hbridge kp", float, PIDs P component)
	ecmd_feature(hbridge_ki_command, "hbridge ki", float, PIDs I component)
	ecmd_feature(hbridge_acc_command, "hbridge acc", float, Acceleration)
*/
