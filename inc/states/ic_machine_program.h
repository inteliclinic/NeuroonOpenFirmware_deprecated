/*
 * ic_machine_program.h
 *
 *  Created on: 26 wrz 2015
 *      Author: kn
 */

#ifndef IC_MACHINE_PROGRAM_H_
#define IC_MACHINE_PROGRAM_H_

#include "ic_state_machine.h"
#include "ic_state_charging.h"
#include "ic_state_standby.h"
#include "ic_state_test_tool.h"
#include "ic_state_remote_controlled.h"



// --------------------------- MACHINE PROGRAM ----------------------------
#define STATES_COUNT    12
#define MAX_EXITS_COUNT 6

typedef enum{
    INITIAL_SM = 0x00,//
    REMOTE_CONTROLL_SM,
    CHARGING_SM,//
    TEST_TOOL_SM,
    STANDBY_SM//

}state_module;

const state_function MAPPING[STATES_COUNT] = {
  0,
  state_rc,
  state_charging,
  state_test_tool,
  state_standby
};

const state PROGRAM[STATES_COUNT][MAX_EXITS_COUNT] =
// PROGRAM
{
  //  INITIAL
  {   STANDBY_SM  },

  // -- REMOTE_CONTROLL
  //  BUTTON_PRESSED, CHARGER_PLUGGED,  TIMEOUT
  {   STANDBY_SM,     CHARGING_SM,      TEST_TOOL_SM  },

  // -- CHARGING
  //  CHARGER_UNPLUGGED
  {   STANDBY_SM  },

  //--TEST TOOL
  //  END_MESSAGE
  {   REMOTE_CONTROLL_SM  },

  // --STANDBY
  //  BUTTON_PRESSED,     CHARGER_PLUGGED,
  {   REMOTE_CONTROLL_SM, CHARGING_SM }

};

#endif /* IC_MACHINE_PROGRAM_H_ */
