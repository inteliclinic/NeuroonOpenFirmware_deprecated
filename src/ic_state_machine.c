/*
 * ic_state_machine.c
 *
 *  Created on: 26 wrz 2015
 *      Author: kn
 */

#include "ic_state_machine.h"
#include "ic_machine_program.h"


void init_state_machine(STATE_MACHINE* sm){
  sm->last_state     = 0;
  sm->next_state     = PROGRAM[INITIAL_SM][0];
  sm->last_exit_code = 0;
}


void process_next_state(STATE_MACHINE* sm){
  sm->last_state = sm->next_state;
  // -- executing state function
  sm->last_exit_code = MAPPING[sm->last_state]();
  // ---------------------------
  sm->next_state     = PROGRAM[sm->last_state][sm->last_exit_code];
}


state get_last_state(const STATE_MACHINE * sm){
  return sm->last_state;
}


state_exit_code get_last_state_exit_code(const STATE_MACHINE * sm){
  return sm->last_exit_code;
}


state get_next_state(const STATE_MACHINE * sm){
  return sm->next_state;
}
