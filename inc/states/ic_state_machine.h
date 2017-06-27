/*
 * ic_state_machine.h
 *
 *  Created on: 26 wrz 2015
 *      Author: kn
 */

#ifndef IC_STATE_MACHINE_H_
#define IC_STATE_MACHINE_H_
#include "ic_state_module.h"

/** \brief Structure holding informations about the state of the machine.
 *
 *  \remark Don't use structure's members. Use below functions to get its state.
 */
typedef struct state_machine{
  /**< Last state whose function were executed. */
  state                  last_state;
  /**< The result of execution of the last state function. */
  state_exit_code        last_exit_code;
  /**< A state, whose function will be executed next. */
  state                  next_state;
}STATE_MACHINE;


/** Init or resets given state machine.
 *  \warning Do it before using any other function from this API
 */
void            init_state_machine       (STATE_MACHINE*);

/** Executes next state's function and updates the machine's state. */
void            process_next_state       (STATE_MACHINE*);

/** Returns last state from the machine structure. */
state           get_last_state           (const STATE_MACHINE *);

/** Returns the state whose function will be executed next. */
state           get_next_state           (const STATE_MACHINE *);

/** Returns the result of the execution of the last state's function. */
state_exit_code get_last_state_exit_code (const STATE_MACHINE *);


#endif /* IC_STATE_MACHINE_H_ */
