/*
 * ic_state_module.h
 *
 *  Created on: 26 wrz 2015
 *      Author: kn
 */

#ifndef IC_STATE_MODULE_H_
#define IC_STATE_MODULE_H_

/** State identifier */
typedef int state;

/** State's function must exit with state_exit_code */
typedef int state_exit_code;

/** Every state module must implement one state_function */
typedef state_exit_code (*state_function)();

#endif /* IC_STATE_MODULE_H_ */
