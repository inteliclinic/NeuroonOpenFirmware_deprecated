/*
 * ic_state_standby.h
 *
 *  Created on: 15 paź 2015
 *      Author: kn
 */

#ifndef IC_STATE_STANDBY_H_
#define IC_STATE_STANDBY_H_

#include "ic_state_module.h"

typedef enum{
	STANDBY_SM_BUTTON_PRESSED,
	STANDBY_SM_CHARGER_PLUGGED
} STANDBY_SM_EXIT_CODE;

state_exit_code state_standby();

#endif /* IC_STATE_STANDBY_H_ */
