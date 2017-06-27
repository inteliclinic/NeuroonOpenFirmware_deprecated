/*
 * ic_state_charging.h
 *
 *  Created on: 26 wrz 2015
 *      Author: kn
 */

#ifndef IC_STATE_CHARGING_H_
#define IC_STATE_CHARGING_H_

#include "ic_state_module.h"

typedef enum{
  CHARGING_SM_CHARGER_UNPLUGGED
} CHARGING_SM_EXIT_CODE;

state_exit_code state_charging();



#endif /* IC_STATE_CHARGING_H_ */
