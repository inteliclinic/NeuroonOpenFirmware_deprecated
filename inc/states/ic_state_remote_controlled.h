/**
 * @file    ic_state_remote_controlled.h
 * @author  Paweł Kaźmierzewski <p.kazmierzewski@inteliclinic.com>
 * @author  Wojciech Weclewski <w.weclewski@inteliclinic.com>
 * @date    July, 2016
 * @brief   State for Neuroon control.
 *
 *  Neuroon is in this state right after pairing.
 *
 */

#ifndef IC_STATE_REMOTE_CONTROLLED_H
#define IC_STATE_REMOTE_CONTROLLED_H

#include "ic_state_module.h"

typedef enum{
	RC_SM_BUTTON_PRESSED,
	RC_SM_CHARGER_PLUGGED,
        RC_SM_MESSAGE_TESTTOOL
} RC_SM_EXIT_CODE;

state_exit_code state_rc();
#endif /* IC_STATE_REMOTE_CONTROLLED_H */
