/*
 * ic_state_test_tool.h
 *
 *  Created on: 15 paź 2015
 *      Author: kn
 */

#ifndef IC_STATE_TEST_TOOL_H_
#define IC_STATE_TEST_TOOL_H_

#include "ic_state_module.h"

#include "ic_state_test.h"

typedef enum{
	TEST_TOOL_SM_END_TEST_MESSAGE
} TEST_TOOL_SM_EXIT_CODE;

state_exit_code state_test_tool();

state_flag interpret_command_ble(uint8_t cmd);
void execute_command(state_flag current_flag);

void state_test_tool_deinit(void);

#endif /* IC_STATE_TEST_TOOL_H_ */
