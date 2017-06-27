//---------------------------------------------------------------------------------------------------
#include "ic_movement.h"
//---------------------------------------------------------------------------------------------------
#ifndef NRF51822_LIB_INC_IC_FIFO_H_
#define NRF51822_LIB_INC_IC_FIFO_H_
//---------------------------------------------------------------------------------------------------
// FIFO uint8_t (especially for UARTLOG module) ()
#define FIFO_UNDERFLOW_RET			0
#define FIFO_TEMPLATE_DEC
#define FIFO_TEMPLATE_FUN_PREFIX	fifou8
#define FIFO_TEMPLATE_TYPE			uint8_t
#define	FIFO_TEMPLATE_STR_NAME		FIFO_u8
#include <ic_fifo_template.h>
#undef FIFO_UNDERFLOW_RET
#undef FIFO_TEMPLATE_DEC
#undef FIFO_TEMPLATE_FUN_PREFIX
#undef FIFO_TEMPLATE_TYPE
#undef	FIFO_TEMPLATE_STR_NAME
//---------------------------------------------------------------------------------------------------
// FIFO uint16_t (especially for LTC_ASYNC_SENDER module)
#define FIFO_UNDERFLOW_RET			0
#define FIFO_TEMPLATE_DEC
#define FIFO_TEMPLATE_FUN_PREFIX	fifou16
#define FIFO_TEMPLATE_TYPE			uint16_t
#define	FIFO_TEMPLATE_STR_NAME		FIFO_u16
#include <ic_fifo_template.h>
#undef FIFO_UNDERFLOW_RET
#undef FIFO_TEMPLATE_DEC
#undef FIFO_TEMPLATE_FUN_PREFIX
#undef FIFO_TEMPLATE_TYPE
#undef	FIFO_TEMPLATE_STR_NAME
//---------------------------------------------------------------------------------------------------
#define FIFO_UNDERFLOW_RET			0
#define FIFO_TEMPLATE_DEC
#define FIFO_TEMPLATE_FUN_PREFIX	fifoi32
#define FIFO_TEMPLATE_TYPE			int32_t
#define	FIFO_TEMPLATE_STR_NAME		FIFO_i32
#include <ic_fifo_template.h>
#undef FIFO_UNDERFLOW_RET
#undef FIFO_TEMPLATE_DEC
#undef FIFO_TEMPLATE_FUN_PREFIX
#undef FIFO_TEMPLATE_TYPE
#undef	FIFO_TEMPLATE_STR_NAME
//---------------------------------------------------------------------------------------------------
// FIFO acc_data
#define FIFO_UNDERFLOW_RET			(acc_data){0,0,0}
#define FIFO_TEMPLATE_DEC
#define FIFO_TEMPLATE_FUN_PREFIX	fifoMOVE
#define FIFO_TEMPLATE_TYPE			acc_data
#define	FIFO_TEMPLATE_STR_NAME		FIFO_MOVE
#include <ic_fifo_template.h>
#undef FIFO_UNDERFLOW_RET
#undef FIFO_TEMPLATE_DEC
#undef FIFO_TEMPLATE_FUN_PREFIX
#undef FIFO_TEMPLATE_TYPE
#undef	FIFO_TEMPLATE_STR_NAME
//---------------------------------------------------------------------------------------------------
#endif /* NRF51822_LIB_INC_IC_FIFO_H_ */
//---------------------------------------------------------------------------------------------------
