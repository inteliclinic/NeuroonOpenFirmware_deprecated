#include <ic_movement.h>
//---------------------------------------------------------------------------------------------------
#ifndef NRF51822_LIB_INC_IC_CBUFFER_H_
#define NRF51822_LIB_INC_IC_CBUFFER_H_
//---------------------------------------------------------------------------------------------------
#define	CBUFFER_TEMPLATE_DEC
#define	CBUFFER_TEMPLATE_FUN_PREFIX			cbumove
#define	CBUFFER_TEMPLATE_TYPE				acc_data
#define	CBUFFER_TEMPLATE_STR_NAME			CBuMOVE
#define	CBUFFER_DEFAULT_RET					(acc_data){0,0,0}
#include <ic_cbuffer_template.h>
//---------------------------------------------------------------------------------------------------
#define	CBUFFER_TEMPLATE_DEC
#define	CBUFFER_TEMPLATE_FUN_PREFIX			cbu16
#define	CBUFFER_TEMPLATE_TYPE				uint16_t
#define	CBUFFER_TEMPLATE_STR_NAME			CBu16
#define	CBUFFER_DEFAULT_RET					0
#define	CBUFFER_TEMPLATE_SUM_TYPE			uint32_t
#define	CBUFFER_TEMPLATE_WEIGHTED_SUM_TYPE	uint32_t
#include <ic_cbuffer_template.h>
//---------------------------------------------------------------------------------------------------
#define	CBUFFER_TEMPLATE_DEC
#define	CBUFFER_TEMPLATE_FUN_PREFIX			cbi16
#define	CBUFFER_TEMPLATE_TYPE				int16_t
#define	CBUFFER_TEMPLATE_STR_NAME			CBi16
#define	CBUFFER_DEFAULT_RET					0
#define	CBUFFER_TEMPLATE_SUM_TYPE			int32_t
#define	CBUFFER_TEMPLATE_WEIGHTED_SUM_TYPE	int32_t
#include <ic_cbuffer_template.h>
//---------------------------------------------------------------------------------------------------
#define	CBUFFER_TEMPLATE_DEC
#define	CBUFFER_TEMPLATE_FUN_PREFIX			cbu8
#define	CBUFFER_TEMPLATE_TYPE				uint8_t
#define	CBUFFER_TEMPLATE_STR_NAME			CBu8
#define	CBUFFER_DEFAULT_RET					0
#define	CBUFFER_TEMPLATE_SUM_TYPE			uint16_t
//#define	CBUFFER_TEMPLATE_WEIGHTED_SUM_TYPE	int32_t
#include <ic_cbuffer_template.h>
//---------------------------------------------------------------------------------------------------
#define	CBUFFER_TEMPLATE_DEC
#define	CBUFFER_TEMPLATE_FUN_PREFIX			cbi32
#define	CBUFFER_TEMPLATE_TYPE				int32_t
#define	CBUFFER_TEMPLATE_STR_NAME			CBi32
#define	CBUFFER_DEFAULT_RET					0
#define	CBUFFER_TEMPLATE_SUM_TYPE			int64_t
#define	CBUFFER_TEMPLATE_WEIGHTED_SUM_TYPE	int64_t
#include <ic_cbuffer_template.h>
//---------------------------------------------------------------------------------------------------

#endif /* NRF51822_LIB_INC_IC_CBUFFER_H_ */
