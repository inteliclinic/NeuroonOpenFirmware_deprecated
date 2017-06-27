//---------------------------------------------------------------------------------------------------
#ifndef INC_IC_LOG_TEMPLATE_H_
#define INC_IC_LOG_TEMPLATE_H_
//---------------------------------------------------------------------------------------------------
#include <stdint.h>
#include <stdbool.h>
#include <ic_crc8.h>
#include <ic_uart_log.h>
//---------------------------------------------------------------------------------------------------
#define LOG_FRAME_TEMPLATE_FUN_NAME_INT1(FUN_NAME, NAME)		FUN_NAME##_##NAME
#define LOG_FRAME_TEMPLATE_FUN_NAME_INT2(FUN_NAME, NAME)		LOG_FRAME_TEMPLATE_FUN_NAME_INT1(FUN_NAME, NAME)
#define LOG_FRAME_TEMPLATE_FUN_NAME(NAME)						LOG_FRAME_TEMPLATE_FUN_NAME_INT2(LOG_FRAME_TEMPLATE_FUN_PREFIX, NAME)
//---------------------------------------------------------------------------------------------------
#endif /* INC_IC_LOG_TEMPLATE_H_ */
//---------------------------------------------------------------------------------------------------
//Un-comment below definition only during testing/changing/developing. Have to be commented during normal operation!

//#define LOG_TEMPLATE_DEVELOPING

#ifdef LOG_TEMPLATE_DEVELOPING
	#define LOG_FRAME_TEMPLATE_DEF
	#define LOG_FRAME_TEMPLATE_FUN_PREFIX		logfi16
	#define LOG_FRAME_TEMPLATE_TYPE				int16_t
	#define LOG_FRAME_TEMPLATE_SIZE				15
	#define LOG_FRAME_TEMPLATE_STR_NAME			LOGFi16
#endif //LOG_TEMPLATE_DEVELOPING
//---------------------------------------------------------------------------------------------------
#if defined(LOG_FRAME_TEMPLATE_FUN_PREFIX) && defined(LOG_FRAME_TEMPLATE_TYPE) && defined(LOG_FRAME_TEMPLATE_SIZE) && defined(LOG_FRAME_TEMPLATE_STR_NAME)
	//---------------------------------------------------------------------------------------------------
	/**
	 * LOG-FRAME main structure
	 */
	typedef struct {
		//internal anonymous union
		union {
			struct __attribute__((packed)) {
				uint8_t 					id;									/// frame id
				uint8_t						fc;									/// frame counter (mod 256) - for missed frame test
				LOG_FRAME_TEMPLATE_TYPE		payload[LOG_FRAME_TEMPLATE_SIZE];	/// payload (array of samples)
				uint8_t						crc8;								/// crc8 of id, pn and payload
			} frame;
			uint8_t ftab[3+(sizeof(LOG_FRAME_TEMPLATE_TYPE))*LOG_FRAME_TEMPLATE_SIZE];
		};
		uint8_t							plPos;	/// payload position - packet status indicator
	} LOG_FRAME_TEMPLATE_STR_NAME;
	//---------------------------------------------------------------------------------------------------
	/**
	 * @brief	init/reinit log-frame especially outside function (e.g. in global scope)
	 * @param	id	Frame ID.
	 */
	#define LOG_SIGNAL_FRAME_INIT(id)	\
	{							\
		.ftab[0] = (id)			\
	}
	//---------------------------------------------------------------------------------------------------
	/**
	 * @brief	Add new sample to frame-builder and insert to logger if payload full (with frame reinit).
	 *
	 * @param	lf		Pointer to instance.
	 * @param	sample	New sample to insert into payload.
	 *
	 * @return	Status code:
	 * 				-1	if frame instance is inconsistent - frame should be reinitialize
	 * 				0	if sample was inserted but payload not full.
	 * 				1	if sample was inserted and payload full, so frame inserted into logger
	 */

	int
	LOG_FRAME_TEMPLATE_FUN_NAME(addSample)			(LOG_FRAME_TEMPLATE_STR_NAME *lf, LOG_FRAME_TEMPLATE_TYPE sample);
	//---------------------------------------------------------------------------------------------------
	/**
	 * @brief	Clear frame instance (frame-builder object).
	 * 			After call this function, #LOG_FRAME_TEMPLATE_SIZE samples have to be inserted by @ref addSample(...)
	 * 			method before auto logger insertion.
	 *
	 * @param	lf		Pointer to instance.
	 */
	static inline void
	LOG_FRAME_TEMPLATE_FUN_NAME(clear)			(LOG_FRAME_TEMPLATE_STR_NAME *lf) {
		lf->plPos = 0;
		lf->frame.fc = 0;
	}
	//---------------------------------------------------------------------------------------------------
#endif //defined(LOG_FRAME_TEMPLATE_FUN_PREFIX) && defined(LOG_FRAME_TEMPLATE_TYPE) && defined(LOG_FRAME_TEMPLATE_SIZE) && defined(LOG_FRAME_TEMPLATE_STR_NAME)
//---------------------------------------------------------------------------------------------------
	#if defined(LOG_FRAME_TEMPLATE_DEF) && defined(LOG_FRAME_TEMPLATE_FUN_PREFIX) && defined(LOG_FRAME_TEMPLATE_TYPE) && defined(LOG_FRAME_TEMPLATE_SIZE) && defined(LOG_FRAME_TEMPLATE_STR_NAME)
	//---------------------------------------------------------------------------------------------------
	int
	LOG_FRAME_TEMPLATE_FUN_NAME(addSample)			(LOG_FRAME_TEMPLATE_STR_NAME *lf, LOG_FRAME_TEMPLATE_TYPE sample) {
		uint8_t p = lf->plPos;
		//Check possible frame inconsistency
		if (p >= LOG_FRAME_TEMPLATE_SIZE) {
			return -1;
		}
		//if first sample is inserting
		if (p == 0) {
			//Compute initial crc8: id & pn - possible place to speed up...
			lf->frame.crc8 = crc8_tab(lf->ftab, 2);
		}
		//Add sample to payload
		lf->frame.payload[p++] = sample;
		//Compute new crc8
		lf->frame.crc8 = crc8_tabex(lf->frame.crc8, (uint8_t*)&sample, sizeof(sample));
		//Check logger trigger - full payload
		if (p >= LOG_FRAME_TEMPLATE_SIZE) {
			//Insert to logger
			UARTLOG_send(lf->ftab, sizeof(lf->ftab));
			//Clear & reinit frame
			lf->plPos = 0;
			lf->frame.fc++;
			//lf->frame.crc8 = 0x00; // not required because of new packet initial calculations

			return 1;
		}
		//Update plPos (variable $p was incremented above)
		lf->plPos = p;

		return 0;
	}
	//---------------------------------------------------------------------------------------------------
#endif // defined(LOG_FRAME_TEMPLATE_DEF) && defined(LOG_FRAME_TEMPLATE_FUN_PREFIX) && defined(LOG_FRAME_TEMPLATE_TYPE) && defined(LOG_FRAME_TEMPLATE_SIZE) && defined(LOG_FRAME_TEMPLATE_STR_NAME)
//---------------------------------------------------------------------------------------------------
//Cleanup section:
#undef LOG_FRAME_TEMPLATE_FUN_PREFIX
#undef LOG_FRAME_TEMPLATE_TYPE
#undef LOG_FRAME_TEMPLATE_SIZE
#undef LOG_FRAME_TEMPLATE_STR_NAME
//---------------------------------------------------------------------------------------------------
