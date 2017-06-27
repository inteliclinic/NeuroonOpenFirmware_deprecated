//---------------------------------------------------------------------------------------------------
#ifndef INC_GLOBAL_CONF_H_
#define INC_GLOBAL_CONF_H_
///---------------------------------------------------------------------------------------------------

/** USER DEFINE BLOCK 1 BEGIN */
/// 0 - option disabled
/// 1 - option enabled

#define GCONF_RAW_GLOBAL_DISABLE		0

#define GCONF_RAW_FLASH_STREAM_ON		1
#define GCONF_RAW_UART_STREAM_ON		1

#define GCONF_RAW_CLEAR_OPTION			1
#define GCONF_RAW_LOG_SECONDS			1
#define GCONF_RAW_LOG_EPOCH				1
#define GCONF_RAW_LOG_EP_HEADER		1
#define GCONF_RAW_LOG_EEG				1
#define GCONF_RAW_LOG_ACC				0
#define GCONF_RAW_LOG_IRLED				1			///use external log_enable(...) in main.c
#define GCONF_RAW_LOG_REDLED			0			///use external log_enable(...) in main.c
#define GCONF_RAW_LOG_NN					0

//---------------------------------------------UART CONTROL------------------------------------
#define UART_CONTROLED_NEUROON		0

//---------------------------------------------DOUBLE CLICK--------------------------------------
#define BUTTON_DOUBLE_CLICK				0

/** USER DEFINE BLOCK 1 END */

///---------------------------------------------------------------------------------------------------


//-- DEFINITION CONVERTER ---------------------------------------------------------------------------
#if (GCONF_RAW_GLOBAL_DISABLE == 0)
	#if (GCONF_RAW_FLASH_STREAM_ON == 1)
		#define UARTLOG_USE_FLASH_STREAM true
	#else
		#define UARTLOG_USE_FLASH_STREAM false
	#endif
	#if (GCONF_RAW_UART_STREAM_ON == 1)
		#define UARTLOG_USE_UART_STREAM true
	#else
		#define UARTLOG_USE_UART_STREAM false
	#endif
	#if (GCONF_RAW_CLEAR_OPTION == 1)
		#define BLECON_USE_RAW_CLEAR
	#endif
	#if (GCONF_RAW_LOG_SECONDS == 1)
		#define EPOCH_LOG_SECONDS
	#endif
	#if (GCONF_RAW_LOG_EPOCH == 1)
		#define EPOCH_LOG_COMPLETED_EPOCH
	#endif
	#if (GCONF_RAW_LOG_EP_HEADER == 1)
		#define EPOCH_LOG_RAW_HEADER
	#endif
	#if (GCONF_RAW_LOG_ACC == 1)
		#define ACC_LOG_RAW_DATA
	#endif
	#if (GCONF_RAW_LOG_IRLED == 1)
		#define LOG_USE_IRLED
	#endif
	#if (GCONF_RAW_LOG_REDLED == 1)
		#define LOG_USE_REDLED
	#endif
	#if (GCONF_RAW_LOG_EEG == 1)
		#define EEG_LOG_RAW_SIGNAL
	#endif
	#if (GCONF_RAW_LOG_NN == 1)
		#define LOG_USE_NN
	#endif
#else
	#define UARTLOG_USE_FLASH_STREAM	false
	#define UARTLOG_USE_UART_STREAM		false

#if (UART_CONTROLED_NEUROON == 0)
	#define UART_CONTROLED_NEUROON_ON
#endif

#if (BUTTON_DOUBLE_CLICK == 1)
	#define BUTTON_DOUBLE_CLICK_ON
#endif

#endif
//---------------------------------------------------------------------------------------------------
#endif /* INC_GLOBAL_CONF_H_ */
//---------------------------------------------------------------------------------------------------
