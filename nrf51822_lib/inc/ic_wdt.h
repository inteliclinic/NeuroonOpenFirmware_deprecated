#ifndef __IC_WDT__H
#define __IC_WDT__H

/**
 * @file
 * @brief watchdog functions header
 */
#include "ic_lib_config.h"

/** RESET REASON CODE */
typedef enum {
	RESETPIN,
	DOG,
	SREQ,
	LOCKUP,
	OFF,
	LPCOMP,
	DIF,
	GEN
}reset_reason;

/** GENERIC FUNCTIONS */
void WDT_Config (uint32_t counter_value);
bool WDT_Start (void);
void WDT_RR(void);
uint8_t WDT_Resetreas (void);

#endif
