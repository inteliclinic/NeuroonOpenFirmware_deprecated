/**
 * @file
 * @brief watchdog methods
 */

#include "ic_wdt.h"
#ifdef USE_IC_WDT
/**
 * @fn WDT_Config()
 * @brief watchdog timer configuration:
 * keep the watchdog running while the CPU is sleeping;
 * set counter_value;
 * @param counter_value
 */
void WDT_Config (uint32_t counter_value)
{
	NRF_WDT-> CONFIG = (WDT_CONFIG_SLEEP_Run<<WDT_CONFIG_SLEEP_Pos)
			|(WDT_CONFIG_HALT_Pause<<WDT_CONFIG_HALT_Pos);
	NRF_WDT->CRV = counter_value;
	NRF_WDT->POWER = (WDT_POWER_POWER_Enabled<<WDT_POWER_POWER_Pos);
}


/**
 * @fn WDT_Start()
 * @brief start watchdog timer; check run status
 * @return true if start enable
 */
bool WDT_Start ()
{
	NRF_WDT->TASKS_START = 1;
	if (NRF_WDT->RUNSTATUS == 0){
		return false;
	}
	return true;
}

/**
 * @fn WDT_RR()
 * @brief reload request enable on 0 channel
 */
void WDT_RR ()
{
	NRF_WDT->RREN = (WDT_RREN_RR0_Enabled<<WDT_RREN_RR0_Pos);
	NRF_WDT->RR [0] = (WDT_RR_RR_Reload<<WDT_RR_RR_Pos);
}

/**
 * @fn WDT_Resetreas()
 * @brief check reset reason
 * @return reset reason code
 */
uint8_t WDT_Resetreas ()
{
	//TODO: Robertowi zawsze przychodzi GEN
	uint32_t resetreas = 0;
	resetreas = (NRF_POWER->RESETREAS);
	NRF_POWER->RESETREAS = resetreas;
	if (resetreas & 0x01){
		NRF_POWER->RESETREAS = 0x01;
		reset_reason rr_check = RESETPIN;
		return rr_check;
	}
	if (resetreas & 0x02){
		NRF_POWER->RESETREAS = 0x02;
		reset_reason rr_check = DOG;
		return rr_check;
	}
	if (resetreas & 0x04){
		NRF_POWER->RESETREAS = 0x04;
		reset_reason rr_check = SREQ;
		return rr_check;
	}
	if (resetreas & 0x10000){
		NRF_POWER->RESETREAS = 0x10000;
		reset_reason rr_check = OFF;
		return rr_check;
	}
	if (resetreas & 0x20000){
		NRF_POWER->RESETREAS = 0x20000;
		reset_reason rr_check = LPCOMP;
		return rr_check;
	}
	if (resetreas & 0x40000){
		NRF_POWER->RESETREAS = 0x40000;
		reset_reason rr_check = DIF;
		return rr_check;
	}
	if (resetreas & 0x08){
		NRF_POWER->RESETREAS = 0x08;
		reset_reason rr_check = LOCKUP;
		return rr_check;
	}
	else {
		reset_reason rr_check = GEN;
		return rr_check;
	}
}
#endif
