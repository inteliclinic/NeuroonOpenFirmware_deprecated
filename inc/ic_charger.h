/*
 * ic_charger.h
 *
 *  Created on: 7 wrz 2015
 *      Author: kn
 */

#ifndef IC_CHARGER_H_
#define IC_CHARGER_H_
#include "ic_lib_config.h"
#include "ic_timers.h"
#include <global_conf.h>

//-------------------------------CHARGER MODULE----------------------------------


//-------------------------------DOCUMENTATION---------------------------------
/**
 * HACK:
 *
 * --------------------------------HOW TO INIT------------------------------------
 * You need to do some stuff before using this module:
 * 1. GPIO initialization 	-> function hardware_init();
 * 2. IRQ initialization	-> function GPIOTE_interrupt_enable();
 * 3. Charger initialization-> to check after restart the state
 * 4. You need to call:
 * 		4.1		charger_plugged_irq_handler() and charger_unplugged_irq_handler(void); 	-> in GPIOTE_IRQHandler();
 * 		4.2		charger_timer_irq_handler()		-> in TIMER2_IRQHandler();
 * 5. Charger module is ready to work
 * -------------------------------HOW TO USE-------------------------------------
 * To check if charger is plugged:
 * is_charger_is_plugged() -> returns charger state -> 1 plugged, 0 unplugged
 * You do not need to clears this flag. It should be automatic
 *
 * --------------------------------DIAGNOSTIC--------------------------------------
 * To check how many times charger irq has been released but charger wasn't plugged.
 * get_unwanted_charger_pulses
 */
typedef struct charger_state{

	uint8_t charger_check_flag;			/* flag is set in it_handlers.c->GPIOTE_IRQHandler when button is pressed
	 	 	 	 	 	 	 	 	 	 reset in timer handlers (?) #TODO */
	uint8_t charger_unplug_check_flag;

	uint8_t charger_plugged_flag;		/* flag is set in TIMER2_IRQHandler */
	//uint8_t charger_unplugged_flag;		/* flag is set in GPIOTE_IRQHandler */

	uint32_t diagnostic_unwanted_pulses; /* flag which counts unwanted pulses on button input */

#ifdef UART_CONTROLED_NEUROON_ON
	uint8_t uart_controled_flag;
	//uint8_t uart_init_flag;
#endif
}charger_state, *charger_state_p;

//------------------------------------PUBLIC FUNCTIONS----------------------------
uint8_t charger_is_plugged(void);

//init after reset! may be used to asynchoronus state determine.
void charger_init(void);
void charger_plugged_irq_handler(void);
void charger_unplugged_irq_handler(void);
bool charger_timer_irq_handler(void);



//----------------------------------DIAGNOSTIC---------------------------------------
uint32_t get_unwanted_charger_pulses(void);

//------------------------PRIVATE FUNCTIONS -> DO NOT USE------------------------------------
static charger_state_p get_charger_state(void);
static uint8_t get_charger_check_flag(void);
static uint8_t get_charger_unplug_check_flag(void);
static inline void disable_charger_sense(void);
//config interrupt to wait for charger unplugged
void charger_config_plugged_int(void);
//config interrupt to wait for charger plugged
void charger_config_unplugged_int(void);

#ifdef UART_CONTROLED_NEUROON_ON
	void charger_set_flag(void);
//	void charger_uart_control_flag_set(uint8_t flag);
//	uint8_t charger_uart_control_flag_get(void);
#endif

#endif /* IC_CHARGER_H_ */
