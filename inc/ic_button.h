/*
 * ic_button.h
 *
 *  Created on: 1 wrz 2015
 *      Author: kn
 */

#ifndef IC_BUTTON_H_
#define IC_BUTTON_H_
//#include "app_managers.h"
#include "ic_timers.h"
//#include "ic_led_driver.h"
//-------------------------------BUTTON MODULE----------------------------------


//-------------------------------DOCUMENTATION---------------------------------
/**
 * HACK:
 *
 * --------------------------------HOW TO INIT------------------------------------
 * You need to do some stuff before using this module:
 * 1. GPIO initialization 	-> function hardware_init();
 * 2. IRQ initialization	-> function GPIOTE_interrupt_enable();
 * 3. You need to call:
 * 		3.1		button_pressed_irq_handler() 	-> in GPIOTE_IRQHandler();
 * 		3.2		button_timer_irq_handler()		-> in TIMER2_IRQHandler();
 * 4. Button module is ready to work
 * -------------------------------HOW TO USE-------------------------------------
 * To check if button was pressed: (ATTENTION it is NOT clearing the state)
 * is_button_pressed() -> returns button state -> 1 pressed, 0 unpressed
 * after check clear button state:
 * clear_button_state()
 *
 * --------------------------------DIAGNOSTIC--------------------------------------
 * To check how many times button has pressed state but shorter than 1s
 * get_unwanted_button_pulses
 */


typedef struct button_state{
	uint8_t button_check_flag;			/* flag is set in it_handlers.c->GPIOTE_IRQHandler when button is pressed
	 	 	 	 	 	 	 	 	 	 reset in timer handlers (?) #TODO */
	uint8_t button_pressed_flag;		/* flag is set in TIMER2_IRQHandler */
#ifdef BUTTON_DOUBLE_CLICK_ON
	bool	button_single_clicked;
	bool	button_double_clicked;
#endif // BUTTON_DOUBLE_CLICK_ON
//	uint8_t timer1_enabled_flag;		/* flag is set in timer1 init #TODO - app_drivers->timer_measurement_init*/
//	uint8_t timer2_enabled_flag;		/* flag is set in timer2 init #TODO
//	 	 	 	 	 	 	 	 	 	 false - disabled, true - enabled, 2 - enabled only for button check*/
	uint32_t diagnostic_unwanted_pulses; /* flag which counts unwanted pulses on button input */
}button_state, *button_state_p;

//------------------------------------PUBLIC FUNCTIONS----------------------------
void button_init(void);
bool button_is_pressed(void);
#ifdef BUTTON_DOUBLE_CLICK_ON
bool button_single_clicked(void);
bool button_double_clicked(void);
#endif // BUTTON_DOUBLE_CLICK_ON
void button_clear_state(void);
#ifdef UART_CONTROLED_NEUROON_ON
void button_set_state (void);
#endif


//------------------------------------IRQ FUNCTIONS----------------------------
void button_pressed_irq_handler(void);
bool button_timer_irq_handler(void);

//----------------------------------DIAGNOSTIC---------------------------------------
uint32_t get_unwanted_button_pulses(void);

#endif /* IC_BUTTON_H_ */
