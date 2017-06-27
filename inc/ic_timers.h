/*
 * ic_timers.h
 *
 *  Created on: 7 wrz 2015
 *      Author: kn
 */

#ifndef IC_TIMERS_H_
#define IC_TIMERS_H_
#include "ic_lib_config.h"
#include "ic_button.h"
#include "ic_charger.h"

typedef enum{
	SECTION_EEG,
	SECTION_PULSE,
	SECTION_TEMP,
	SECTION_ACC,
	SECTION_FLASH_WRITE,
	SECTION_BATTERY,
	SECTION_LTC_REFRESH
} SECTION_INDEX;

//---------------------------------------------------------------------------------------------------
/** EEG & PULSE & SATRUATION MEASUREMENT TIMER OPTIONS */
//---------------------------------------------------------------------------------------------------
/**
 * @brief	Init timer1
 * @param	freq	Operation frequency: 125Hz, 200Hz or 250Hz.
 */
void timer_busInit(unsigned int freq);
void timer_busDeinit(void);
//---------------------------------------------------------------------------------------------------
void timer_startSection(SECTION_INDEX ind);
void timer_stopSection(SECTION_INDEX ind);
bool timer_getSection(SECTION_INDEX ind);
//---------------------------------------------------------------------------------------------------
typedef struct timer_state{
	uint8_t how_many_users;
	uint8_t init_flag;
}timer_state, *timer_state_p;

uint32_t timer_get_ms_counter(void);

void TIMER1_IRQHandler(void);
void TIMER2_IRQHandler(void);
/**	Time counter*/
void RTC1_IRQHandler(void);


uint32_t get_timer1Counter(void);


/**
 * Timer functions for LED/VIBRA driver. Also button and charger modules use it.
 */
void timer_led_driver_init(void);
void timer_led_driver_deinit(void);
void timer_led_driver_hard_deinit(void);

/** Time keeper :)*/
void timer_RTC_init(void);
void timer_RTC_deinit(void);
uint32_t timer_get_RTC_counter(void);


//---------------------------------------------- AD ------------------------------------------------

/**
 * @brief	Capture current TIMER1 value to CC[3] register
 *
 * @return  Register value
 */
static uint32_t timer1_capture();

/**
 * @brief	Check if time elapsed since last reference capture is above threshold
 *
 * @param 	threshold	Time in ms
 * @return  Has threshold time already elapsed:
 *            1 - true
 *            0 - false
 */
bool timer1_capture_elapsed(uint32_t threshold);


/**
 * @brief	Reads capture register and updates reference capture value
 */
void timer1_update_reference();

/**
 * @brief	Get number of 2Hz timer ticks
 *
 * @return  Number of 2Hz timer ticks
 */
uint32_t timer1_2Hz_get();

/**
 * @brief	Clear 2Hz timer counter
 */

void timer1_2Hz_clear();
//--------------------------------------------------------------------------------------------------

//----------------------------PRIVATE-------------------------------
static timer_state_p get_LED_timer_state(void);

void timer_led_driver_start(void);
void timer_led_driver_stop(void);

void timer_measurement_start(void);
void timer_measurement_stop(void);
#endif /* IC_TIMERS_H_ */
