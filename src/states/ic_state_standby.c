/*
 * ic_state_standby.c
 *
 *  Created on: 15 paź 2015
 *      Author: kn
 */

/*
 * ic_state_sleep.c
 *
 *  Created on: 26 wrz 2015
 *      Author: kn
 */

#include "ic_state_standby.h"
//#include "ic_ltc3220.h"
#include "ic_delay.h"
#include "ic_charger.h"
#include "ic_button.h"
#include "ic_gpio.h"
#include "ic_bq27742.h"
#include "ic_uart_log.h"
#include "global_conf.h"
#include <stddef.h>
#ifdef DEBUG_STATE_MACHINE_LOL
	#include <ic_uart_log.h>
#endif //DEBUG_STATE_MACHINE_LOL

static void HARDware_init(void);
static void HARDware_deinit(void);

state_exit_code state_standby(){

	state_exit_code ret_value;
//	uint16_t bq_shutdown;
//	uint8_t slow_timer=0;
//	uint32_t pin_cfg[32];
//	uint8_t i;

	//INIT section
	HARDware_init();
#ifndef UART_CONTROLED_NEUROON_ON
	UARTLOG_Deinit();
#endif


#ifdef DEBUG_STATE_MACHINE_LOL
		    uint8_t log[1];
		    log[0]=0xE3;
		    UARTLOG_send(log, 1);
#endif //DEBUG_STATE_MACHINE_LOL

	//LOOP SECTION
	while (1){
	//	return STANDBY_SM_BUTTON_PRESSED;
          __WFE();
          __SEV();
          __WFE();
		if(charger_is_plugged()){
			ret_value = STANDBY_SM_CHARGER_PLUGGED;
			break;
		}
		if(button_is_pressed()){
			button_clear_state();
			ret_value = STANDBY_SM_BUTTON_PRESSED;
			break;
		}
		WDT_RR();
	}

	//DEINIT SECTION
	HARDware_deinit();

#ifndef UART_CONTROLED_NEUROON_ON
	UARTLOG_Init(false, NULL);
#endif

	return ret_value;
}
static void HARDware_init(){
	ResetBit(GPIO_POWER_ANALOG);
	/*IF TWI/I2C NEEDED (BQ, LTC, ADS, LIS)*/
//	TWI_PIN_SELECT(GPIO_SCL_PIN, GPIO_SDA_PIN);
//	TWI_Init(K400);
	/*IF SPI NEEDED (AFE, FLASH) */
	//spi_master_init();
	/*IF YOU USE ANY DIGITAL INTERFACES or TEMP_MEASURE*/
	ResetBit(GPIO_POWER_DIGITAL);
	/*IF YOU USE LEDS*/
	//SetBit(GPIO_LEDS_ON);
	//ltc3220_clear_async();

	/*IF YOU USE ADS (EEG_SIGNAL)*/
	//SetBit(GPIO_POWER_ANALOG);
	//ads_init();

	/*IF YOU NEED RTC IRQ*/
	timer_RTC_init();

	/*IF YOU NEED TIMER1 (signal measurement, LTC refreshing)*/
	//timer_busInit(125);

	/*CHOOSE NEEDED Sections*/
	//timer_startSection(SECTION_EEG);
	//timer_startSection(SECTION_PULSE);
	//timer_startSection(SECTION_TEMP);
	//timer_startSection(SECTION_ACC);
	//timer_startSection(SECTION_FLASH_WRITE);
	//timer_startSection(SECTION_BATTERY);
	//timer_startSection(SECTION_LTC_REFRESH);

	/*HIGHLY RECOMMENDED IN ALL INIT SECTIONS*/
	button_clear_state();
	charger_init();
}

static void HARDware_deinit(){
	/*IF TWI/I2C WAS USED (BQ, LTC, ADS, LIS)*/
	//TWI_Deinit();
	/*IF SPI WAS USED (AFE, FLASH) */
	//spi_master_deinit();
	/*IF YOU USED ANY DIGITAL INTERFACES or TEMP_MEASURE*/
	ResetBit(GPIO_POWER_DIGITAL);
	/*IF YOU USED LEDS (LTC)*/
	//ResetBit(GPIO_LEDS_ON);
	//ltc3220_clear_async();

	/*IF YOU USE ADS (EEG_SIGNAL)*/
	//ResetBit(GPIO_POWER_ANALOG);
	//ads_deinit();

	/*IF RTC IRQ WAS USED*/
	timer_RTC_deinit();

	/*IF YOU UDE TIMER1 (signal measurement, LTC refreshing)*/
	//timer_busDeinit();

	/*CHOOSE USED Sections*/
	//timer_stopSection(SECTION_EEG);
	//timer_stopSection(SECTION_PULSE);
	//timer_stopSection(SECTION_TEMP);
	//timer_stopSection(SECTION_ACC);
	//timer_stopSection(SECTION_FLASH_WRITE);
	//timer_stopSection(SECTION_BATTERY);
	//timer_stopSection(SECTION_LTC_REFRESH);

	/*HIGHLY RECOMMENDED IN ALL DEINIT SECTIONS*/
	button_clear_state();
	charger_init();

}

