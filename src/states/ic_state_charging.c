/*
 * ic_state_charging.c
 *
 *  Created on: 26 wrz 2015
 *      Author: kn
 */

#include "ic_state_charging.h"
#include "ic_ltc3220.h"
#include "ic_delay.h"
#include "ic_charger.h"
#include "ic_bq27742.h"

#ifdef DEBUG_STATE_MACHINE_LOL
	#include <ic_uart_log.h>
#endif

static void HARDware_init(void);
static void HARDware_deinit(void);

state_exit_code state_charging(){

	HARDware_init();
	//LOOP SECTION
	uint8_t battery_charged=0;
	uint8_t fully_charged=false;
	bool battery_charged_flag=false;
	bool battery_charged_flag_last=false;
	bool flag_changes=false;
	bool charging_allowed=false;

	ltc3220_led_ramp_up(RGB_COLOR_R, 0, 0x10, ltc3220_Left);

	while (1){
		flag_changes=false;
		fully_charged=bq_battery_measurement();
		charging_allowed = bq_is_charging_allowed();

		if (fully_charged>90){
			battery_charged++;
			if (battery_charged>=10)
				battery_charged_flag=true;
		}
		else
			battery_charged=0;

		if(battery_charged_flag_last != battery_charged_flag)
			flag_changes=true;

		battery_charged_flag_last=battery_charged_flag;

		if(flag_changes)
		{
			if(battery_charged_flag==false)
			{
				ltc3220_led_ramp_down(RGB_COLOR_G, 0x10, 0x0, ltc3220_Left);
				ltc3220_led_ramp_up(RGB_COLOR_R, 0, 0x10, ltc3220_Left);
			}
		}

		if (!battery_charged_flag && charging_allowed)
		{
			ResetBit(GPIO_LEDS_ON);
			SetBit(GPIO_LEDS_ON);
			ltc3220_set_brightness(RGB2_G_CHANNEL, 0x00);
			ltc3220_set_brightness(RGB2_R_CHANNEL, 0x10);
		}
		else if (!battery_charged_flag && !charging_allowed)
		{
			// ResetBit(GPIO_LEDS_ON);
			// SetBit(GPIO_LEDS_ON);
			// ltc3220_set_brightness(RGB2_G_CHANNEL, 0x05);
			// ltc3220_set_brightness(RGB2_R_CHANNEL, 0x15);
			ltc3220_led_ramp_down(RGB_COLOR_R, 0x10, 0x0, ltc3220_Left);
			ltc3220_led_ramp_up(RGB_COLOR_R, 0, 0x10, ltc3220_Left);
		}
		else
		{
			ResetBit(GPIO_LEDS_ON);
			SetBit(GPIO_LEDS_ON);
			ltc3220_set_brightness(RGB2_R_CHANNEL, 0x00);
			ltc3220_set_brightness(RGB2_G_CHANNEL, 0x10);
		}

		__WFI();
		if(!charger_is_plugged()){
			break;
		}
		//charger_init();
		WDT_RR();
	}

	//DEINIT SECTION
	if(battery_charged_flag==true )
		ltc3220_led_ramp_down(RGB_COLOR_G, 0x10, 0x0, ltc3220_Left);
	if(battery_charged_flag==false)
		ltc3220_led_ramp_down(RGB_COLOR_R, 0x10, 0x0, ltc3220_Left);


	HARDware_deinit();

	return CHARGING_SM_CHARGER_UNPLUGGED;
}

static void HARDware_init(){
	/*IF TWI/I2C NEEDED (BQ, LTC, ADS, LIS)*/
	TWI_PIN_SELECT(GPIO_SCL_PIN, GPIO_SDA_PIN);
	TWI_Init(K400);
	/*IF YOU USE ANY DIGITAL INTERFACES or TEMP_MEASURE*/
	SetBit(GPIO_POWER_DIGITAL);
	/*IF YOU USE LEDS*/
	SetBit(GPIO_LEDS_ON);
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
	TWI_Deinit();
	/*IF YOU USED ANY DIGITAL INTERFACES or TEMP_MEASURE*/
	ResetBit(GPIO_POWER_DIGITAL);
	/*IF YOU USED LEDS (LTC)*/
	ResetBit(GPIO_LEDS_ON);
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
