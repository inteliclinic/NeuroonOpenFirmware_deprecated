/*
 * ic_charger.c
 *
 *  Created on: 7 wrz 2015
 *      Author: kn
 */
#include "ic_charger.h"


static charger_state charger_state_struct = {
			.charger_check_flag=0,
			.charger_plugged_flag=0,
			.charger_unplug_check_flag=0,
			.diagnostic_unwanted_pulses=0,
#ifdef UART_CONTROLED_NEUROON_ON
			.uart_controled_flag=0
#endif

};

//------------------------------------PUBLIC FUNCTIONS----------------------------
//call button_init() fun before
void charger_init(){
#ifdef UART_CONTROLED_NEUROON_ON
	if (charger_state_struct.uart_controled_flag==0){
#endif
		if(!(NRF_GPIO->IN & 1<<GPIO_USB_CONNECTED)){
			get_charger_state()->charger_plugged_flag=1;
			charger_config_plugged_int();
		}
		else{
			get_charger_state()->charger_plugged_flag=0;
			charger_config_unplugged_int();
		}
#ifdef UART_CONTROLED_NEUROON_ON
	}
#endif
}

uint8_t charger_is_plugged(){
	return get_charger_state()->charger_plugged_flag;
}
//void charger_clear_state(void){
//	get_charger_state()->charger_plugged_flag=0;
//}

void charger_plugged_irq_handler(){
	get_charger_state()->charger_check_flag=1;
	timer_led_driver_init();
	disable_charger_sense();
}
void charger_unplugged_irq_handler(void){
	get_charger_state()->charger_unplug_check_flag=1;
	timer_led_driver_init();
	disable_charger_sense();
}

static uint8_t charger_counter = 0;
static uint8_t unplug_charger_counter =0;
bool charger_timer_irq_handler(){
	//CHARGER PLUGGED LOGIC
	if(get_charger_check_flag()){
		if(!(NRF_GPIO->IN & 1<<GPIO_USB_CONNECTED)){
			charger_counter++;
			if(charger_counter>=10) {//it should be 1s
				get_charger_state()->charger_plugged_flag=1;
				get_charger_state()->charger_check_flag=0;
				charger_counter=0;
				timer_led_driver_deinit();
				charger_config_plugged_int();
			}
		}
		else{
			get_charger_state()->charger_check_flag=0;
			get_charger_state()->diagnostic_unwanted_pulses++;
			charger_counter=0;
			timer_led_driver_deinit();
			charger_config_unplugged_int();
		}
		return true;
	}//END OF CHARGER PLUGGED LOGIC
	//CHARGER UNPLUGGED
	if(get_charger_unplug_check_flag()){
		if(NRF_GPIO->IN & 1<<GPIO_USB_CONNECTED){
			unplug_charger_counter++;
			if(unplug_charger_counter>=10) {//it should be 1s
				get_charger_state()->charger_plugged_flag=0;
				get_charger_state()->charger_unplug_check_flag=0;
				unplug_charger_counter=0;
				timer_led_driver_deinit();
				charger_config_unplugged_int();
			}
		}
		else{
			get_charger_state()->charger_unplug_check_flag=0;
			get_charger_state()->diagnostic_unwanted_pulses++;
			unplug_charger_counter=0;
			timer_led_driver_deinit();
			charger_config_plugged_int();
		}
		return true;
	}
	//END OF CHARGER UNPLUGGED
	return false;
}



//----------------------------------DIAGNOSTIC---------------------------------------
uint32_t get_unwanted_charger_pulses(){
	return get_charger_state()->diagnostic_unwanted_pulses;
}

//------------------------PRIVATE FUNCTIONS -> DO NOT USE------------------------------------
static charger_state_p get_charger_state(void){
	return &charger_state_struct;
}
static uint8_t get_charger_check_flag(){
	return get_charger_state()->charger_check_flag;
}
static uint8_t get_charger_unplug_check_flag(){
	return get_charger_state()->charger_unplug_check_flag;
}
static inline void disable_charger_sense(void){
	NRF_GPIO->PIN_CNF[GPIO_USB_CONNECTED] =
		  (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)
		| (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos)
		| (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos)
		| (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos)
		| (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos);
}

//config interrupt to wait for charger unplugged
void charger_config_plugged_int(){
	NRF_GPIO->PIN_CNF[GPIO_USB_CONNECTED] =
			(GPIO_PIN_CNF_SENSE_High << GPIO_PIN_CNF_SENSE_Pos)
		  | (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos)
		  | (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos)
		  | (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos)
		  | (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos);
}
//config interrupt to wait for charger plugged
void charger_config_unplugged_int(){
	NRF_GPIO->PIN_CNF[GPIO_USB_CONNECTED] =
		  (GPIO_PIN_CNF_SENSE_Low << GPIO_PIN_CNF_SENSE_Pos)
		| (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos)
		| (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos)
		| (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos)
		| (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos);
}
#ifdef UART_CONTROLED_NEUROON_ON
	void charger_set_flag(void){
		charger_state_struct.charger_plugged_flag = !charger_state_struct.charger_plugged_flag;
		charger_state_struct.uart_controled_flag = !charger_state_struct.uart_controled_flag;
	}
//	void charger_uart_control_flag_set(uint8_t flag){
//		charger_state_struct.uart_controled_flag = flag;
//	}
//	uint8_t charger_uart_control_flag_get(void){
//		return charger_state_struct.uart_controled_flag;
//	}
#endif
