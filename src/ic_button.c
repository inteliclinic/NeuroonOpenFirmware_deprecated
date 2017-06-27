/*
 * ic_button.c
 *
 *  Created on: 1 wrz 2015
 *      Author: kn
 */


#include "ic_button.h"
#include "global_conf.h"


static button_state button_state_struct;


//------------------------PRIVATE FUNCTIONS -> DO NOT USE------------------------------------
static uint8_t get_button_check_flag(void);
static inline void disable_button_sense(void);
static inline void enable_button_sense(void);


void button_init(){
	NRF_GPIO->PIN_CNF[GPIO_POWER_ON_OFF] =
					(GPIO_PIN_CNF_SENSE_High << GPIO_PIN_CNF_SENSE_Pos)
			      |	(GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos)
			      | (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos)
			      | (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos)
			      | (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos);

	NRF_GPIO->PIN_CNF[GPIO_USB_CONNECTED] =
					(GPIO_PIN_CNF_SENSE_Low << GPIO_PIN_CNF_SENSE_Pos)
				  | (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos)
				  | (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos)
			      | (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos)
			      | (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos);

	NRF_GPIOTE->EVENTS_PORT = 0;
	NRF_GPIOTE->INTENSET = GPIOTE_INTENSET_PORT_Set << GPIOTE_INTENSET_PORT_Pos;

	sd_nvic_SetPriority(GPIOTE_IRQn, APP_IRQ_PRIORITY_LOW);
	sd_nvic_EnableIRQ(GPIOTE_IRQn);
}

void button_pressed_irq_handler(void){
	button_state_struct.button_check_flag=1;
	timer_led_driver_init();
	disable_button_sense();
}

bool button_timer_irq_handler(){
	static uint8_t button_counter = 0;
#ifdef BUTTON_DOUBLE_CLICK_ON
	static uint8_t double_click_timer = 0;
	static uint8_t double_click_counter =0;
	//BUTTON LOGIC
	if(get_button_check_flag()){
		if(NRF_GPIO->IN & 1<<GPIO_POWER_ON_OFF){
			if(double_click_timer==0){
				if(button_counter<=10)
					button_counter++;
				if(button_counter==10) {//it should be 1s
					button_state_struct.button_pressed_flag=1;
				}

			}
			else
				double_click_counter++;

			return true;
		}
		else{
			if (button_counter<10){
				if (double_click_timer<=5){
					double_click_timer++;
					if(double_click_counter>=2){
						button_state_struct.button_double_clicked=true;
						button_counter=100;
					}
				}
				if(double_click_timer==5 ){
					if (button_counter>1)
						button_state_struct.button_single_clicked=true;
					button_counter=100;
				}
				return true;
			}
			else{
				button_state_struct.button_check_flag=0;
				//button_state_struct.diagnostic_unwanted_pulses++;
				button_counter=0;
				double_click_timer=0;
				double_click_counter=0;
				timer_led_driver_deinit();
				enable_button_sense();
				return false;
			}
		}
	}//END OF BUTTON LOGIC
#else
	if(get_button_check_flag())
	{
		if(NRF_GPIO->IN & 1<<GPIO_POWER_ON_OFF)
		{
			if(button_counter<=10)
				button_counter++;
			if(button_counter==10) //it should be 1s
				button_state_struct.button_pressed_flag=1;

			return true;
		}
		else
		{
			button_state_struct.button_check_flag=0;
			//button_state_struct.diagnostic_unwanted_pulses++;
			button_counter=0;
			timer_led_driver_deinit();
			enable_button_sense();
			return false;
		}
	}//END OF BUTTON LOGIC
#endif // BUTTON_DOUBLE_CLICK_ON

	return false;
}

bool button_is_pressed (void){
	return button_state_struct.button_pressed_flag!=0;
}

#ifdef BUTTON_DOUBLE_CLICK_ON
bool button_single_clicked(void){
	return button_state_struct.button_single_clicked;
}
bool button_double_clicked(void){
	return button_state_struct.button_double_clicked;
}
#endif // BUTTON_DOUBLE_CLICK_ON

void button_clear_state (void){
	button_state_struct.button_pressed_flag = false;

#ifdef BUTTON_DOUBLE_CLICK_ON
	button_state_struct.button_double_clicked = false;
	button_state_struct.button_single_clicked = false;
#endif // BUTTON_DOUBLE_CLICK_ON

}

#ifdef UART_CONTROLED_NEUROON_ON
void button_set_state (void){
	button_state_struct.button_pressed_flag = true;
}
#endif

//----------------------------DIAGNOSTIC---------------------------------
uint32_t get_unwanted_button_pulses(void){
	return button_state_struct.diagnostic_unwanted_pulses;
}
//----------------------------PRIVATE------------------------------
//static button_state_p get_button_state(void){
//	return &button_state_struct;
//}
static uint8_t get_button_check_flag(void){
	return button_state_struct.button_check_flag;
}
static void disable_button_sense(void){
	NRF_GPIO->PIN_CNF[GPIO_POWER_ON_OFF] =
					(GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)
			      |	(GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos)
			      | (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos)
			      | (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos)
			      | (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos);
}
static void enable_button_sense(void){
	NRF_GPIO->PIN_CNF[GPIO_POWER_ON_OFF] =
					(GPIO_PIN_CNF_SENSE_High << GPIO_PIN_CNF_SENSE_Pos)
			      |	(GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos)
			      | (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos)
			      | (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos)
			      | (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos);
}

