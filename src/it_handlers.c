/**
 * @file
 * @brief interrupt handler methods
 */

#include "it_handlers.h"


/**
 * @fn ADC_IRQHandler ()
 * @brief ADC interrupt handler (electrotest control)
 */
#define how_much_data_analyzed 5
volatile uint8_t licznik=0;
volatile uint32_t electrode_data[how_much_data_analyzed];
volatile uint16_t min=10000, max=0, difference=0;

void ADC_IRQHandler ()
{
	/*
	NRF_ADC->EVENTS_END = 0;
	ADC_config_p ADC_config = app_managers_get_adc_config();
	ADC_config->values_left--;
	if(ADC_config->values_left == 0) {
		TIM_ELECTROTEST->TASKS_STOP = 1;
		ADC_config->ADC_ready=1;
	}
	electrotest_adc_rdy_handler(ADC_config->values_left);
	*/

	NRF_ADC->EVENTS_END = 0;
	uint32_t result =NRF_ADC->RESULT;
	if(licznik!=how_much_data_analyzed)
	{
		electrode_data[licznik++]=result;
		if(result<min)
			min=result;
		if(result>max)
			max=result;
	}
	else
	{
		licznik=0;
		difference=max-min;
		set_differance_min_max_of_signal(difference);
		set_next_electrode_on_test();
		min=10000;
		max=0;
	}
	WDT_RR();
}

/**
 * @fn GPIOTE_IRQHandler ()
 * @brief GPIOTE interrupt handler (on/off switch, usb charger)
 */
void GPIOTE_IRQHandler()
 {
	uint32_t port_state = NRF_GPIO->IN;
	//uint16_t switch_counter = 0;
	NRF_GPIOTE->EVENTS_PORT = 0;


//	//----------------------PRZERWANIE OD AKCELEROMETRU/ACCELEROMETER IRQ------------------------------
//	if(port_state & 1<<GPIO_INT1_ACC_PIN) {
//		app_managers_get_app_config()->acc_irq_flag=true;
//	}
//	if(port_state & 1<<GPIO_INT1_ACC_PIN){
//	//-----------------------wYŚWIETL GODZINĘ PO PODWÓJNYM PUKNIĘCIU W MASKĘ--------------------------------
//		lis3dh_double_click_irq_handler();
//		return;
//	}
	//----------------------PRZERWANIE OD PRZYCISKU/BUTTON IRQ------------------------------
	if(port_state & 1<<GPIO_POWER_ON_OFF) {
		button_pressed_irq_handler();
	}
	//-----------------------PRZERWANIE OD ŁADOWARKI/CHARGING INTERRUPT--------------------------------
	if(!(port_state & 1<<GPIO_USB_CONNECTED)) {
	//-----------------------ŁADOWARKA PODŁĄCZONA/CHARGER ENABLED--------------------------------
		charger_plugged_irq_handler();
	}
	if(port_state & 1<<GPIO_USB_CONNECTED){
	//-----------------------ŁADOWARKA ODŁĄCZONA/CHARGER DISABLED--------------------------------
		charger_unplugged_irq_handler();
	}

}
