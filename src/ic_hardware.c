//---------------------------------------------------------------------------------------------------
#include <ic_hardware.h>
#include <ic_lib_config.h>
//---------------------------------------------------------------------------------------------------
void hardware_clockInit() {
	NRF_CLOCK->LFCLKSRC = CLOCK_LFCLKSRC_SRC_Xtal << CLOCK_LFCLKSRC_SRC_Pos;
	NRF_CLOCK->TASKS_LFCLKSTART = 1;
	while(NRF_CLOCK->EVENTS_LFCLKSTARTED != 1)
		__NOP();
	NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;

	NRF_CLOCK->TASKS_HFCLKSTART = 1;
	while(NRF_CLOCK->EVENTS_HFCLKSTARTED == 0)
		__NOP();
	NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
}
//---------------------------------------------------------------------------------------------------
void hardware_gpioInit() {
	uint8_t output_conf = (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)
                        | (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos)
                        | (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos)
                        | (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos)
                        | (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos);

	uint8_t input_conf = GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos;

	NRF_GPIO->PIN_CNF[GPIO_AFE_DIAG_END] = input_conf;
	NRF_GPIO->PIN_CNF[GPIO_AFE_LED_ALM] = input_conf;
	NRF_GPIO->PIN_CNF[GPIO_AFE_PD_ALM] = input_conf;
	NRF_GPIO->PIN_CNF[GPIO_AFE_ADC_RDY] = input_conf;

	NRF_GPIO->PIN_CNF[GPIO_AFE_RESET] = output_conf;
	NRF_GPIO->PIN_CNF[GPIO_AFE_PDN] = output_conf;
	NRF_GPIO->PIN_CNF[GPIO_POWER_ANALOG] = output_conf;
	NRF_GPIO->PIN_CNF[GPIO_POWER_DIGITAL] = output_conf;
	NRF_GPIO->PIN_CNF[GPIO_LEDS_ON] = output_conf;
	NRF_GPIO->PIN_CNF[GPIO_2MS_DRIVER_ON] = output_conf;

	ResetBit(GPIO_POWER_DIGITAL);
	ResetBit(GPIO_POWER_ANALOG);

	SetBit(GPIO_AFE_RESET);
	ResetBit(GPIO_AFE_PDN);
	ResetBit(GPIO_2MS_DRIVER_ON);
}
//---------------------------------------------------------------------------------------------------
void hardware_enableIrqGPIOTE(){
	NRF_GPIOTE->EVENTS_PORT = 0;
	NRF_GPIOTE->INTENSET = GPIOTE_INTENSET_PORT_Set << GPIOTE_INTENSET_PORT_Pos;

	NVIC_SetPriority(GPIOTE_IRQn, APP_IRQ_PRIORITY_LOW);
	NVIC_EnableIRQ(GPIOTE_IRQn);
}
//---------------------------------------------------------------------------------------------------
