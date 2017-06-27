/**
 * @file
 * @brief electrode contact test methods
 */

#include "ic_electrotest.h"

#ifdef USE_IC_ELECTROTEST

/** GLOBAL VARIABLES */
static int64_t electrode_input[3];
static int64_t electrode_output[3];

static uint64_t electrode1_power = 0;
static uint64_t electrode2_power = 0;

static uint32_t adc_pin;

static uint8_t electrode_on_test;



static double gain = 0.01;//0.0001;//0.0000019121;

/**
 * @fn static electrotest_compute_power()
 * @brief compute signal power on electrotest pins using Butterworth filter
 * @return signal power
 */
static uint64_t electrotest_compute_power ()
{

/*
 * 	 [z p g] = butter(2, [1031/2083 1141/2083])
 *
   1024   2048   1024   1024    146    950
   1024  -2048   1024   1024     -1    950


	gain = 0.0014557
*/
	electrode_output[2] = electrode_output[1];
	electrode_output[1] = electrode_output[0];

	electrode_output[0] = electrode_input[0]*1024 + electrode_input[1]*2048 + electrode_input[2]*1024 - electrode_output[1]*146 - electrode_output[2]*950;
	electrode_output[0] /= 1024;

	electrode_output[2] = electrode_output[1];
	electrode_output[1] = electrode_output[0];

	electrode_output[0] = electrode_input[0]*1024 + electrode_input[1]*(-2048) + electrode_input[2]*1024 + electrode_output[1]*1 - electrode_output[2]*950;
	electrode_output[0] /= 1024;

	return (electrode_output[0] * gain) * (electrode_output[0] * gain);
}

/**
 * @fn electrotest_add_sample ()
 * @brief add signal sample to power filter
 * @param sample value
 */
static void electrotest_add_sample(int32_t sample)
{
	electrode_input[2] = electrode_input[1];
	electrode_input[1] = electrode_input[0];
	electrode_input[0] = sample;
}

/**
 * @fn static electrotest_clear ()
 * @brief clear all global variables
 */
/* zbędne po zmianach PB
static void electrotest_clear ()
{
	electrode_input[0] = electrode_input[1] = electrode_input[2] = 0;
	electrode_output[0] = electrode_output[1] = electrode_output[2] = 0;

	electrode1_power = 0;
	electrode2_power = 0;
}*/

/**
 * @fn static electrotest_set_pin ()
 * @brief set ADC channel depending on electrode pin
 * @param numer of ADC channel
 */
static void electrotest_set_pin(uint32_t adc_channel)
{
	adc_pin = adc_channel;
	NRF_ADC->CONFIG = (ADC_CONFIG_RES_10bit<<ADC_CONFIG_RES_Pos)
					| ((1 << adc_channel) << ADC_CONFIG_PSEL_Pos)
					| (ADC_CONFIG_INPSEL_AnalogInputOneThirdPrescaling<<ADC_CONFIG_INPSEL_Pos);
}
// numebr is 1 or 2, depends on electrode count


/**
 * @fn electrotest_init ()
 * @brief electrotest initialization (pins, timer, ADC, interrupt)
 */
void electrotest_init ()
{
	//electrotest_clear();

	uint8_t input_conf = (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos);

	NRF_GPIO->PIN_CNF[GPIO_TEST_EL1] = input_conf;
	NRF_GPIO->PIN_CNF[GPIO_TEST_EL2] = input_conf;

	TIM_ELECTROTEST->MODE = TIMER_MODE_MODE_Timer << TIMER_MODE_MODE_Pos;
	TIM_ELECTROTEST->PRESCALER = TIM_ELECTROTEST_PRESCALER;
	TIM_ELECTROTEST->BITMODE = TIMER_BITMODE_BITMODE_16Bit << TIMER_BITMODE_BITMODE_Pos;
	//2 eventy zostaną wygenerowane dla tego timera na 2 oddzielnych channelach
	TIM_ELECTROTEST->CC[TIM_ELECTROTEST_ADC_CHANNEL] = TIM_ELECTROTEST_ADC_CC_VALUE;
	TIM_ELECTROTEST->CC[TIM_ELECTROTEST_GEN_CHANNEL] = TIM_ELECTROTEST_GEN_CC_VALUE;
	TIM_ELECTROTEST->SHORTS = (1 << TIM_ELECTROTEST_GEN_CHANNEL);

	NRF_GPIOTE->CONFIG[GPIOTE_ELECTROTEST_GEN_CHANNEL] = (GPIOTE_CONFIG_MODE_Task << GPIOTE_CONFIG_MODE_Pos)
													   | (GPIO_GEN << GPIOTE_CONFIG_PSEL_Pos)
													   | (GPIOTE_CONFIG_POLARITY_Toggle << GPIOTE_CONFIG_POLARITY_Pos);

	sd_ppi_channel_assign(PPI_ELECTROTEST_GEN_CHANNEL,
			&TIM_ELECTROTEST->EVENTS_COMPARE[TIM_ELECTROTEST_GEN_CHANNEL], &NRF_GPIOTE->TASKS_OUT[GPIOTE_ELECTROTEST_GEN_CHANNEL]);

	sd_ppi_channel_assign(PPI_ELECTROTEST_ADC_CHANNEL1,
			&TIM_ELECTROTEST->EVENTS_COMPARE[TIM_ELECTROTEST_ADC_CHANNEL], &NRF_ADC->TASKS_START);

	sd_ppi_channel_assign(PPI_ELECTROTEST_ADC_CHANNEL2,
			&TIM_ELECTROTEST->EVENTS_COMPARE[TIM_ELECTROTEST_GEN_CHANNEL], &NRF_ADC->TASKS_START);

	sd_ppi_channel_enable_set(1 << PPI_ELECTROTEST_GEN_CHANNEL);

	sd_ppi_channel_enable_set(1 << PPI_ELECTROTEST_ADC_CHANNEL1);
	sd_ppi_channel_enable_set(1 << PPI_ELECTROTEST_ADC_CHANNEL2);

	NRF_ADC->INTENSET = ADC_INTENSET_END_Enabled << ADC_INTENSET_END_Pos;

	sd_nvic_SetPriority(ADC_IRQn, APP_IRQ_PRIORITY_HIGH);
	sd_nvic_EnableIRQ(ADC_IRQn);
}

/**
 * @fn electrotest_start ()
 * @brief start electrotest (start ADS and timer)
 */
void electrotest_start ()
{
	electrode_on_test=2;
	electrotest_set_pin(ELECTROTEST_E2_ADC_CHANNEL);
//	set_next_electrode_on_test();
	NRF_ADC->ENABLE = ADC_ENABLE_ENABLE_Enabled << ADC_ENABLE_ENABLE_Pos;
	TIM_ELECTROTEST->TASKS_START = 1;
}

void set_next_electrode_on_test(void)
{
	if(electrode_on_test==1)
	{
		electrode_on_test=2;
		electrotest_set_pin(ELECTROTEST_E2_ADC_CHANNEL);
	}
	else
	{
		electrode_on_test=1;
		electrotest_set_pin(ELECTROTEST_E1_ADC_CHANNEL);
	}
}
void set_differance_min_max_of_signal(uint16_t diff)
{
	if(electrode_on_test==1)
	{
		diff_p->electrode1=diff;
	}
	if(electrode_on_test==2)
	{
		diff_p->electrode2=diff;
	}
}
/**
 * @fn electrotest_stop ()
 * @brief stop electrotest (stop ADS and timer)
 */
void electrotest_stop ()
{
	TIM_ELECTROTEST->TASKS_STOP = 1;
	NRF_ADC->INTENCLR = ADC_INTENCLR_END_Enabled << ADC_INTENCLR_END_Pos;
	NRF_ADC->ENABLE = ADC_ENABLE_ENABLE_Disabled << ADC_ENABLE_ENABLE_Pos;
}

/**
 * @fn electrotest_deinit ()
 * @brief electrotest initialization
 */
void electrotest_deinit ()
{
	NRF_GPIOTE->CONFIG[GPIOTE_ELECTROTEST_GEN_CHANNEL] = (GPIOTE_CONFIG_MODE_Disabled << GPIOTE_CONFIG_MODE_Pos);
	sd_ppi_channel_enable_clr(1 << PPI_ELECTROTEST_GEN_CHANNEL);
	sd_ppi_channel_enable_clr(1 << PPI_ELECTROTEST_ADC_CHANNEL1);
	sd_ppi_channel_enable_clr(1 << PPI_ELECTROTEST_ADC_CHANNEL2);
	diff_p->electrode1=0;
	diff_p->electrode2=0;
}

/**
 * @fn electrotest_get_mean_power_e1 ()
 * @brief get mean power from electrode 1
 * @return electrode 1 power
 */
uint64_t electrotest_get_mean_power_e1 ()
{
	return electrode1_power;
}

/**
 * @fn electrotest_get_mean_power_e2 ()
 * @brief get mean power from electrode 2
 * @return electrode 2 power
 */
uint64_t electrotest_get_mean_power_e2 ()
{
	return electrode2_power;
}

/**
 * @fn electrotest_adc_rdy_handler ()
 * @brief handler to action after ADC conversion finish (calculate power; after ELECTROTEST_SAMPLE_COUNT/2 change electrode
 */
void electrotest_adc_rdy_handler (uint32_t values_left)
{
	electrotest_add_sample((int32_t)(NRF_ADC->RESULT));
	if(adc_pin == ELECTROTEST_E1_ADC_CHANNEL && (ELECTROTEST_SAMPLE_COUNT-values_left) > 100){
		electrode1_power += electrotest_compute_power();
	}
	else if (adc_pin == ELECTROTEST_E2_ADC_CHANNEL && (ELECTROTEST_SAMPLE_COUNT - values_left) > (100+ELECTROTEST_SAMPLE_COUNT/2)) {
		electrode2_power += electrotest_compute_power();
	}

	if(values_left == ELECTROTEST_SAMPLE_COUNT/2){
		electrotest_set_pin(ELECTROTEST_E2_ADC_CHANNEL);
		electrode_input[0] = electrode_input[1] = electrode_input[2] = 0;
		electrode_output[0] = electrode_output[1] = electrode_output[2] = 0;
	}
}

#endif
