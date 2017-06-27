/**
 * @file
 * @brief analog thermometer (PT1000) driver methods
 */

#include "ic_analog_thermometer.h"
#include "ic_cbuffer.h"

#ifdef USE_IC_ANALOG_THERMOMETER

#define THERM_FREQ 			125 	///frequency of reading data from thermometer (in timer cycles)
#define THERM_CBUFF_SIZE	5		///size of accelerometer cyclic buffer

static uint16_t	raw_buf1[THERM_CBUFF_SIZE];
static CBu16 *therm1_cbuf = &(CBu16)CBUFFER_INIT(raw_buf1, THERM_CBUFF_SIZE);
static uint16_t	raw_buf2[THERM_CBUFF_SIZE];
static CBu16 *therm2_cbuf = &(CBu16)CBUFFER_INIT(raw_buf2, THERM_CBUFF_SIZE);
static uint16_t therm_id = 0;
volatile uint8_t temp_flag, start_flag[2];


/**
 * @fn analog_thermometer_init ()
 * @brief analog thermometer initialization (GPIO and uC ADC configuration)
 */
void analog_thermometer_init ()
{
	uint8_t input_conf = (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos);

	NRF_GPIO->PIN_CNF[GPIO_ANALOG_T_RIGHT_PIN] = input_conf;
	NRF_GPIO->PIN_CNF[GPIO_ANALOG_T_LEFT_PIN] = input_conf;

	NRF_ADC->CONFIG = (ADC_CONFIG_RES_10bit << ADC_CONFIG_RES_Pos)
					| (ADC_CONFIG_INPSEL_AnalogInputOneThirdPrescaling << ADC_CONFIG_INPSEL_Pos)
					| (ADC_CONFIG_REFSEL_SupplyOneThirdPrescaling << ADC_CONFIG_REFSEL_Pos);

	NRF_ADC->ENABLE = ADC_ENABLE_ENABLE_Enabled << ADC_ENABLE_ENABLE_Pos;
	start_flag[0] = 0;
	start_flag[0] = 1;
	temp_flag = 0;
}

/**
 * @fn analog_thermometer_deinit ()
 * @brief analog thermometer deinitialization (disable uC ADC)
 */
void analog_thermometer_deinit ()
{
	NRF_ADC->CONFIG = 0;
	NRF_ADC->ENABLE = ADC_ENABLE_ENABLE_Disabled << ADC_ENABLE_ENABLE_Pos;
	start_flag[0] = 0;
	start_flag[0] = 0;
	temp_flag = 0;
}

/**
 * @fn analog_thermometer_select()
 * @brief select analog thermometer (right or left)
 * @param thermometer symbol
 */
void analog_thermometer_select (uint8_t therm_number)
{
	therm_id = (uint16_t)(therm_number << 14);
	if(therm_number == 1){
		NRF_ADC->CONFIG &= ~(ADC_CONFIG_PSEL_AnalogInput6 << ADC_CONFIG_PSEL_Pos);
		NRF_ADC->CONFIG |= (ADC_CONFIG_PSEL_AnalogInput5 << ADC_CONFIG_PSEL_Pos);
	}
	else if(therm_number == 2){
		NRF_ADC->CONFIG &= ~(ADC_CONFIG_PSEL_AnalogInput5 << ADC_CONFIG_PSEL_Pos);
		NRF_ADC->CONFIG |= (ADC_CONFIG_PSEL_AnalogInput6 << ADC_CONFIG_PSEL_Pos);
	}
	else
		return;
}

/**
 * @fn analog_thermometer_start ()
 * @brief analog thermometer start (start ADC conversion)
 */
void analog_thermometer_start ()
{
	if ((therm_id >> 14) == 1)
		start_flag[0] = 1;
	else if ((therm_id >> 14) == 2)
		start_flag[1] = 1;
	else
		return;

	NRF_ADC->TASKS_START = 1;
}

/**
 * @fn analog_thermometer_stop ()
 * @brief analog thermometer stop (stop ADC conversion)
 */
void analog_thermometer_stop ()
{
	start_flag[0] = 0;
	start_flag[1] = 1;
	NRF_ADC->TASKS_STOP = 1;
}

/**
 * @fn analog_thermometer_IRQHandler ()
 * @brief Handler to use in TIMER_IRQHandler function in ic_timers.c. Capturing data with period configurable via THERM_FREQ macro.
 */
void analog_thermometer_IRQHandler() {
	uint16_t result;

		//RIGHT THERMOMETER
		if ((start_flag[0] == 0) && (start_flag[1] != 0)) {
			analog_thermometer_select(1);
			analog_thermometer_start();
		} else if ((start_flag[0] != 0) && (start_flag[1] == 0)) { //LEFT THERMOMETER
			analog_thermometer_select(2);
			analog_thermometer_start();
		}
		temp_flag |= analog_thermometer_result(&result);

	//}
}

/**
 * @fn analog_thermometer_result ()
 * @brief read analog thermometer result (conversion result)
 * @param conversion result
 * @return 1 when new data is available on thermometer no. 1; 2 when new data is available on thermometer no. 2; 0 otherwise
 */
uint8_t analog_thermometer_result (uint16_t* result)
{
	if(NRF_ADC->BUSY)
		return 0;
	if(NRF_ADC->EVENTS_END){
		*result = (uint16_t)(NRF_ADC->RESULT);
		*result |= therm_id;
		*result &= 0x3FF;
		if ((therm_id >> 14) == 1) {
			cbu16_add(therm1_cbuf, *result);
			start_flag[1] = 0;
			return 1;
		} else if ((therm_id >> 14) == 2) {
			cbu16_add(therm2_cbuf, *result);
			start_flag[0] = 0;
			return 2;
		} else
			return 0;
	} else {
		*result = ANALOG_T_ERROR;
		return 0;
	}
}

/**
 * @fn analog_thermometer_getLastResult ()
 * @brief read analog thermometer result (conversion result) if up to date
 * @param therm_number select analog thermometer (right or left)
 * @param result pointer to variable with conversion result
 * @return 1 when new data is available, 0 otherwise
 */
uint8_t analog_thermometer_getLastResult(uint8_t therm_number, uint16_t* result) {
	uint8_t status_tmp = analog_thermometer_getResultStatus(therm_number);
	if (status_tmp == 0 || status_tmp == 0xFF)
		return 0;

	if(therm_number == 1) {
		*result = cbu16_sgetNewestVal(therm1_cbuf);
		temp_flag &= ~(1);
	} else if (therm_number == 2) {
		*result = cbu16_sgetNewestVal(therm2_cbuf);
		temp_flag &= ~(1 << 1);
	}

	return 1;
}

/**
 * @fn analog_thermometer_getAverage ()
 * @brief get average from last analog thermometer measurements (conversion results)
 * @param therm_number select analog thermometer (right or left)
 * @return average from last analog thermometer measurements or 0xFFFF when therm_number is wrong
 */
uint16_t analog_thermometer_getAverage(uint8_t therm_number) {
	uint16_t tmp_count;
	switch(therm_number) {
	case 1:
		tmp_count = cbu16_getCounter(therm1_cbuf);
		if (tmp_count > THERM_CBUFF_SIZE)
			tmp_count = THERM_CBUFF_SIZE;

		return (uint16_t)cbu16_getFastSum(therm1_cbuf)/tmp_count;

	case 2:
		tmp_count = cbu16_getCounter(therm2_cbuf);
		if (tmp_count > THERM_CBUFF_SIZE)
			tmp_count = THERM_CBUFF_SIZE;

		return (uint16_t)cbu16_getFastSum(therm2_cbuf)/tmp_count;

	default:
		return 0xFFFF;
	}
}

/**
 * @fn analog_thermometer_getResultStatus ()
 * @brief read analog thermometer result flag, then reset flag
 * @param therm_number thermometer symbol (flags of both when type 3)
 * @return 1 when new data is available, 0 when is unavailable, 0xFF when wrong therm_number typed. When both thermometers are checked, check 2 LSBs of returned value.
 */
uint8_t analog_thermometer_getResultStatus(uint8_t therm_number) {
	uint8_t temp_tmp_flag;
	switch(therm_number) {
	case 1:
		temp_tmp_flag = temp_flag & 1;
		break;
	case 2:
		temp_tmp_flag = temp_flag & (1 << 1);
		break;
	case 3:
		temp_tmp_flag = temp_flag;
		break;
	default:
		temp_tmp_flag = 0xFF;
	}
	return temp_tmp_flag;
}

#endif

