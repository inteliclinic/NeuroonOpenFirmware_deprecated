/**
 * @file    ic_afe4400.c
 * @Author  Wojciech Węclewski <w.weclewski@inteliclinic.com>
 * @date    February, 2017
 * @brief   AFE4400 driver header
 *
 * Description
 */

#include "ic_afe4400.h"

#ifdef USE_IC_AFE4400

#ifndef USE_IC_DELAY
	#define USE_IC_DELAY
#endif
#ifndef USE_IC_SPI
	#define USE_IC_SPI
#endif

#include "ic_delay.h"
#include "ic_spi.h"

/** @defgroup AFE4400_REG_CONF global variables with current AFE4400 registers configuration
 *
 * @{
 */
t_afe4400RegisterConf control1_r;
t_afe4400RegisterConf tia_amb_gain_r;
t_afe4400RegisterConf ledcntrl_r;
t_afe4400RegisterConf control2_r;
t_afe4400RegisterConf alarm_r;
t_afe4400RegisterConf diag_r;

t_afe4400RegisterConf prpcount_r;

t_afe4400RegisterConf led2stc_r;
t_afe4400RegisterConf led2endc_r;
t_afe4400RegisterConf led2ledstc_r;
t_afe4400RegisterConf led2ledendc_r;
t_afe4400RegisterConf aled2stc_r;
t_afe4400RegisterConf aled2endc_r;

t_afe4400RegisterConf led1stc_r;
t_afe4400RegisterConf led1endc_r;
t_afe4400RegisterConf led1ledstc_r;
t_afe4400RegisterConf led1ledendc_r;
t_afe4400RegisterConf aled1stc_r;
t_afe4400RegisterConf aled1endc_r;

t_afe4400RegisterConf led2convst_r;
t_afe4400RegisterConf led2convend_r;
t_afe4400RegisterConf aled2convst_r;
t_afe4400RegisterConf aled2convend_r;

t_afe4400RegisterConf led1convst_r;
t_afe4400RegisterConf led1convend_r;
t_afe4400RegisterConf aled1convst_r;
t_afe4400RegisterConf aled1convend_r;

t_afe4400RegisterConf adcrststct0_r;
t_afe4400RegisterConf adcrstendct0_r;

t_afe4400RegisterConf adcrststct1_r;
t_afe4400RegisterConf adcrstendct1_r;

t_afe4400RegisterConf adcrststct2_r;
t_afe4400RegisterConf adcrstendct2_r;

t_afe4400RegisterConf adcrststct3_r;
t_afe4400RegisterConf adcrstendct3_r;
/** @} */ //end of AFE4400_REG_CONF

/** global auxiliary variables */
afe4400_config_t afe_config; /*OBSOLETE*/
uint32_t ledcntrl_reg_config;/*OBSOLETE*/
uint32_t tia_amb_gain_reg_config;/*OBSOLETE*/

/**
 * @fn afe4400_TakeData ()
 * @brief read data from device
 * @param pointer to data variables
 */
void afe4400_TakeData(int32_t *data)
{
	uint32_t led2_value = afe4400_ReadReg(LED2_ALED2VAL);
	uint32_t led1_value = afe4400_ReadReg(LED1_ALED1VAL);
	if (led2_value & 0x200000)
		data[0] = (int32_t)(led2_value)|0xFFC00000;
	else
		data[0] = (int32_t)(led2_value);
	if (led1_value & 0x200000)
		data[1] = (int32_t)(led1_value)|0xFFC00000;
	else
		data[1] = (int32_t)(led1_value);
}

void afe4400_TakeData_LED1(int32_t *data)
{
	//uint32_t led2_value = afe4400_ReadReg(LED2_ALED2VAL);
	uint32_t led1_value = afe4400_ReadReg(LED1_ALED1VAL);

	if (led1_value & 0x200000)
		data[0] = (int32_t)(led1_value)|0xFFC00000;
	else
		data[0] = (int32_t)(led1_value);
}

void afe4400_TakeData_LED2(int32_t *data)
{
	uint32_t led2_value = afe4400_ReadReg(LED2_ALED2VAL);
	//uint32_t led1_value = afe4400_ReadReg(LED1_ALED1VAL);
	if (led2_value & 0x200000)
		data[0] = (int32_t)(led2_value)|0xFFC00000;
	else
		data[0] = (int32_t)(led2_value);
}

/**
 * @fn afe4400_ReadReg ()
 * @brief read register value THIS FUNCTION SHOULD BE PRIVATE (STATIC)
 * @param register number
 * @return register value
 */
uint32_t afe4400_ReadReg(uint8_t reg)
{
	uint8_t read_control_value [3] = {0x00, 0x00, SPI_READ};
	uint8_t read_value [3] = {0};
	uint32_t value = 0;
	spi_master_cs_low (GPIO_AFE_CS);

	spi_multipleWrite_data (CONTROL0, read_control_value, 3);

	spi_multipleRead_data (reg, read_value, 3);
	value  = (uint32_t) read_value [0] << 16;
	value |= (uint32_t) read_value [1] << 8;
	value |= (uint32_t) read_value [2];

	spi_master_cs_high (GPIO_AFE_CS);

	return value;
}

/**
 * @fn afe4400_WriteReg ()
 * @brief write value to register  THIS FUNCTION SHOULD BE PRIVATE (STATIC)
 * @param register number
 * @param value to write
 */
void afe4400_WriteReg(uint8_t reg, uint32_t data)
{
	uint8_t read_control_value [3] = {0x00, 0x00, SPI_WRITE};
	uint8_t write_value [3] = {0};
	write_value [0] = (uint8_t) (data >> 16);
	write_value [1] = (uint8_t) (data >> 8);
	write_value [2] = (uint8_t) data;
	spi_master_cs_low (GPIO_AFE_CS);

	spi_multipleWrite_data (CONTROL0, read_control_value, 3);

	spi_multipleWrite_data (reg, write_value, 3);

	spi_master_cs_high (GPIO_AFE_CS);
}

/**
 * @fn afe4400_init ()
 * @brief initialization device THIS FUNCTION IS OBSOLETE. DO NOT USE.
 */
void afe4400_init ()
{
	ledcntrl_reg_config = 0x010000 | ((afe4400_get_config()->led1_current)<<8) | (afe4400_get_config()->led2_current);
	tia_amb_gain_reg_config = 0x004000 | ((afe4400_get_config()->gain)<<8) | ((afe4400_get_config()->cf_led)<<3) | (afe4400_get_config()->rf_led);

	spi_master_cs_init(GPIO_AFE_CS);
	spi_master_cs_high(GPIO_AFE_CS);

	afe4400_WriteReg(CONTROL0, DIAG_EN);
	afe4400_WriteReg(CONTROL1, TIMEREN);

	afe4400_WriteReg(ALARM, ALMPINCLKEN);
	afe4400_WriteReg(LED2STC, LED2STC_STD_VAL);
	afe4400_WriteReg(LED2ENDC, LED2ENDC_STD_VAL);
	afe4400_WriteReg(LED2LEDSTC, LED2LEDSTC_STD_VAL);
	afe4400_WriteReg(LED2LEDENDC, LED2LEDENDC_STD_VAL);
	afe4400_WriteReg(ALED2STC, ALED2STC_STD_VAL);
	afe4400_WriteReg(ALED2ENDC, ALED2ENDC_STD_VAL);
	afe4400_WriteReg(LED1STC, LED1STC_STD_VAL);
	afe4400_WriteReg(LED1ENDC, LED1ENDC_STD_VAL);
	afe4400_WriteReg(LED1LEDSTC, LED1LEDSTC_STD_VAL);
	afe4400_WriteReg(LED1LEDENDC, LED1LEDENDC_STD_VAL);
	afe4400_WriteReg(ALED1STC, ALED1STC_STD_VAL);
	afe4400_WriteReg(ALED1ENDC, ALED1ENDC_STD_VAL);
	afe4400_WriteReg(LED2CONVST, LED2CONVST_STD_VAL);
	afe4400_WriteReg(LED2CONVEND, LED2CONVEND_STD_VAL);
	afe4400_WriteReg(ALED2CONVST, ALED2CONVST_STD_VAL);
	afe4400_WriteReg(ALED2CONVEND, ALED2CONVEND_STD_VAL);
	afe4400_WriteReg(LED1CONVST, LED1CONVST_STD_VAL);
	afe4400_WriteReg(LED1CONVEND, LED1CONVEND_STD_VAL);
	afe4400_WriteReg(ALED1CONVST, ALED1CONVST_STD_VAL);
	afe4400_WriteReg(ALED1CONVEND, ALED1CONVEND_STD_VAL);
	afe4400_WriteReg(ADCRSTSTCT0, ADCRSTSTCT0_STD_VAL);
	afe4400_WriteReg(ADCRSTENDCT0, ADCRSTENDCT0_STD_VAL);
	afe4400_WriteReg(ADCRSTSTCT1, ADCRSTSTCT1_STD_VAL);
	afe4400_WriteReg(ADCRSTENDCT1, ADCRSTENDCT1_STD_VAL);
	afe4400_WriteReg(ADCRSTSTCT2, ADCRSTSTCT2_STD_VAL);
	afe4400_WriteReg(ADCRSTENDCT2, ADCRSTENDCT2_STD_VAL);
	afe4400_WriteReg(ADCRSTSTCT3, ADCRSTSTCT3_STD_VAL);
	afe4400_WriteReg(ADCRSTENDCT3, ADCRSTENDCT3_STD_VAL);
	afe4400_WriteReg(PRPCOUNT, PRPCOUNT_STD_VAL);

	/** regulacja prądu diody 0x 00 LED1 LED2
	 * LED current = ((LED1 lub LED2)/256)*50mA
	 * */
	afe4400_WriteReg(LEDCNTRL, ledcntrl_reg_config);

	/** regulacja wzmocnienia i wartosci Rf, Cf 0x 00 00 (CF_LED2[7:3] RF_LED[2:0])
	* 8 - 500k
	* 9 - 250k
	* A - 100k
	* B - 50k
	* C - 25k
	* D - 10k
	* E - 1M
	* */
	afe4400_WriteReg(TIA_AMB_GAIN, tia_amb_gain_reg_config);
}

void afe4400_hdw_init(void){
  spi_master_cs_init(GPIO_AFE_CS);
  spi_master_cs_high(GPIO_AFE_CS);

  afe4400_set_control2_r(PDN_AFE_OFF + PDN_RX_OFF + PDN_TX_OFF);
  afe4400_set_control0_r(DIAG_EN);
  afe4400_set_control1_r(SAMPLE_LED2_SAMPLE_LED1 + TIMEREN);
  afe4400_set_alarm_r(ALMPINCLKEN);
}

void afe4400_std_val_init(void){
 afe4400_set_selected_timer_module_r(LED2STC, LED2STC_STD_VAL);
 afe4400_set_selected_timer_module_r(LED2ENDC, LED2ENDC_STD_VAL);
 afe4400_set_selected_timer_module_r(LED2LEDSTC, LED2LEDSTC_STD_VAL);
 afe4400_set_selected_timer_module_r(LED2LEDENDC, LED2LEDENDC_STD_VAL);
 afe4400_set_selected_timer_module_r(ALED2STC, ALED2STC_STD_VAL);
 afe4400_set_selected_timer_module_r(ALED2ENDC, ALED2ENDC_STD_VAL);
 afe4400_set_selected_timer_module_r(LED1STC, LED1STC_STD_VAL);
 afe4400_set_selected_timer_module_r(LED1ENDC, LED1ENDC_STD_VAL);
 afe4400_set_selected_timer_module_r(LED1LEDSTC, LED1LEDSTC_STD_VAL);
 afe4400_set_selected_timer_module_r(LED1LEDENDC, LED1LEDENDC_STD_VAL);
 afe4400_set_selected_timer_module_r(ALED1STC, ALED1STC_STD_VAL);
 afe4400_set_selected_timer_module_r(ALED1ENDC, ALED1ENDC_STD_VAL);
 afe4400_set_selected_timer_module_r(LED2CONVST, LED2CONVST_STD_VAL);
 afe4400_set_selected_timer_module_r(LED2CONVEND, LED2CONVEND_STD_VAL);
 afe4400_set_selected_timer_module_r(ALED2CONVST, ALED2CONVST_STD_VAL);
 afe4400_set_selected_timer_module_r(ALED2CONVEND, ALED2CONVEND_STD_VAL);
 afe4400_set_selected_timer_module_r(LED1CONVST, LED1CONVST_STD_VAL);
 afe4400_set_selected_timer_module_r(LED1CONVEND, LED1CONVEND_STD_VAL);
 afe4400_set_selected_timer_module_r(ALED1CONVST, ALED1CONVST_STD_VAL);
 afe4400_set_selected_timer_module_r(ALED1CONVEND, ALED1CONVEND_STD_VAL);
 afe4400_set_selected_timer_module_r(ADCRSTSTCT0, ADCRSTSTCT0_STD_VAL);
 afe4400_set_selected_timer_module_r(ADCRSTENDCT0, ADCRSTENDCT0_STD_VAL);
 afe4400_set_selected_timer_module_r(ADCRSTSTCT1, ADCRSTSTCT1_STD_VAL);
 afe4400_set_selected_timer_module_r(ADCRSTENDCT1, ADCRSTENDCT1_STD_VAL);
 afe4400_set_selected_timer_module_r(ADCRSTSTCT2, ADCRSTSTCT2_STD_VAL);
 afe4400_set_selected_timer_module_r(ADCRSTENDCT2, ADCRSTENDCT2_STD_VAL);
 afe4400_set_selected_timer_module_r(ADCRSTSTCT3, ADCRSTSTCT3_STD_VAL);
 afe4400_set_selected_timer_module_r(ADCRSTENDCT3, ADCRSTENDCT3_STD_VAL);
 afe4400_set_selected_timer_module_r(PRPCOUNT, PRPCOUNT_STD_VAL);
 afe4400_set_ledcntrl_r(LEDCNTRL_STD_VAL);
 afe4400_set_tia_amb_gain_r(TIA_AMB_GAIN_STD_VAL);
}

/**
 * @brief Turn AFE4400 power OFF
 */
void afe4400_powerdown_on(void){
  afe4400_set_control2_r(PDN_AFE_ON);
}

/**
 * @brief Turn AFE4400 power ON
 */
void afe4400_powerdown_off(void){
  afe4400_set_control2_r(PDN_AFE_OFF+PDN_RX_OFF+PDN_TX_OFF);
}

/**
 * @fn afe4400_start ()
 * @brief start device THIS FUNCTION IS OBSOLETE. DO NOT USE.
 */
void afe4400_start ()
{
	afe4400_WriteReg(CONTROL2, PDN_AFE_OFF);
}

/**
 * @fn afe4400_stop ()
 * @brief stop device THIS FUNCTION IS OBSOLETE. DO NOT USE.
 */
void afe4400_stop ()
{
	afe4400_WriteReg(CONTROL2, PDN_AFE_ON);
}

/**
 * @fn afe4400_reset ()
 * @brief reset device THIS FUNCTION IS OBSOLETE. DO NOT USE.
 */
void afe4400_reset ()
{
	afe4400_WriteReg(CONTROL0, SW_RST);
}

/** @defgroup AFE4400_REG_CONF_FUNC global variables with current AFE4400 registers configuration
 *
 * @{
 */
void afe4400_set_control0_r(t_afe4400RegisterConf reg_val){
  afe4400_WriteReg(CONTROL0, CONTROL0_VAL + reg_val);
}

void afe4400_set_control1_r(t_afe4400RegisterConf reg_val){
  control1_r = reg_val;
  afe4400_WriteReg(CONTROL1, CONTROL1_VAL + reg_val);
}

void afe4400_set_tia_amb_gain_r(t_afe4400RegisterConf reg_val){
  tia_amb_gain_r = reg_val;
  afe4400_WriteReg(TIA_AMB_GAIN, TIA_AMB_GAIN_VAL + reg_val);
}

void afe4400_set_ledcntrl_r(t_afe4400RegisterConf reg_val){
  ledcntrl_r = reg_val;
  afe4400_WriteReg(LEDCNTRL, LEDCNTRL_VAL + reg_val);
}

void afe4400_set_control2_r(t_afe4400RegisterConf reg_val){
  control2_r = reg_val;
  afe4400_WriteReg(CONTROL2, CONTROL2_VAL + reg_val);
}

void afe4400_set_alarm_r(t_afe4400RegisterConf reg_val){
  alarm_r = reg_val;
  afe4400_WriteReg(ALARM, ALARM_VAL + reg_val);
}

t_afe4400RegisterConf afe4400_get_selected_r(t_afe4400Register reg){
  switch(reg) {
    case CONTROL1:      return control1_r;
    case TIA_AMB_GAIN:  return tia_amb_gain_r;
    case LEDCNTRL:      return ledcntrl_r;
    case CONTROL2:      return control2_r;
    case ALARM:         return alarm_r;
    case DIAG:          return diag_r;
    case LED2STC:       return led2stc_r;
    case LED2ENDC:      return led2endc_r;
    case LED2LEDSTC:    return led2ledstc_r;
    case LED2LEDENDC:   return led2ledendc_r;
    case ALED2STC:      return aled2stc_r;
    case ALED2ENDC:     return aled2endc_r;
    case LED1STC:       return led1stc_r;
    case LED1ENDC:      return led1endc_r;
    case LED1LEDSTC:    return led1ledstc_r;
    case LED1LEDENDC:   return led1ledendc_r;
    case ALED1STC:      return aled1stc_r;
    case ALED1ENDC:     return aled1endc_r;
    case LED2CONVST:    return led2convst_r;
    case LED2CONVEND:   return led2convend_r;
    case ALED2CONVST:   return aled2convst_r;
    case ALED2CONVEND:  return aled2convend_r;
    case LED1CONVST:    return led1convst_r;
    case LED1CONVEND:   return led1convend_r;
    case ALED1CONVST:   return aled1convst_r;
    case ALED1CONVEND:  return aled1convend_r;
    case ADCRSTSTCT0:   return adcrststct0_r;
    case ADCRSTENDCT0:  return adcrstendct0_r;
    case ADCRSTSTCT1:   return adcrststct1_r;
    case ADCRSTENDCT1:  return adcrstendct1_r;
    case ADCRSTSTCT2:   return adcrststct2_r;
    case ADCRSTENDCT2:  return adcrstendct2_r;
    case ADCRSTSTCT3:   return adcrststct3_r;
    case ADCRSTENDCT3:  return adcrstendct3_r;
    case PRPCOUNT:      return prpcount_r;
  }
  return 0xFFFFUL;
}
void afe4400_set_selected_timer_module_r(t_afe4400Register reg, t_afe4400RegisterConf reg_val){
  switch(reg) {
    case LED2STC:       led2stc_r = reg_val; break;
    case LED2ENDC:      led2endc_r = reg_val; break;
    case LED2LEDSTC:    led2ledstc_r = reg_val; break;
    case LED2LEDENDC:   led2ledendc_r = reg_val; break;
    case ALED2STC:      aled2stc_r = reg_val; break;
    case ALED2ENDC:     aled2endc_r = reg_val; break;
    case LED1STC:       led1stc_r = reg_val; break;
    case LED1ENDC:      led1endc_r = reg_val; break;
    case LED1LEDSTC:    led1ledstc_r = reg_val; break;
    case LED1LEDENDC:   led1ledendc_r = reg_val; break;
    case ALED1STC:      aled1stc_r = reg_val; break;
    case ALED1ENDC:     aled1endc_r = reg_val; break;
    case LED2CONVST:    led2convst_r = reg_val; break;
    case LED2CONVEND:   led2convend_r = reg_val; break;
    case ALED2CONVST:   aled2convst_r = reg_val; break;
    case ALED2CONVEND:  aled2convend_r = reg_val; break;
    case LED1CONVST:    led1convst_r = reg_val; break;
    case LED1CONVEND:   led1convend_r = reg_val; break;
    case ALED1CONVST:   aled1convst_r = reg_val; break;
    case ALED1CONVEND:  aled1convend_r = reg_val; break;
    case ADCRSTSTCT0:   adcrststct0_r = reg_val; break;
    case ADCRSTENDCT0:  adcrstendct0_r = reg_val; break;
    case ADCRSTSTCT1:   adcrststct1_r = reg_val; break;
    case ADCRSTENDCT1:  adcrstendct1_r = reg_val; break;
    case ADCRSTSTCT2:   adcrststct2_r = reg_val; break;
    case ADCRSTENDCT2:  adcrstendct2_r = reg_val; break;
    case ADCRSTSTCT3:   adcrststct3_r = reg_val; break;
    case ADCRSTENDCT3:  adcrstendct3_r = reg_val; break;
    case PRPCOUNT:      prpcount_r = reg_val; break;
    default:
      return;
  }
  afe4400_WriteReg(reg, reg_val);
}
void afe4400_read_selected_r(t_afe4400Register reg){
  t_afe4400RegisterConf reg_val = afe4400_ReadReg(reg);
  switch(reg) {
    case CONTROL1:      control1_r = reg_val; break;
    case TIA_AMB_GAIN:  tia_amb_gain_r = reg_val; break;
    case LEDCNTRL:      ledcntrl_r = reg_val; break;
    case CONTROL2:      control2_r = reg_val; break;
    case ALARM:         alarm_r = reg_val; break;
    case DIAG:          diag_r = reg_val; break;
    case LED2STC:       led2stc_r = reg_val; break;
    case LED2ENDC:      led2endc_r = reg_val; break;
    case LED2LEDSTC:    led2ledstc_r = reg_val; break;
    case LED2LEDENDC:   led2ledendc_r = reg_val; break;
    case ALED2STC:      aled2stc_r = reg_val; break;
    case ALED2ENDC:     aled2endc_r = reg_val; break;
    case LED1STC:       led1stc_r = reg_val; break;
    case LED1ENDC:      led1endc_r = reg_val; break;
    case LED1LEDSTC:    led1ledstc_r = reg_val; break;
    case LED1LEDENDC:   led1ledendc_r = reg_val; break;
    case ALED1STC:      aled1stc_r = reg_val; break;
    case ALED1ENDC:     aled1endc_r = reg_val; break;
    case LED2CONVST:    led2convst_r = reg_val; break;
    case LED2CONVEND:   led2convend_r = reg_val; break;
    case ALED2CONVST:   aled2convst_r = reg_val; break;
    case ALED2CONVEND:  aled2convend_r = reg_val; break;
    case LED1CONVST:    led1convst_r = reg_val; break;
    case LED1CONVEND:   led1convend_r = reg_val; break;
    case ALED1CONVST:   aled1convst_r = reg_val; break;
    case ALED1CONVEND:  aled1convend_r = reg_val; break;
    case ADCRSTSTCT0:   adcrststct0_r = reg_val; break;
    case ADCRSTENDCT0:  adcrstendct0_r = reg_val; break;
    case ADCRSTSTCT1:   adcrststct1_r = reg_val; break;
    case ADCRSTENDCT1:  adcrstendct1_r = reg_val; break;
    case ADCRSTSTCT2:   adcrststct2_r = reg_val; break;
    case ADCRSTENDCT2:  adcrstendct2_r = reg_val; break;
    case ADCRSTSTCT3:   adcrststct3_r = reg_val; break;
    case ADCRSTENDCT3:  adcrstendct3_r = reg_val; break;
    case PRPCOUNT:      prpcount_r = reg_val; break;
  }
}
/** @} */ //end of AFE4400_REGS_CONF

/**
 * @fn afe4400_red_off ()
 * @brief turn off red LED. DIRTY OLD HACK
 */
void afe4400_red_off () {
	ledcntrl_reg_config &= ~(0xFF<<8);
	afe4400_WriteReg(LED1STC, 0x0000);
	afe4400_WriteReg(LED1ENDC, 0x0000);
	afe4400_WriteReg(LED1LEDSTC, 0x0000);
	afe4400_WriteReg(LED1LEDENDC, 0x0000);
	/** regulacja prądu diody 0x 00 LED1 LED2
	 * LED current = ((LED1 lub LED2)/256)*50mA
	 * */
	afe4400_WriteReg(LEDCNTRL, ledcntrl_reg_config);
}
/**
 * @fn afe4400_red_on ()
 * @brief turn on red LED. DIRTY OLD HACK
 */
void afe4400_red_on () {
	ledcntrl_reg_config |= ((afe4400_get_config()->led1_current)<<8);
	afe4400_WriteReg(LED1STC, LED1STC_STD_VAL);
	afe4400_WriteReg(LED1ENDC, LED1ENDC_STD_VAL);
	afe4400_WriteReg(LED1LEDSTC, LED1LEDSTC_STD_VAL);
	afe4400_WriteReg(LED1LEDENDC, LED1LEDENDC_STD_VAL);
	/** regulacja prądu diody 0x 00 LED1 LED2
	 * LED current = ((LED1 lub LED2)/256)*50mA
	 * */
	afe4400_WriteReg(LEDCNTRL, ledcntrl_reg_config);
}
/**
* @fn afe4400_get_config ()
* @brief Getter to afe4400 config structure THIS FUNCTION IS OBSOLETE. DO NOT USE.
* @return afe config structure pointer
*/
afe4400_config_p afe4400_get_config ()
{
	return &afe_config;
}

int afe4400_SelfTest(){
	spi_master_cs_init(GPIO_AFE_CS);
	spi_master_cs_high(GPIO_AFE_CS);

	afe4400_set_control0_r(DIAG_EN);
	ic_delay_ms(20);

	afe4400_read_selected_r(DIAG);
	return diag_r;

}
#endif
