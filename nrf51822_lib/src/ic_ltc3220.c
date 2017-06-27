/**
 * @file
 * @brief LTC3220 LEDs driver methods
 */

#include "ic_ltc3220.h"
#include "ic_ltc_async_sender.h"
#include "ic_timers.h"

#define NO_OF_REFRESHED_CHANNELS 6
static uint16_t ltc_buf[NO_OF_REFRESHED_CHANNELS];

/**
 * @fn ltc3220_init ()
 * @brief initialization device
 * @return 0 if operation correct
 */
uint8_t ltc3220_init ()
{
	uint8_t value = LTC3220_NORMAL_MODE;
	SetBit(GPIO_LEDS_ON);
	uint8_t error = TWI_WriteReg(LTC3220_TWI_ADDRESS, LTC3220_COMMAND_REG, &value, 1);
	return error;
}

/**
 * @fn ltc3220_shutdown ()
 * @brief shutdown device
 * @return 0 if operation correct
 */
uint8_t ltc3220_shutdown ()
{
	uint8_t value = LTC3220_SHUTDOWN;
	uint8_t error = TWI_WriteReg(LTC3220_TWI_ADDRESS, LTC3220_COMMAND_REG, &value, 1);
	return error;
}

/**
 * @fn ltc3220_set_chargepump ()
 * @brief force charge pump value
 * @param charge pump mode
 * @return 0 if operation correct
 */
uint8_t ltc3220_set_chargepump (uint8_t mode)
{
	uint8_t value;
	if(mode == 1) //1x mode
		value = LTC3220_FORCE1p5 | LTC3220_FORCE2;
	else if (mode == 2) //2x mode
		value = LTC3220_FORCE2;
	else if(mode == 3) //1.5x mode
		value = LTC3220_FORCE1p5;

	uint8_t error = TWI_WriteReg(LTC3220_TWI_ADDRESS, LTC3220_COMMAND_REG, &value, 1);

	return error;
}

/**
 * @fn ltc3220_serial_write ()
 * @brief serial write to each register enable (REG1 data is written to all 18 ULED register)
 * @return 0 if operation correct
 */
uint8_t ltc3220_serial_write ()
{
	uint8_t value = LTC3220_QUICK_WRITE;
	uint8_t error = TWI_WriteReg(LTC3220_TWI_ADDRESS, LTC3220_COMMAND_REG, &value, 1);
	return error;
}

/**
 * @fn ltc3220_set_brightness ()
 * @brief set specified ULED on brightness
 * @param ULED number
 * @param level of brightness
 * @return 0 if operation correct
 */
uint8_t ltc3220_set_brightness(uint8_t led, uint8_t level) /*TODO ta funkcja ma nazwę z dupy, bo dotyczy kanałów LTC, a nie LEDów*/
{
	if(led > 18){
			return 1;
	}
	uint8_t led_level = level&0x3F;
	uint8_t error = TWI_WriteReg(LTC3220_TWI_ADDRESS, led, &led_level, 1);
	return error;
}



/**
 * @fn ltc3220_clear ()
 * @brief off all ULED (high impedance)
 * @return 0 if operation correct
 */
void ltc3220_clear ()
{
	ResetBit(GPIO_LEDS_ON);
	SetBit(GPIO_LEDS_ON);

//	uint8_t i =1;
//	for (;i<19;i++){
//		ltc3220_set_brightness(i, LTC3220_LED_OFF);
//	}
}

/**Asynchronus LTC refreshing*/
static void add_to_ltc_buff(uint8_t led, uint8_t level) {
  switch (led) {
  case RGB1_R_CHANNEL:
    ltc_buf[1] &= 0xFF00;
    ltc_buf[1] |= level;
    break;
  case RGB1_G_CHANNEL:
    ltc_buf[0] &= 0xFF00;
    ltc_buf[0] |= level;
    break;
  case RGB1_B_CHANNEL:
    ltc_buf[2] &= 0xFF00;
    ltc_buf[2] |= level;
    break;
  case RGB2_R_CHANNEL:
    ltc_buf[4] &= 0xFF00;
    ltc_buf[4] |= level;
    break;
  case RGB2_G_CHANNEL:
    ltc_buf[3] &= 0xFF00;
    ltc_buf[3] |= level;
    break;
  case RGB2_B_CHANNEL:
    ltc_buf[5] &= 0xFF00;
    ltc_buf[5] |= level;
    break;
  default:
    break;
  }
}

//returns actual level
uint8_t ltc3220_dec_level_in_buf(uint8_t led, uint8_t level){
	uint8_t tmp=0;
	switch (led) {
	case RGB1_R_CHANNEL:
		tmp=(uint8_t)ltc_buf[1]-level;
		if(tmp<=0x3f)
			ltc_buf[1]=(ltc_buf[1]&0xFF00)+tmp;
		break;
	case RGB1_G_CHANNEL:
		tmp=(uint8_t)ltc_buf[0]-level;
		if(tmp<=0x3f)
			ltc_buf[0]=(ltc_buf[0]&0xFF00)+tmp;
		break;
	case RGB1_B_CHANNEL:
		tmp=(uint8_t)ltc_buf[2]-level;
		if(tmp<=0x3f)
			ltc_buf[2]=(ltc_buf[2]&0xFF00)+tmp;
		break;
	case RGB2_R_CHANNEL:
		tmp=(uint8_t)ltc_buf[4]-level;
		if(tmp<=0x3f)
			ltc_buf[4]=(ltc_buf[4]&0xFF00)+tmp;
		break;
	case RGB2_G_CHANNEL:
		tmp=(uint8_t)ltc_buf[3]-level;
		if(tmp<=0x3f)
			ltc_buf[3]=(ltc_buf[3]&0xFF00)+tmp;
		break;
	case RGB2_B_CHANNEL:
		tmp=(uint8_t)ltc_buf[5]-level;
		if(tmp<=0x3f)
			ltc_buf[5]=(ltc_buf[5]&0xFF00)+tmp;
		break;
	}
	if (tmp==255)
		tmp++;
	return tmp;
}

uint8_t ltc3220_inc_level_in_buf(uint8_t led, uint8_t level){
	uint8_t tmp=0;
	switch (led) {
	case RGB1_R_CHANNEL:
		tmp=(uint8_t)ltc_buf[1]+level;
		if(tmp<=0x3f)
			ltc_buf[1]=(ltc_buf[1]&0xFF00)+tmp;
		break;
	case RGB1_G_CHANNEL:
		tmp=(uint8_t)ltc_buf[0]+level;
		if(tmp<=0x3f)
			ltc_buf[0]=(ltc_buf[0]&0xFF00)+tmp;
		break;
	case RGB1_B_CHANNEL:
		tmp=(uint8_t)ltc_buf[2]+level;
		if(tmp<=0x3f)
			ltc_buf[2]=(ltc_buf[2]&0xFF00)+tmp;
		break;
	case RGB2_R_CHANNEL:
		tmp=(uint8_t)ltc_buf[4]+level;
		if(tmp<=0x3f)
			ltc_buf[4]=(ltc_buf[4]&0xFF00)+tmp;
		break;
	case RGB2_G_CHANNEL:
		tmp=(uint8_t)ltc_buf[3]+level;
		if(tmp<=0x3f)
			ltc_buf[3]=(ltc_buf[3]&0xFF00)+tmp;
		break;
	case RGB2_B_CHANNEL:
		tmp=(uint8_t)ltc_buf[5]+level;
		if(tmp<=0x3f)
			ltc_buf[5]=(ltc_buf[5]&0xFF00)+tmp;
		break;
	}
	if (tmp==0x40)
		tmp--;
	return tmp;
}
void 	ltc3220_clear_async(){
	LTCSEND_int_HARD_FIFOClear();
	ResetBit(GPIO_LEDS_ON);
	ltc_buf[0]=RGB1_G_CHANNEL<<8;
	ltc_buf[1]=RGB1_R_CHANNEL<<8;
	ltc_buf[2]=RGB1_B_CHANNEL<<8;
	ltc_buf[3]=RGB2_G_CHANNEL<<8;
	ltc_buf[4]=RGB2_R_CHANNEL<<8;
	ltc_buf[5]=RGB2_B_CHANNEL<<8;
	SetBit(GPIO_LEDS_ON);

}

bool ltc3220_set_brightness_add_fifo(uint8_t led, uint8_t level) {
  if (led > 18)
    return false;
  add_to_ltc_buff(led, level & 0x3F);
  return true;
}
bool ltc3220_refresh_all_channels(void) {
  ResetBit(GPIO_LEDS_ON);
  SetBit(GPIO_LEDS_ON);
  return LTCSEND_send(ltc_buf, 6);
}

/**
 * @fn get_channel_current_val()
 * @brief return selected value from ltc_buf
 * @param chnnel_number which led from all of RGB you want
 * @return 8 msb are the LTC channel number, 8 lsb are value of this channel (max 0x3F)
 */
uint16_t get_channel_current_val(uint8_t channel_number) {
	return (channel_number >= NO_OF_REFRESHED_CHANNELS) ? ltc_buf[NO_OF_REFRESHED_CHANNELS-1] : ltc_buf[channel_number];
}

/** LTC FUNCTIONS */
/**
 * @fn RGB_right_set()
 * @brief set RGB right eye led
 * @param color symbol
 * @param level of brightness
 */
void RGB_right_set(uint32_t color, uint8_t level) {
	uint8_t red = ((color&COLOR_RED_MASK)>>COLOR_RED_POS)*level/100;
	uint8_t green = ((color&COLOR_GREEN_MASK)>>COLOR_GREEN_POS)*level/100;
	uint8_t blue = ((color&COLOR_BLUE_MASK)>>COLOR_BLUE_POS)*level/100;

	ltc3220_set_brightness(RGB1_R_CHANNEL, red);
	ltc3220_set_brightness(RGB1_G_CHANNEL, green);
	ltc3220_set_brightness(RGB1_B_CHANNEL, blue);
}

void RGB_left_set(uint32_t color, uint8_t level) {
	uint8_t red = ((color&COLOR_RED_MASK)>>COLOR_RED_POS)*level/100;
	uint8_t green = ((color&COLOR_GREEN_MASK)>>COLOR_GREEN_POS)*level/100;
	uint8_t blue = ((color&COLOR_BLUE_MASK)>>COLOR_BLUE_POS)*level/100;

	ltc3220_set_brightness(RGB2_R_CHANNEL, red);
	ltc3220_set_brightness(RGB2_G_CHANNEL, green);
	ltc3220_set_brightness(RGB2_B_CHANNEL, blue);
}


/**
 * @fn power_led_flash ()
 * @brief generate 2ms light flash on all power led
 */
//fukncja do zapalanie jednokrotnie ledow w testowaniu po ble i uart
void power_led_flash_test_tool ()
{
	ltc3220_set_chargepump(2);
	SetBit(GPIO_2MS_DRIVER_ON);
	ic_delay_ms(2);


	ResetBit(GPIO_2MS_DRIVER_ON);
}
//do teriapii jet lag używane w maine
void power_led_flash ()
{
	uint8_t tmp;
	sd_nvic_critical_region_enter(&tmp);
	ltc3220_set_chargepump(2);
	sd_nvic_critical_region_exit(0);

	SetBit(GPIO_2MS_DRIVER_ON);
	ic_delay_ms(2);


	ResetBit(GPIO_2MS_DRIVER_ON);
}
void power_led_flash_on()
{
	SetBit(GPIO_2MS_DRIVER_ON);
}

void power_led_flash_off ()
{
	ResetBit(GPIO_2MS_DRIVER_ON);
}

void power_led_flash_test ()
{
	SetBit(GPIO_2MS_DRIVER_ON);
	ic_delay_ms(3);
	ResetBit(GPIO_2MS_DRIVER_ON);
}


void system_on_notify ()
{
	uint32_t delay = 300;
	uint32_t delay2 = 350;
	uint32_t i;
	SetBit(GPIO_POWER_DIGITAL);
	TWI_PIN_SELECT(GPIO_SCL_PIN, GPIO_SDA_PIN);
	TWI_Init(K400);
	SetBit(GPIO_LEDS_ON);

	ic_delay_ms(5);

	ltc3220_set_brightness(RGB2_R_CHANNEL, 0x3F);
	ltc3220_set_brightness(RGB2_G_CHANNEL, 0x00);
	ltc3220_set_brightness(RGB2_B_CHANNEL, 0x00);

	ltc3220_set_brightness(RGB1_R_CHANNEL, 0x3F);
	ltc3220_set_brightness(RGB1_G_CHANNEL, 0x00);
	ltc3220_set_brightness(RGB1_B_CHANNEL, 0x00);
	ic_delay_ms(delay);
	//ltc3220_clear();

	ltc3220_set_brightness(RGB2_R_CHANNEL, 0x00);
	ltc3220_set_brightness(RGB2_G_CHANNEL, 0x3F);
	ltc3220_set_brightness(RGB2_B_CHANNEL, 0x00);

	ltc3220_set_brightness(RGB1_R_CHANNEL, 0x00);
	ltc3220_set_brightness(RGB1_G_CHANNEL, 0x3F);
	ltc3220_set_brightness(RGB1_B_CHANNEL, 0x00);
	ic_delay_ms(delay);
	//ltc3220_clear();

	ltc3220_set_brightness(RGB2_R_CHANNEL, 0x00);
	ltc3220_set_brightness(RGB2_G_CHANNEL, 0x00);
	ltc3220_set_brightness(RGB2_B_CHANNEL, 0x3F);

	ltc3220_set_brightness(RGB1_R_CHANNEL, 0x00);
	ltc3220_set_brightness(RGB1_G_CHANNEL, 0x00);
	ltc3220_set_brightness(RGB1_B_CHANNEL, 0x3F);
	ic_delay_ms(delay);
	//ltc3220_clear();
	ResetBit(GPIO_LEDS_ON);
	ic_delay_ms(delay2);
	SetBit(GPIO_LEDS_ON);
	ic_delay_ms(5);
	power_led_flash();
	ic_delay_ms(delay2);
	power_led_flash();
	ic_delay_ms(delay2);
	//ltc3220_clear();

	ResetBit(GPIO_LEDS_ON);
	ic_delay_ms(50);
	SetBit(GPIO_LEDS_ON);
	ic_delay_ms(50);
	for (i=50; i!=0; i-=1){
		ltc3220_set_brightness(VIBRATOR1_CHANNEL1,0x3F);
		ltc3220_set_brightness(VIBRATOR1_CHANNEL2,0x3F);
		ic_delay_ms(20);
		ResetBit(GPIO_LEDS_ON);
		SetBit(GPIO_LEDS_ON);
	}
	//ic_delay_ms(3*delay);
	ltc3220_clear();

	ResetBit(GPIO_LEDS_ON);
	TWI_Deinit();
}
/**
* @fn ltc3220_led_ramp_up ()
* @brief
* @param color:  RGB_COLOR_R, RGB_COLOR_G, RGB_COLOR_B
* @param stop max value 0x10
*
* */
void ltc3220_led_ramp_up(uint8_t color, uint8_t start, uint8_t stop, ltc3220_Leds which)
{
	if(stop > 0x10)
		stop = 0x10;

	if(start>=stop && color!=RGB_COLOR_R && color!=RGB_COLOR_G && color!=RGB_COLOR_B)
		return;

	volatile uint8_t i=0, how_many_states=stop - start + 1;
	//oblicz ile dodac do delaya
	volatile uint16_t delay_add=0;
	if(how_many_states>=2 && how_many_states<=5)
		delay_add=-78.5 * how_many_states + 484;
	else if(how_many_states>=6 && how_many_states<=17)
		delay_add=-7.2273 * how_many_states + 112.86;

	if(delay_add>500)
		delay_add=0;

	for(i=0; i<how_many_states; i++)
	{
		if(which&ltc3220_Left)
				ltc3220_set_brightness(RGB2_G_CHANNEL+color, i+start);
		if(which&ltc3220_Right)
				ltc3220_set_brightness(RGB1_G_CHANNEL+color, i+start);
		ic_delay_ms(49+delay_add);

		WDT_RR();
	}
}
/**
* @fn ltc3220_led_ramp_down ()
* @brief
* @param color:  RGB_COLOR_R, RGB_COLOR_G, RGB_COLOR_B
* @param start max value 0x10
*
* */
void ltc3220_led_ramp_down(uint8_t color, uint8_t start, uint8_t stop, ltc3220_Leds which)
{
	if(start > 0x10)
		start = 0x10;

	if(start<=stop && color!=RGB_COLOR_R && color!=RGB_COLOR_G && color!=RGB_COLOR_B)
		return;

	volatile uint8_t i=0, how_many_states=start - stop + 1;
	//oblicz ile dodac do delaya
	volatile uint16_t delay_add=0;
	if(how_many_states>=2 && how_many_states<=5)
		delay_add=-78.5 * how_many_states + 484;
	else if(how_many_states>=6 && how_many_states<=17)
		delay_add=-7.2273 * how_many_states + 112.86;

	if(delay_add>500)
		delay_add=0;

	for(i=how_many_states; i>0; i--)
	{
		if(which&ltc3220_Left)
				ltc3220_set_brightness(RGB2_G_CHANNEL+color, i-1);
		if(which&ltc3220_Right)
				ltc3220_set_brightness(RGB1_G_CHANNEL+color, i-1);
		ic_delay_ms(49+delay_add);

		WDT_RR();
	}
}
