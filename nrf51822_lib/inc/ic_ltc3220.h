#ifndef IC_LTC3220_H_
#define IC_LTC3220_H_

/**
 * @file
 * @brief LTC3220 driver header
 */

#include "ic_lib_config.h"

/** SLAVE ADDRESS */
#define LTC3220_TWI_ADDRESS 					0x1C
/// 400kHZ I2C speed

/** REGISTER ADDRESS */
#define LTC3220_COMMAND_REG						0x00
#define LTC3220_GRAD_BLINK_REG					0x13

/** REGISTER VALUE */

/** COMMAND REGISTER */
#define LTC3220_NORMAL_MODE						0x00
#define LTC3220_QUICK_WRITE						0x01
#define LTC3220_FORCE1p5						0x02
#define LTC3220_FORCE2							0x04
#define LTC3220_SHUTDOWN						0x08

/** ULED REGISTER */
#define LTC3220_LED_OFF							0x00
#define LTC3220_LED_BLINK_EN					0x40
#define LTC3220_LED_GRADATION_EN				0x80
#define LTC3220_LED_GPO_MODE					0xC0

/** GRADATION/BLINK REGISTER */
#define LTC3220_GRAD_DOWN						0x00
#define LTC3220_GRAD_UP							0x01
// Gradation Ramp Times and Period
#define LTC3220_GRAD_DIS						0x00
#define LTC3220_GRAD_MODE1						0x02		/// Ramp time = 0,24s; Period = 0.313s
#define LTC3220_GRAD_MODE2						0x04		/// Ramp time = 0,48s; Period = 0.625s
#define LTC3220_GRAD_MODE3						0x06		/// Ramp time = 0,96s; Period = 1,25s
// Blink Times and Period
#define LTC3220_BLINK_MODE0						0x00		/// On time = 0,625s; Period = 1,25s
#define LTC3220_BLINK_MODE1						0x08		/// On time = 0,156; Period = 1,25s
#define LTC3220_BLINK_MODE2						0x10		/// On time = 0,625; Period = 2,5s
#define LTC3220_BLINK_MODE3						0x18		/// On time = 0,156; Period = 2,5s

/** COLOR CONTROL FOR RGB LEDs */
#define COLOR_RED_MASK							0x3F0000
#define COLOR_GREEN_MASK						0x003F00
#define COLOR_BLUE_MASK							0x00003F

#define COLOR_RED_POS							16
#define COLOR_GREEN_POS							8
#define COLOR_BLUE_POS							0

#define COLOR_BLACK								0x000000
#define COLOR_WHITE								0x3F3F3F
#define COLOR_RED								0x3F0000
#define COLOR_GREEN								0x003F00
#define COLOR_BLUE								0x00003F
#define COLOR_YELLOW							0x3F3F00
#define COLOR_MAGENTA							0x3F003F
#define COLOR_CYAN								0x003F3F

#define RGB_COLOR_R				1
#define RGB_COLOR_G				0
#define RGB_COLOR_B				2

#define LEDS_IN_SLEEP_INTENSITY	0x03  //poprzednio 0x10,ale było za dużo


typedef enum{
	ltc3220_Both=3,
	ltc3220_Left=1,
	ltc3220_Right=2,
	ltc3220_leds_not_set=4
}ltc3220_Leds;

typedef enum{
	ANIM_direction_up=1,
	ANIM_direction_down=2,
	ANIM_direction_not_set=3
}ANIM_direction;

typedef struct{
	uint8_t color;
	uint8_t start;
	uint8_t stop;
	uint8_t licznik;
	ltc3220_Leds which;
	ANIM_direction direction;
	uint8_t how_many_tri;
	uint8_t how_many_tri_left;
	bool in_progress;
}ANIM_struct;
ANIM_struct ANIM_STRUCT;

/** DEVICE DEPENDENT FUNCTIONS */
uint8_t ltc3220_init(void);
uint8_t ltc3220_shutdown(void);
uint8_t ltc3220_set_chargepump(uint8_t mode);
uint8_t ltc3220_serial_write(void);
uint8_t ltc3220_set_brightness(uint8_t led, uint8_t level);
void ltc3220_clear(void);

/**Asynchronus LTC refreshing*/
void 	ltc3220_clear_async(void);
bool	ltc3220_set_brightness_add_fifo(uint8_t led, uint8_t level);
bool	ltc3220_refresh_all_channels(void);
uint16_t get_channel_current_val(uint8_t channel_number);
//returns actual level after decrease value in buf
uint8_t ltc3220_dec_level_in_buf(uint8_t led, uint8_t level);
//returns actual level after increase value in buf
uint8_t ltc3220_inc_level_in_buf(uint8_t led, uint8_t level);


/** LTC FUNCTIONS */
void RGB_right_set(uint32_t color, uint8_t level);
void RGB_left_set(uint32_t color, uint8_t level);

void power_led_flash(void);
void power_led_flash_test(void);
void power_led_flash_test_tool(void);
void power_led_flash_on (void);

void power_led_flash_off (void);
void system_on_notify (void);

void ltc3220_led_ramp_up(uint8_t color, uint8_t start, uint8_t stop, ltc3220_Leds which);
void ltc3220_led_ramp_down(uint8_t color, uint8_t start, uint8_t stop, ltc3220_Leds which);

#endif /* IC_LTC3220_H_ */
