/**
 * @file    ic_mixed_cli_procedures.h
 * @author  Paweł Kaźmierzewski <p.kazmierzewski@inteliclinic.com>
 * @author  Wojciech Weclewski <w.weclewski@inteliclinic.com>
 * @date    February, 2017
 * @brief   Contains CLI commands for testing all modules together.
 *
 *  For additional information about CLI, look into ic_cli module.
 */

#include "ic_mixed_cli_procedures.h"
#include "ic_common_cli_procedures.h"
#include "ic_low_level_control.h"
#include "ic_cmd_emiter.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "ic_timers.h"
#include "ic_cli.h"

/** @defgroup DIFFERENT_RESPONSE_SECTION macros for different response check tests
 *
 * @{
 */
#define LTC_DEV_1_TIME    0
#define AFE_PDN_ON_TIME   LTC_DEV_1_TIME+100
#define LTC_DEV_2_TIME    AFE_PDN_ON_TIME+100
#define AFE_PDN_OFF_TIME  LTC_DEV_2_TIME+100
#define DRC_CHK_SEQ_TIME  AFE_PDN_OFF_TIME+1000

#define SEQ_LOOPS 3

uint8_t drc_loop_counter;
bool drc_start = false;

typedef enum {
  DRC_IDLE,
  DRC_LTC_DEV_1,
  DRC_AFE_PDN_ON,
  DRC_LTC_DEV_2,
  DRC_AFE_PDN_OFF,
  DRC_TEST_END
}e_drc_states;

static void drc_afe4400_powerdown_on(void);
static void drc_afe4400_powerdown_off(void);
static void drc_cli_state_selector(uint32_t timer);
static void drc_cli_state_machine(uint32_t timer);
/** @} */ //DIFFERENT_RESPONSE_SECTION

/** @defgroup SIGNAL_COLLISION_SECTION macros for signal collision
 *
 * @{
 */
#define PWR_TOGGLE_TEST_TIME          0
#define STOP_1_TIME                   PWR_TOGGLE_TEST_TIME+7000
#define RGB_TOGGLE_TEST_TIME          STOP_1_TIME+2000
#define STOP_2_TIME                   RGB_TOGGLE_TEST_TIME+7000
#define VIB_TOGGLE_TEST_TIME          STOP_2_TIME+2000
#define STOP_3_TIME                   VIB_TOGGLE_TEST_TIME+7000
#define TWO_DEVICES_TOGGLE_TEST_TIME  STOP_3_TIME+2000
#define STOP_4_TIME                   TWO_DEVICES_TOGGLE_TEST_TIME+7000
#define ALL_DEVICES_SEND_1_TEST_TIME  STOP_4_TIME+2000
#define ALL_DEVICES_SEND_2_TEST_TIME  ALL_DEVICES_SEND_1_TEST_TIME+2000
#define ALL_DEVICES_SEND_3_TEST_TIME  ALL_DEVICES_SEND_2_TEST_TIME+2000
#define ALL_DEVICES_SEND_4_TEST_TIME  ALL_DEVICES_SEND_3_TEST_TIME+2000
#define ALL_DEVICES_SEND_5_TEST_TIME  ALL_DEVICES_SEND_4_TEST_TIME+2000
#define ALL_DEVICES_SEND_6_TEST_TIME  ALL_DEVICES_SEND_5_TEST_TIME+2000
#define STOP_5_TIME                   ALL_DEVICES_SEND_6_TEST_TIME+2000
#define TEST_END_TIME                 STOP_5_TIME+2000

#define STEP_DELAY 100

bool scd_start = false;

typedef enum {
  SCD_IDLE,
  SCD_STOP_1,
  SCD_STOP_2,
  SCD_STOP_3,
  SCD_TEST_END,
  SCD_PWR_TOGGLE,
  SCD_PWR_TOGGLE_IDLE,
  SCD_RGB_TOGGLE,
  SCD_RGB_TOGGLE_IDLE,
  SCD_VIB_TOGGLE,
  SCD_VIB_TOGGLE_IDLE,
  SCD_TWO_DEVICES_TOGGLE_1,
  SCD_TWO_DEVICES_TOGGLE_2,
  SCD_TWO_DEVICES_TOGGLE_IDLE,
  SCD_ALL_DEVICES_SEND_1_1,
  SCD_ALL_DEVICES_SEND_1_2,
  SCD_ALL_DEVICES_SEND_1_3,
  SCD_ALL_DEVICES_SEND_2_1,
  SCD_ALL_DEVICES_SEND_2_2,
  SCD_ALL_DEVICES_SEND_2_3,
  SCD_ALL_DEVICES_SEND_3_1,
  SCD_ALL_DEVICES_SEND_3_2,
  SCD_ALL_DEVICES_SEND_3_3
}e_scd_states;

static void scd_rgb_both_sin_white_max_2s(void);
static void scd_rgb_both_on_white_half_1500(void);
static void scd_toggle_pwr_led(void);
static void scd_toggle_rgb(void);
static void scd_toggle_vib(void);
static void scd_stop_rgb(void);
static void scd_stop_pwr_led(void);
static void scd_stop_vib(void);
static void scd_vib_tri_max_1500(void);
static void scd_vib_on_max_1s(void);
static void scd_vib_sq_max_500(void);
static void scd_pwr_led_on_500(void);
static void scd_pwr_led_on_1000(void);
static void scd_pwr_led_on_500_1s(void);
static void scd_cli_state_selector(uint32_t timer);
static void scd_cli_state_machine(uint32_t timer);

/** @} */ //SIGNAL_COLLISION_SECTION

static uint32_t test_start_timer;
static uint32_t step_timer;
static uint8_t state_selector;
static uint8_t state_counter;

void cli_get_status(int c, int *args){
  char array[CLI_BLE_FRAME_SIZE];
  memset(array, 0, sizeof(array));
  size_t len = CLI_BLE_FRAME_SIZE;
  status_cmd_gen_func(array, &len, (uint16_t)args[0]);
  push_data_CMD0Frame((uint8_t*)array, (uint16_t)len);
}

void cli_different_rsp_check_func(int c, int *args){
  print_cli("DIFF RSP TEST_START\n\r");
  test_start_timer = timer_get_ms_counter();
  drc_start = true;
  state_selector = DRC_LTC_DEV_1;
  state_counter = 0;
  drc_loop_counter = 0;
}

void cli_signal_collision_demo_func(int c, int *args) {
  print_cli("SIG COL TEST_START\n\r");
  test_start_timer = timer_get_ms_counter();
  scd_start = true;
  state_selector = SCD_PWR_TOGGLE;
  state_counter = 0;
}

void cli_different_rsp_check_update(void) {
  if (drc_start) {
    drc_cli_state_selector(timer_get_ms_counter());
    drc_cli_state_machine(timer_get_ms_counter());
  }
}

void cli_signal_collision_demo_update(void) {
  if (scd_start) {
    scd_cli_state_selector(timer_get_ms_counter());
    scd_cli_state_machine(timer_get_ms_counter());
  }
}

static void drc_cli_state_selector(uint32_t timer){
  if ((timer >= test_start_timer + AFE_PDN_ON_TIME) && (state_counter == 0)) {
    state_selector = DRC_AFE_PDN_ON;
    state_counter++;
  } else if ((timer >= test_start_timer + LTC_DEV_2_TIME) && (state_counter == 1)) {
    state_selector = DRC_LTC_DEV_2;
    state_counter++;
  } else if ((timer >= test_start_timer + AFE_PDN_OFF_TIME) && (state_counter == 2)) {
    state_selector = DRC_AFE_PDN_OFF;
    state_counter++;
  } else if ((timer >= test_start_timer + DRC_CHK_SEQ_TIME) && (state_counter == 3)) {
    if(++drc_loop_counter >= SEQ_LOOPS){
      state_selector = DRC_TEST_END;
    } else {
      state_selector = DRC_LTC_DEV_1;
      test_start_timer = timer_get_ms_counter();
      state_counter = 0;
    }
  }
}

static void drc_cli_state_machine(uint32_t timer){
  if (timer >= step_timer + STEP_DELAY) {
    step_timer = timer;
    switch(state_selector) {
      case DRC_IDLE:
        break;
      case DRC_LTC_DEV_1:
        print_cli("LTC_DEV_1 STATE\n\r");
        scd_rgb_both_sin_white_max_2s();
        state_selector = DRC_IDLE;
        break;
      case DRC_AFE_PDN_ON:
        print_cli("AFE_PDN_ON STATE\n\r");
        drc_afe4400_powerdown_on();
        state_selector = DRC_IDLE;
        break;
      case DRC_LTC_DEV_2:
        print_cli("LTC_DEV_2 STATE\n\r");
        scd_rgb_both_on_white_half_1500();
        state_selector = DRC_IDLE;
        break;
      case DRC_AFE_PDN_OFF:
        print_cli("AFE_PDN_OFF STATE\n\r");
        drc_afe4400_powerdown_off();
        state_selector = DRC_IDLE;
        break;
      case DRC_TEST_END:
        print_cli("TEST_END\n\r");
        drc_start = false;
      default:
        break;
    }
  }
}

static void scd_cli_state_selector(uint32_t timer) {
  if ((timer >= test_start_timer + STOP_1_TIME) && (state_counter == 0)) {
    state_selector = SCD_STOP_1;
    state_counter++;
  } else if ((timer >= test_start_timer + RGB_TOGGLE_TEST_TIME) && (state_counter == 1)) {
    state_selector = SCD_RGB_TOGGLE;
    state_counter++;
  } else if ((timer >= test_start_timer + STOP_2_TIME) && (state_counter == 2)) {
    state_selector = SCD_STOP_1;
    state_counter++;
  } else if ((timer >= test_start_timer + VIB_TOGGLE_TEST_TIME) && (state_counter == 3)) {
    state_selector = SCD_VIB_TOGGLE;
    state_counter++;
  } else if ((timer >= test_start_timer + STOP_3_TIME) && (state_counter == 4)) {
    state_selector = SCD_STOP_1;
    state_counter++;
  } else if ((timer >= test_start_timer + TWO_DEVICES_TOGGLE_TEST_TIME) && (state_counter == 5)) {
    state_selector = SCD_TWO_DEVICES_TOGGLE_1;
    state_counter++;
  } else if ((timer >= test_start_timer + STOP_4_TIME) && (state_counter == 6)) {
    state_selector = SCD_STOP_1;
    state_counter++;
  } else if ((timer >= test_start_timer + ALL_DEVICES_SEND_1_TEST_TIME) && (state_counter == 7)) {
    state_selector = SCD_ALL_DEVICES_SEND_1_1;
    state_counter++;
  } else if ((timer >= test_start_timer + ALL_DEVICES_SEND_2_TEST_TIME) && (state_counter == 8)) {
    state_selector = SCD_ALL_DEVICES_SEND_2_1;
    state_counter++;
  } else if ((timer >= test_start_timer + ALL_DEVICES_SEND_3_TEST_TIME) && (state_counter == 9)) {
    state_selector = SCD_ALL_DEVICES_SEND_3_1;
    state_counter++;
  } else if ((timer >= test_start_timer + ALL_DEVICES_SEND_4_TEST_TIME) && (state_counter == 10)) {
    state_selector = SCD_ALL_DEVICES_SEND_1_1;
    state_counter++;
  } else if ((timer >= test_start_timer + ALL_DEVICES_SEND_5_TEST_TIME) && (state_counter == 11)) {
    state_selector = SCD_ALL_DEVICES_SEND_2_1;
    state_counter++;
  } else if ((timer >= test_start_timer + ALL_DEVICES_SEND_6_TEST_TIME) && (state_counter == 12)) {
    state_selector = SCD_ALL_DEVICES_SEND_3_1;
    state_counter++;
  } else if ((timer >= test_start_timer + STOP_5_TIME) && (state_counter == 13)) {
    state_selector = SCD_STOP_1;
    state_counter++;
  } else if (timer >= test_start_timer + TEST_END_TIME) {
    state_selector = SCD_TEST_END;
  }
}

static void scd_cli_state_machine(uint32_t timer) {
  static uint32_t step_timer = 0;

  if (timer >= step_timer + STEP_DELAY) {
    /*scd_stop_pwr_led();*/
    /*scd_stop_rgb();*/
    /*scd_stop_vib();*/
    step_timer = timer;
    switch(state_selector) {
      case SCD_IDLE:
        break;
      case SCD_STOP_1:
        print_cli("STOP STATE\n\r");
        scd_stop_rgb();
        state_selector = SCD_STOP_2;
        break;
      case SCD_STOP_2:
        scd_stop_vib();
        state_selector = SCD_STOP_3;
        break;
      case SCD_STOP_3:
        scd_stop_pwr_led();
        state_selector = SCD_IDLE;
        break;
      case SCD_PWR_TOGGLE:
        print_cli("PWR_TOGGLE STATE\n\r");
        scd_rgb_both_sin_white_max_2s();
        state_selector = SCD_PWR_TOGGLE_IDLE;
        break;
      case SCD_PWR_TOGGLE_IDLE:
        scd_toggle_pwr_led();
        break;
      case SCD_RGB_TOGGLE:
        print_cli("RGB_TOGGLE STATE\n\r");
        scd_vib_tri_max_1500();
        state_selector = SCD_RGB_TOGGLE_IDLE;
        break;
      case SCD_RGB_TOGGLE_IDLE:
        scd_toggle_rgb();
        break;
      case SCD_VIB_TOGGLE:
        print_cli("VIB_TOGGLE STATE\n\r");
        scd_pwr_led_on_500();
        state_selector = SCD_VIB_TOGGLE_IDLE;
        break;
      case SCD_VIB_TOGGLE_IDLE:
        scd_toggle_vib();
        break;
      case SCD_TWO_DEVICES_TOGGLE_1:
        print_cli("TWO_DEVICES_TOGGLE_1 STATE\n\r");
        scd_rgb_both_sin_white_max_2s();
        state_selector = SCD_TWO_DEVICES_TOGGLE_2;
        break;
      case SCD_TWO_DEVICES_TOGGLE_2:
        print_cli("TWO_DEVICES_TOGGLE_2 STATE\n\r");
        scd_vib_tri_max_1500();
        state_selector = SCD_TWO_DEVICES_TOGGLE_IDLE;
        break;
      case SCD_TWO_DEVICES_TOGGLE_IDLE:
        scd_toggle_pwr_led();
        break;
      case SCD_ALL_DEVICES_SEND_1_1:
        print_cli("ALL_DEVICES_SEND_1 STATE\n\r");
        scd_rgb_both_on_white_half_1500();
        state_selector = SCD_ALL_DEVICES_SEND_1_2;
        break;
      case SCD_ALL_DEVICES_SEND_1_2:
        scd_vib_on_max_1s();
        state_selector = SCD_ALL_DEVICES_SEND_1_3;
        break;
      case SCD_ALL_DEVICES_SEND_1_3:
        scd_pwr_led_on_500_1s();
        state_selector = SCD_IDLE;
        break;
      case SCD_ALL_DEVICES_SEND_2_1:
        print_cli("ALL_DEVICES_SEND_2 STATE\n\r");
        scd_rgb_both_on_white_half_1500();
        state_selector = SCD_ALL_DEVICES_SEND_2_2;
        break;
      case SCD_ALL_DEVICES_SEND_2_2:
        scd_vib_sq_max_500();
        state_selector = SCD_ALL_DEVICES_SEND_2_3;
        break;
      case SCD_ALL_DEVICES_SEND_2_3:
        scd_pwr_led_on_500();
        state_selector = SCD_IDLE;
        break;
      case SCD_ALL_DEVICES_SEND_3_1:
        print_cli("ALL_DEVICES_SEND_3 STATE\n\r");
        scd_rgb_both_on_white_half_1500();
        state_selector = SCD_ALL_DEVICES_SEND_3_2;
        break;
      case SCD_ALL_DEVICES_SEND_3_2:
        scd_vib_tri_max_1500();
        state_selector = SCD_ALL_DEVICES_SEND_3_3;
        break;
      case SCD_ALL_DEVICES_SEND_3_3:
        scd_pwr_led_on_1000();
        state_selector = SCD_IDLE;
        break;
      case SCD_TEST_END:
        print_cli("TEST_END\n\r");
        scd_start = false;
      default:
        break;
    }
  }
}

static void drc_afe4400_powerdown_on(void){
  char array[CLI_BLE_FRAME_SIZE];
  memset(array, 0, sizeof(array));
  size_t len = CLI_BLE_FRAME_SIZE;
  pox_powerdown_on(array, &len, 0x30);
  push_data_CMD0Frame((uint8_t*)array, (uint16_t)len);
}

static void drc_afe4400_powerdown_off(void){
  char array[CLI_BLE_FRAME_SIZE];
  memset(array, 0, sizeof(array));
  size_t len = CLI_BLE_FRAME_SIZE;
  pox_powerdown_off(array, &len, 0x40);
  push_data_CMD0Frame((uint8_t*)array, (uint16_t)len);
}

static void scd_rgb_both_sin_white_max_2s(void) {
  char array[CLI_BLE_FRAME_SIZE];
  memset(array, 0, sizeof(array));
  size_t len = CLI_BLE_FRAME_SIZE;
  rgb_led_set_func(array, &len, RGB_LED_SIDE_BOTH, FUN_TYPE_SIN_WAVE, RGB_LED_COLOR_WHITE, 63, 0, 20, 0);
  push_data_CMD0Frame((uint8_t*)array, (uint16_t)len);
}

static void scd_rgb_both_on_white_half_1500(void) {
  char array[CLI_BLE_FRAME_SIZE];
  memset(array, 0, sizeof(array));
  size_t len = CLI_BLE_FRAME_SIZE;
  rgb_led_set_func(array, &len, RGB_LED_SIDE_BOTH, FUN_TYPE_ON, RGB_LED_COLOR_WHITE, 30, 15, 700, 0);
  push_data_CMD0Frame((uint8_t*)array, (uint16_t)len);
}

static void scd_toggle_pwr_led(void) {
  char array[CLI_BLE_FRAME_SIZE];
  memset(array, 0, sizeof(array));
  size_t len = CLI_BLE_FRAME_SIZE;
  static uint8_t selector = 0;
  if (selector%2 == 1) {
    pwr_led_set_func(array, &len, FUN_TYPE_ON, 0, 500, 0);
  } else {
    pwr_led_set_func(array, &len, FUN_TYPE_ON, 0, 200, 0);
  }
  selector++;
  push_data_CMD0Frame((uint8_t*)array, (uint16_t)len);
}

static void scd_toggle_rgb(void) {
  char array[CLI_BLE_FRAME_SIZE];
  memset(array, 0, sizeof(array));
  size_t len = CLI_BLE_FRAME_SIZE;
  static uint8_t selector = 0;
  if (selector%8 == 1) {
    rgb_led_set_func(array, &len, RGB_LED_SIDE_BOTH, FUN_TYPE_ON, RGB_LED_COLOR_RED, 63, 0, 2000, 0);
  } else if (selector%8 == 3) {
    rgb_led_set_func(array, &len, RGB_LED_SIDE_BOTH, FUN_TYPE_ON, RGB_LED_COLOR_GREEN, 63, 1000, 1000, 0);
  } else if (selector%8 == 5) {
    rgb_led_set_func(array, &len, RGB_LED_SIDE_BOTH, FUN_TYPE_ON, RGB_LED_COLOR_BLUE, 63, 2000, 500, 0);
  } else if (selector%8 == 7) {
    rgb_led_set_func(array, &len, RGB_LED_SIDE_BOTH, FUN_TYPE_ON, RGB_LED_COLOR_WHITE, 63, 3000, 3000, 0);
  } else {
    rgb_led_set_func(array, &len, RGB_LED_SIDE_BOTH, FUN_TYPE_OFF, RGB_LED_COLOR_TEAL, 30, 0, 0, 0);
  }
  selector++;
  push_data_CMD0Frame((uint8_t*)array, (uint16_t)len);
}

static void scd_toggle_vib(void) {
  char array[CLI_BLE_FRAME_SIZE];
  memset(array, 0, sizeof(array));
  size_t len = CLI_BLE_FRAME_SIZE;
  static uint8_t selector = 0;
  if (selector%2 == 1) {
    vibrator_set_func(array, &len, FUN_TYPE_ON, 63, 0, 500, 0);
  } else {
    vibrator_set_func(array, &len, FUN_TYPE_OFF, 63, 0, 500, 0);
  }
  selector++;
  push_data_CMD0Frame((uint8_t*)array, (uint16_t)len);
}

static void scd_stop_rgb(void) {
  char array[CLI_BLE_FRAME_SIZE];
  memset(array, 0, sizeof(array));
  size_t len = CLI_BLE_FRAME_SIZE;
  rgb_led_set_func(array, &len, RGB_LED_SIDE_BOTH, FUN_TYPE_OFF, RGB_LED_COLOR_WHITE, 30, 0, 1000, 0);
  push_data_CMD0Frame((uint8_t*)array, (uint16_t)len);
}

static void scd_stop_pwr_led(void) {
  char array[CLI_BLE_FRAME_SIZE];
  memset(array, 0, sizeof(array));
  size_t len = CLI_BLE_FRAME_SIZE;
  pwr_led_set_func(array, &len, FUN_TYPE_OFF, 0, 1000, 0);
  push_data_CMD0Frame((uint8_t*)array, (uint16_t)len);
}

static void scd_stop_vib(void) {
  char array[CLI_BLE_FRAME_SIZE];
  memset(array, 0, sizeof(array));
  size_t len = CLI_BLE_FRAME_SIZE;
  vibrator_set_func(array, &len, FUN_TYPE_OFF, 50, 0, 100, 0);
  push_data_CMD0Frame((uint8_t*)array, (uint16_t)len);
}

static void scd_vib_tri_max_1500(void) {
  char array[CLI_BLE_FRAME_SIZE];
  memset(array, 0, sizeof(array));
  size_t len = CLI_BLE_FRAME_SIZE;
  vibrator_set_func(array, &len, FUN_TYPE_TRIANGLE, 63, 0, 1500, 0);
  push_data_CMD0Frame((uint8_t*)array, (uint16_t)len);
}

static void scd_vib_on_max_1s(void) {
  char array[CLI_BLE_FRAME_SIZE];
  memset(array, 0, sizeof(array));
  size_t len = CLI_BLE_FRAME_SIZE;
  vibrator_set_func(array, &len, FUN_TYPE_ON, 63, 1000, 2500, 0);
  push_data_CMD0Frame((uint8_t*)array, (uint16_t)len);
}

static void scd_vib_sq_max_500(void) {
  char array[CLI_BLE_FRAME_SIZE];
  memset(array, 0, sizeof(array));
  size_t len = CLI_BLE_FRAME_SIZE;
  vibrator_set_func(array, &len, FUN_TYPE_SQUARE, 63, 0, 500, 0);
  push_data_CMD0Frame((uint8_t*)array, (uint16_t)len);
}

static void scd_pwr_led_on_500(void) {
  char array[CLI_BLE_FRAME_SIZE];
  memset(array, 0, sizeof(array));
  size_t len = CLI_BLE_FRAME_SIZE;
  pwr_led_set_func(array, &len, FUN_TYPE_ON, 0, 500, 0);
  push_data_CMD0Frame((uint8_t*)array, (uint16_t)len);
}

static void scd_pwr_led_on_1000(void) {
  char array[CLI_BLE_FRAME_SIZE];
  memset(array, 0, sizeof(array));
  size_t len = CLI_BLE_FRAME_SIZE;
  pwr_led_set_func(array, &len, FUN_TYPE_ON, 0, 1000, 0);
  push_data_CMD0Frame((uint8_t*)array, (uint16_t)len);
}

static void scd_pwr_led_on_500_1s(void) {
  char array[CLI_BLE_FRAME_SIZE];
  memset(array, 0, sizeof(array));
  size_t len = CLI_BLE_FRAME_SIZE;
  pwr_led_set_func(array, &len, FUN_TYPE_ON, 1000, 500, 0);
  push_data_CMD0Frame((uint8_t*)array, (uint16_t)len);
}
