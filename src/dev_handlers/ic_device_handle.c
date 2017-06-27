/**
 * @file    ic_device_handle.c
 * @author  Paweł Kaźmierzewski <p.kazmierzewski@inteliclinic.com>
 * @author  Wojciech Weclewski <w.weclewski@inteliclinic.com>
 * @date    September, 2016
 * @brief   LTC devices high-level functions.
 *
 *  LTC devices are RGB LEDs, vibration motor and power LEDs.
 */

#include "ic_device_handle.h"
#include <stdio.h>
#include <stdlib.h>

typedef enum __attribute__((packed)) {
  FUNC_TYPE_OFF = 0x01,
  FUNC_TYPE_ON,
  FUNC_TYPE_SIN_WAVE,
  FUNC_TYPE_BLINK,
  FUNC_TYPE_SQUARE,
  FUNC_TYPE_SAW,
  FUNC_TYPE_TRIANGLE,
  FUNC_TYPE_RAMP
} e_funcType;

typedef struct {
  uint32_t start_timer;
  uint32_t end_timer;
  uint32_t state_timer;
  bool is_timer_rollover;
  const char *name;
  uint32_t period;
  uint8_t id;
  e_funcType function;
  uint8_t current_value;
  uint8_t amplitude;
  bool is_active;
  bool is_val_changed;
} s_deviceState;

#define MAX_DEVICE_NUMBER       8
#define MAX_AMPLITUDE           0x3F
#define MIN_AMPLITUDE           0
#define SIN_TAB_SIZE_INDEX      6
#define SIN_TAB_SIZE            1<<SIN_TAB_SIZE_INDEX
#define TRIANGLE_TAB_SIZE_INDEX 6
#define TRIANGLE_TAB_SIZE       1<<TRIANGLE_TAB_SIZE_INDEX
#define SAW_TAB_SIZE_INDEX      6
#define SAW_TAB_SIZE            1<<SAW_TAB_SIZE_INDEX
#define TIME_OF_BLINK           10
#define RAMP_F_STEP             20 //step duration in ms

// -- GLOBALS -- //
static s_deviceState devices[MAX_DEVICE_NUMBER] = {{0}};
static uint8_t func_ptr;
static uint8_t fun_tab_cnt[MAX_DEVICE_NUMBER];
const uint8_t sin_wave[SIN_TAB_SIZE] =
  {32, 35, 38, 41, 44, 47, 50, 52, 54, 57, 58, 60, 61, 62, 63, 63,
    63, 63, 63, 62, 61, 59, 57, 55, 53, 51, 48, 45, 42, 39, 36, 33,
    30, 27, 24, 21, 18, 15, 12, 10, 8, 6, 4, 2, 1, 0, 0, 0,
    0, 0, 1, 2, 3, 5, 6, 9, 11, 13, 16, 19, 22, 25, 28, 32};
const uint8_t triangle[TRIANGLE_TAB_SIZE] =
  {0, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30,
    32, 34, 36, 38, 40, 42, 44, 46, 48, 50, 52, 54, 56, 58, 60, 62,
    63, 62, 60, 58, 56, 54, 52, 50, 48, 46, 44, 42, 40, 38, 36, 34,
    32, 30, 28, 26, 24, 22, 20, 18, 16, 14, 12, 10, 8, 6, 4, 2};
const uint8_t saw[SAW_TAB_SIZE] =
  {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15,
    16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31,
    32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47,
    48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63};

// --         -- //

static void priv_f_selected_wave_type(s_deviceState *device, devDescriptor desc, const uint8_t *function_tab, uint8_t function_tab_len);
static void priv_f_square(s_deviceState *device);
static void priv_f_ramp(s_deviceState *device, uint32_t timer);

bool dev_register(uint8_t id, const char *name, devDescriptor *desc) {
  if (func_ptr >= MAX_DEVICE_NUMBER)
    return false;

  devices[func_ptr].id = id;
  devices[func_ptr].name = name;
  devices[func_ptr].is_timer_rollover = false;
  devices[func_ptr].is_active = false;
  devices[func_ptr].is_val_changed = false;

  *desc = func_ptr++;
  return true;
}

void dev_unregister_all(void) {
  while (func_ptr) {
    devices[func_ptr].id = 0;
    func_ptr--;
    devices[func_ptr].name = "";
    devices[func_ptr].is_active = false;
    devices[func_ptr].is_val_changed = false;
  }
}

bool dev_set_new_parameters(devDescriptor desc, uint8_t function, uint8_t amplitude, uint32_t duration, uint32_t period, uint32_t timer) {
  if (desc >= func_ptr)
    return false;
  devices[desc].function = (e_funcType)function;
  devices[desc].period = period;
  devices[desc].amplitude = amplitude & MAX_AMPLITUDE;
  devices[desc].start_timer = timer;
  devices[desc].state_timer = 0;
  devices[desc].end_timer = timer + duration;
  devices[desc].is_timer_rollover = devices[desc].end_timer < devices[desc].start_timer ? true : false;
  devices[desc].is_active = true;
  return true;
}

const char* dev_get_name(devDescriptor desc) {
  return devices[desc].name;
}

uint8_t dev_get_function(devDescriptor desc) {
  return (uint8_t)devices[desc].function;
}

uint32_t dev_get_period(devDescriptor desc) {
  return devices[desc].period;
}

uint8_t dev_get_id(devDescriptor desc) {
  return devices[desc].id;
}

bool dev_is_active(devDescriptor desc) {
  return devices[desc].is_active;
}

bool dev_is_val_changed(devDescriptor desc) {
  bool tmp = devices[desc].is_val_changed;
  devices[desc].is_val_changed = false;
  return tmp;
}

deviceOutput dev_update(devDescriptor desc, uint32_t timer) {
  if (desc >= func_ptr)
    return 0;
  else if (!devices[desc].is_active)
    return devices[desc].current_value;

  if (timer >= devices[desc].state_timer) {
    switch (devices[desc].function) {
    case FUNC_TYPE_OFF:
      devices[desc].current_value = 0;
      devices[desc].is_active = false;
      fun_tab_cnt[desc] = 0;
      devices[desc].is_val_changed = true;
      break;
    case FUNC_TYPE_ON:
      devices[desc].current_value = devices[desc].amplitude;
      devices[desc].is_val_changed = (devices[desc].current_value == devices[desc].amplitude) ? false : true;
      break;
    case FUNC_TYPE_SIN_WAVE:
      priv_f_selected_wave_type(&devices[desc], desc, sin_wave, SIN_TAB_SIZE);
      devices[desc].state_timer = timer + (devices[desc].period>>SIN_TAB_SIZE_INDEX);
      devices[desc].is_val_changed = true;
      break;
    case FUNC_TYPE_BLINK:
      devices[desc].current_value = devices[desc].amplitude;
      devices[desc].state_timer = timer + TIME_OF_BLINK;
      devices[desc].function = FUNC_TYPE_OFF;
      devices[desc].is_val_changed = true;
      break;
    case FUNC_TYPE_SQUARE:
      priv_f_square(&devices[desc]);
      devices[desc].state_timer = timer + (devices[desc].period>>1);
      devices[desc].is_val_changed = true;
      break;
    case FUNC_TYPE_SAW:
      priv_f_selected_wave_type(&devices[desc], desc, saw, SAW_TAB_SIZE);
      devices[desc].state_timer = timer + (devices[desc].period>>SAW_TAB_SIZE_INDEX);
      devices[desc].is_val_changed = true;
      break;
    case FUNC_TYPE_TRIANGLE:
      priv_f_selected_wave_type(&devices[desc], desc, triangle, TRIANGLE_TAB_SIZE);
      devices[desc].state_timer = timer + (devices[desc].period>>TRIANGLE_TAB_SIZE_INDEX);
      devices[desc].is_val_changed = true;
      break;
    case FUNC_TYPE_RAMP:
      priv_f_ramp(&devices[desc], timer);
      break;
    default:
      devices[desc].function = FUNC_TYPE_OFF;
    }
  }

  if (devices[desc].start_timer != devices[desc].end_timer) {
    if (!devices[desc].is_timer_rollover && timer > devices[desc].end_timer) {
      devices[desc].function = FUNC_TYPE_OFF;
    } else if (devices[desc].is_timer_rollover && timer < devices[desc].start_timer) {
      devices[desc].is_timer_rollover = false;
    }
  }

  return devices[desc].current_value;
}

static void priv_f_selected_wave_type(s_deviceState *device, devDescriptor desc, const uint8_t *function_tab, uint8_t function_tab_len) {
  uint8_t val_correction = MAX_AMPLITUDE - device->amplitude;
  device->current_value = function_tab[fun_tab_cnt[desc]] > val_correction ? function_tab[fun_tab_cnt[desc]] - val_correction : 0;
  if ((++fun_tab_cnt[desc]) >= function_tab_len)
    fun_tab_cnt[desc] = 0;
}

static void priv_f_square(s_deviceState *device) {
  if (device->current_value == device->amplitude)
    device->current_value = 0;
  else
    device->current_value = device->amplitude;
}

static void priv_f_ramp(s_deviceState *device, uint32_t timer){
  if(device->amplitude > device->current_value){
    device->current_value++;
    device->is_val_changed = true;
  }else if(device->amplitude < device->current_value){
    device->current_value--;
    device->is_val_changed = true;
  }else{
    device->function = FUNC_TYPE_ON;
    device->is_val_changed = false;
  }

  device->state_timer = timer + RAMP_F_STEP;
}

