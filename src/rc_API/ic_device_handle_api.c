/**
 * @file    ic_device_handle_api.c
 * @author  Paweł Kaźmierzewski <p.kazmierzewski@inteliclinic.com>
 * @author  Wojciech Węclewski <w.weclewski@inteliclinic.com>
 * @date    October, 2016
 * @brief   LTC devices interface.
 *
 *  LTC devices are RGB LEDs, vibration motor and power LEDs.
 */

#include <stdint.h>
#include <string.h>
#include "ic_device_handle_api.h"
#include "ic_timers.h"
#include "ic_cmd_emiter.h"
#include "ic_rsp_emiter.h"
#include "ic_cc.h"

#define TIME_CONVERT_MULTIPLIER 100 //converts duration and period (put in hundreds of ms to ms)

const char* dev_names[REGISTERED_DEVICES_NUMBER] = {"power led", "right red led", "right green led", "right blue led", "left red led", "left green led", "left blue led", "vibrator"};
devDescriptor devices[REGISTERED_DEVICES_NUMBER];
deviceOutput dev_current_values[REGISTERED_DEVICES_NUMBER];

static void priv_dev_build_resp_frame(s_deviceRsp *response_payload, s_deviceCmd *command_payload, bool resp_state_code);
static void priv_dev_update_emit(uint8_t desc);
static void priv_dev_emit_signal(s_deviceRsp *device_rsp_frame);

u_devVal dev_get_selected_output(uint8_t desc) {
  u_devVal tmp_dev_val = {.channels.desc = desc, .channels.value = dev_current_values[desc]};
  return tmp_dev_val;
}

u_devVal dev_power_led_get_function(void) {
  u_devVal tmp_dev_val = {.power_led_period = dev_get_period(POWER_LED)};
  return tmp_dev_val;
}

s_devsFunc dev_get_all_functions(void){
  s_devsFunc current_devices_functions = {
    .func_of_pwr_led = dev_get_function(POWER_LED),
    .func_of_right_red_led = dev_get_function(RIGHT_RED_LED),
    .func_of_right_green_led = dev_get_function(RIGHT_GREEN_LED),
    .func_of_right_blue_led = dev_get_function(RIGHT_BLUE_LED),
    .func_of_left_red_led = dev_get_function(LEFT_RED_LED),
    .func_of_left_green_led = dev_get_function(LEFT_GREEN_LED),
    .func_of_left_blue_led = dev_get_function(LEFT_BLUE_LED),
    .func_of_vibrator = dev_get_function(VIBRATOR) };
  return current_devices_functions;
}

void dev_all_handlers_register(void) {
  for (uint8_t i = 0; i < REGISTERED_DEVICES_NUMBER; i++)
    dev_register(i, dev_names[i], &devices[i]);
}

void dev_all_update_when_active(void) {
  for (uint8_t i = 0; i < REGISTERED_DEVICES_NUMBER; i++) {
    if (dev_is_active(devices[i])){
      dev_current_values[i] = dev_update(devices[i], timer_get_ms_counter());
      priv_dev_update_emit(i);
    }
  }
}

// RGB section ==============================
bool dev_right_rgb_set_new_parameters(s_deviceCmd *device_cmd_data) {
  bool resp_code = true;
  if (DEV_CHECK(device_cmd_data->device,DEV_RIGHT_RED_LED)) {
    resp_code &= dev_set_new_parameters(
        RIGHT_RED_LED,
        device_cmd_data->func_type,
        device_cmd_data->intensity.right_red_led,
        device_cmd_data->func_parameter.periodic_func.duration*TIME_CONVERT_MULTIPLIER,
        (uint32_t)(device_cmd_data->func_parameter.periodic_func.period*TIME_CONVERT_MULTIPLIER),
        timer_get_ms_counter());
  }
  if (DEV_CHECK(device_cmd_data->device,DEV_RIGHT_GREEN_LED)) {
    resp_code &= dev_set_new_parameters(
        RIGHT_GREEN_LED,
        device_cmd_data->func_type,
        device_cmd_data->intensity.right_green_led,
        device_cmd_data->func_parameter.periodic_func.duration*TIME_CONVERT_MULTIPLIER,
        (uint32_t)(device_cmd_data->func_parameter.periodic_func.period*TIME_CONVERT_MULTIPLIER),
        timer_get_ms_counter());
  }
  if (DEV_CHECK(device_cmd_data->device,DEV_RIGHT_BLUE_LED)) {
    resp_code &= dev_set_new_parameters(
        RIGHT_BLUE_LED,
        device_cmd_data->func_type,
        device_cmd_data->intensity.right_blue_led,
        device_cmd_data->func_parameter.periodic_func.duration*TIME_CONVERT_MULTIPLIER,
        (uint32_t)(device_cmd_data->func_parameter.periodic_func.period*TIME_CONVERT_MULTIPLIER),
        timer_get_ms_counter());
  }

  return resp_code;
}

bool dev_left_rgb_set_new_parameters(s_deviceCmd *device_cmd_data) {
  bool resp_code = true;
  if (DEV_CHECK(device_cmd_data->device,DEV_LEFT_RED_LED)) {
    resp_code &= dev_set_new_parameters(
        LEFT_RED_LED,
        device_cmd_data->func_type,
        device_cmd_data->intensity.left_red_led,
        device_cmd_data->func_parameter.periodic_func.duration*TIME_CONVERT_MULTIPLIER,
        (uint32_t)(device_cmd_data->func_parameter.periodic_func.period*TIME_CONVERT_MULTIPLIER),
        timer_get_ms_counter());
  }
  if (DEV_CHECK(device_cmd_data->device,DEV_LEFT_GREEN_LED)) {
    resp_code &= dev_set_new_parameters(
        LEFT_GREEN_LED,
        device_cmd_data->func_type,
        device_cmd_data->intensity.left_green_led,
        device_cmd_data->func_parameter.periodic_func.duration*TIME_CONVERT_MULTIPLIER,
        (uint32_t)(device_cmd_data->func_parameter.periodic_func.period*TIME_CONVERT_MULTIPLIER),
        timer_get_ms_counter());
  }
  if (DEV_CHECK(device_cmd_data->device,DEV_LEFT_BLUE_LED)) {
    resp_code &= dev_set_new_parameters(
        LEFT_BLUE_LED,
        device_cmd_data->func_type,
        device_cmd_data->intensity.left_blue_led,
        device_cmd_data->func_parameter.periodic_func.duration*TIME_CONVERT_MULTIPLIER,
        (uint32_t)(device_cmd_data->func_parameter.periodic_func.period*TIME_CONVERT_MULTIPLIER),
        timer_get_ms_counter());
  }

  return resp_code;
}
// RGB section end ===========================

// VIBRATOR section ==========================
bool dev_vibrator_set_new_parameters(s_deviceCmd *device_cmd_data) {
  bool resp_code = true;
  if (DEV_CHECK(device_cmd_data->device,DEV_VIBRATOR)) {
    resp_code &= dev_set_new_parameters(
        VIBRATOR,
        device_cmd_data->func_type,
        device_cmd_data->intensity.vibrator,
        device_cmd_data->func_parameter.periodic_func.duration*TIME_CONVERT_MULTIPLIER,
        (uint32_t)(device_cmd_data->func_parameter.periodic_func.period*TIME_CONVERT_MULTIPLIER),
        timer_get_ms_counter());
  }

  return resp_code;
}
// VIBRATOR section end ========================

// POWER_LED section =========================
bool dev_power_set_new_parameters(s_deviceCmd *device_cmd_data) {
  bool resp_code = true;
  if (DEV_CHECK(device_cmd_data->device,DEV_POWER_LED)) {
    if (device_cmd_data->func_type != FUN_TYPE_OFF && device_cmd_data->func_type != FUN_TYPE_BLINK) {
      device_cmd_data->func_type = FUN_TYPE_ON;
    }
    resp_code &= dev_set_new_parameters(
        POWER_LED,
        device_cmd_data->func_type,
        0,
        device_cmd_data->func_parameter.periodic_func.duration*TIME_CONVERT_MULTIPLIER,
        (uint32_t)(device_cmd_data->func_parameter.periodic_func.period*TIME_CONVERT_MULTIPLIER),
        timer_get_ms_counter());
  }

  return resp_code;
}
// POWER_LED section end ======================

// DEVICE FRAME section ========================

void dev_all_set_new_parameters(s_deviceCmd *device_cmd_data) {
  bool resp_code = true;
  s_deviceRsp device_rsp_frame;
  memset(&device_rsp_frame, 0, sizeof(s_deviceRsp));

  resp_code &= dev_right_rgb_set_new_parameters(device_cmd_data);
  resp_code &= dev_left_rgb_set_new_parameters(device_cmd_data);
  resp_code &= dev_power_set_new_parameters(device_cmd_data);
  resp_code &= dev_vibrator_set_new_parameters(device_cmd_data);

  priv_dev_build_resp_frame(&device_rsp_frame, device_cmd_data, resp_code);
  priv_dev_emit_signal(&device_rsp_frame);
}
// DEVICE FRAME section end =====================

static void priv_dev_build_resp_frame(s_deviceRsp *response_payload, s_deviceCmd *command_payload, bool resp_state_code) {
  response_payload->id          = command_payload->id;
  response_payload->device      = command_payload->device;
  response_payload->func_type   = command_payload->func_type;
  response_payload->duration    = command_payload->func_parameter.periodic_func.duration;
  response_payload->period      = command_payload->func_parameter.periodic_func.period;
  response_payload->state_code  = resp_state_code;
}

static void priv_dev_update_emit(uint8_t desc){
  if (dev_is_val_changed((devDescriptor)desc)) {
    switch(desc) {
      case RIGHT_RED_LED:
        cc_emit(CC_UPDATE_RR_LED,(uint32_t)dev_get_selected_output);
        break;
      case RIGHT_GREEN_LED:
        cc_emit(CC_UPDATE_RG_LED,(uint32_t)dev_get_selected_output);
        break;
      case RIGHT_BLUE_LED:
        cc_emit(CC_UPDATE_RB_LED,(uint32_t)dev_get_selected_output);
        break;
      case LEFT_RED_LED:
        cc_emit(CC_UPDATE_LR_LED,(uint32_t)dev_get_selected_output);
        break;
      case LEFT_GREEN_LED:
        cc_emit(CC_UPDATE_LG_LED,(uint32_t)dev_get_selected_output);
        break;
      case LEFT_BLUE_LED:
        cc_emit(CC_UPDATE_LB_LED,(uint32_t)dev_get_selected_output);
        break;
      case VIBRATOR:
        cc_emit(CC_UPDATE_VIBRATOR,(uint32_t)dev_get_selected_output);
        break;
      case POWER_LED:
        if (dev_is_active(POWER_LED))
          cc_emit(CC_UPDATE_POWER,(uint32_t)dev_power_led_get_function);
        break;
      default:
        break;
    }
  }
}

static void priv_dev_emit_signal(s_deviceRsp *device_rsp_frame){
  add_rsp_frame((u_BLECmdPayload)*device_rsp_frame);
  cc_emit(CC_DEVICE_RSP, (uint32_t)get_rsp_frame);
}
