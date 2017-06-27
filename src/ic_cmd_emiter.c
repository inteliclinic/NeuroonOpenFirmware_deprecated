/**
 * @file    ic_cmd_emiter.c
 * @author  Paweł Kaźmierzewski <p.kazmierzewski@inteliclinic.com>
 * @author  Wojciech Weclewski <w.weclewski@inteliclinic.com>
 * @date    September, 2016
 * @brief   Command payload wrapper.
 *
 *  Bluetooth command frames parser.
 */

#include <string.h>
#include "ic_cc.h"
#include "ic_frame_handle.h"
#include "ic_cli.h"
#include "ic_cmd_emiter.h"

static s_rgbCmd rgb_cmd;
static s_powerCmd power_cmd;
static s_vibraCmd vibra_cmd;
static s_deviceCmd device_cmd;
static s_poxCmd pox_cmd;
static s_alarmCmd alarm_cmd;
static bool dfu_cmd;
static uint16_t status_cmd_id;

s_rgbCmd get_rgb_cmd(){
  return rgb_cmd;
}
s_powerCmd get_power_cmd(){
  return power_cmd;
}
s_vibraCmd get_vibra_cmd(){
  return vibra_cmd;
}
s_deviceCmd get_device_cmd(){
  return device_cmd;
}
s_poxCmd get_pox_cmd(){
  return pox_cmd;
}
s_alarmCmd get_alarm_cmd(){
  return alarm_cmd;
}
bool get_dfu_cmd(){
  return dfu_cmd;
}
uint16_t get_status_cmd_id(){
  return status_cmd_id;
}

/**
 * @fn push_data_CMD0Frame(uint8_t *data, uint16_t len)
 * @brief validates data from CMD channel
 *
 *
 * @param [in] data provided data. It's expected to be 20 bytes long
 * @param [in] len added to provide future compatibility
 *
 * @retval false data is not valid
 * @retval true data is a valid frame
 *
 */
bool push_data_CMD0Frame(uint8_t *data, uint16_t len){
  u_cmdFrameContainer cmd_frame_container;
  memcpy((void *)&cmd_frame_container, (void *)data, len);

  if (!neuroon_cmd_frame_validate(data, len))
    return false;

  uint32_t fp = 0;
  e_ccSignalType signal = CC_NO_DATA;

  switch(cmd_frame_container.frame.cmd){
    case RGB_LED_CMD:
      signal = CC_RGB_CMD;
      rgb_cmd = cmd_frame_container.frame.payload.rgb_cmd;
      fp = (uint32_t)get_rgb_cmd;
      break;
    case POWER_LED_CMD:
      signal = CC_POWER_CMD;
      power_cmd = cmd_frame_container.frame.payload.power_cmd;
      fp = (uint32_t)get_power_cmd;
      break;
    case VIBRATOR_CMD:
      signal = CC_VIBRA_CMD;
      vibra_cmd = cmd_frame_container.frame.payload.vibra_cmd;
      fp = (uint32_t)get_vibra_cmd;
      break;
    case DEVICE_CMD:
      signal = CC_DEVICE_CMD;
      device_cmd = cmd_frame_container.frame.payload.device_cmd;
      fp = (uint32_t)get_device_cmd;
      break;
    case PULSEOXIMETER_CMD:
      signal = CC_PULSEOXIMETER_CMD;
      pox_cmd = cmd_frame_container.frame.payload.pox_cmd;
      fp = (uint32_t)get_pox_cmd;
      break;
    case E_ALARM_CMD:
      signal = CC_E_ALARM_CMD;
      alarm_cmd = cmd_frame_container.frame.payload.alarm_cmd;
      fp = (uint32_t)get_alarm_cmd;
      break;
    case STATUS_CMD:
      signal = CC_STATUS_CMD;
      status_cmd_id = cmd_frame_container.frame.payload.status_cmd.id;
      fp = (uint32_t)get_status_cmd_id;
      break;
    case DFU_CMD:
      signal = CC_DFU_CMD;
      dfu_cmd = cmd_frame_container.frame.payload.enable_dfu;
      fp = (uint32_t)get_dfu_cmd;
      break;
    default:
      break;
  }
  cc_emit(signal, fp);
  return true;
}

