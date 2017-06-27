/**
 * @file    ic_cc.h
 * @author  Paweł Kaźmierzewski <p.kazmierzewski@inteliclinic.com>
 * @author  Wojciech Weclewski <w.weclewski@inteliclinic.com>
 * @date    July, 2016
 * @brief   Command control module.
 *
 *  Command frames interpreter.
 *
 */
#ifndef IC_CC_H
#define IC_CC_H

#include <stdint.h>
#include <stdbool.h>
#include "ic_frame_handle.h"
#include "ic_device_handle_api.h"

typedef enum {
  CC_NO_DATA = 0x00,
  CC_RGB_CMD,
  CC_POWER_CMD,
  CC_VIBRA_CMD,
  CC_DEVICE_CMD,
  CC_DEVICE_RSP,
  CC_PULSEOXIMETER_CMD,
  CC_PULSEOXIMETER_RSP,
  CC_E_ALARM_CMD,
  CC_E_ALARM_RSP,
  CC_EEG_MEASURED,
  CC_RED_MEASURED,
  CC_IR_MEASURED,
  CC_ACC_MEASURED,
  CC_TEMP_MEASURED,
  CC_UPDATE_POWER,
  CC_UPDATE_RR_LED,
  CC_UPDATE_RG_LED,
  CC_UPDATE_RB_LED,
  CC_UPDATE_LR_LED,
  CC_UPDATE_LG_LED,
  CC_UPDATE_LB_LED,
  CC_UPDATE_VIBRATOR,
  CC_STATUS_CMD,
  CC_DFU_CMD,
  CC_BATTERY_MEASURE,
  CC_BLE_CONNECTED,
  CC_BLE_DISCONNECTED,
  CC_CLI_PARSE,

  /*Do not touch. I'm just a sneaky motherfucker*/
  /*So sneaky*/
  CC_NO_OF_SIGNALS
}e_ccSignalType;

#ifndef ACC_DATA
#define ACC_DATA
typedef struct __attribute__((packed)){
  int16_t ACC_X, ACC_Y, ACC_Z;
} acc_data;
#endif /* !ACC_DATA */

typedef struct{
  uint32_t timestamp;
  int16_t data;
}s_int16_timestamp;

typedef struct{
  uint32_t timestamp;
  int32_t data;
}s_int32_timestamp;

typedef struct{
  uint32_t timestamp;
  acc_data data;
}s_acc_data_timestamp;

typedef uint32_t(*fp_get_u32)();
typedef uint16_t(*fp_get_u16)();
typedef uint8_t(*fp_get_u8)();
typedef int32_t(*fp_get_i32)();
typedef int16_t(*fp_get_i16)();
typedef int8_t(*fp_get_i8)();
typedef bool(*fp_get_bool)();
typedef s_rgbCmd(*fp_get_rgb_cmd)();
typedef s_powerCmd(*fp_get_power_cmd)();
typedef s_vibraCmd(*fp_get_vibra_cmd)();
typedef s_deviceCmd(*fp_get_device_cmd)();
typedef s_poxCmd(*fp_get_pox_cmd)();
typedef s_alarmCmd(*fp_get_alarm_cmd)();
typedef acc_data(*fp_get_acc_data)();
typedef s_int16_timestamp(*fp_get_i16_time)();
typedef s_int32_timestamp(*fp_get_i32_time)();
typedef s_acc_data_timestamp(*fp_get_acc_data_time)();
typedef u_devVal(*fp_dev_get_selected_output)(uint8_t);
typedef u_devVal(*fp_dev_power_led_get_function)();
typedef u_BLECmdPayload(*fp_get_rsp_frame)();
typedef uint16_t(*fp_get_status_cmd_id)();
/*ONLY FOR CLI PURPOSES*/
typedef s_deviceCmd(*fp_get_pwr_led_cmd)();
typedef s_deviceCmd(*fp_get_vib_cmd)();
/*ONLY FOR CLI PURPOSES END*/

typedef union {
  uint32_t dummy;
  fp_get_u32 get_u32;
  fp_get_u16 get_u16;
  fp_get_u8 get_u8;

  fp_get_i32 get_i32;
  fp_get_i16 get_i16;
  fp_get_i8 get_i8;

  fp_get_bool get_bool;

  fp_get_rgb_cmd get_rgb_cmd;
  fp_get_power_cmd get_power_cmd;
  fp_get_vibra_cmd get_vibra_cmd;
  fp_get_device_cmd get_device_cmd;
  fp_get_pox_cmd get_pox_cmd;
  fp_get_alarm_cmd get_alarm_cmd;

  fp_get_rsp_frame get_rsp_frame;
  fp_get_status_cmd_id get_status_cmd_id;

  fp_get_acc_data get_acc_data;

  fp_get_i16_time get_i16_time;
  fp_get_i32_time get_i32_time;
  fp_get_acc_data_time get_acc_data_time;
  fp_dev_get_selected_output dev_get_selected_output;
  fp_dev_power_led_get_function dev_power_led_get_function;

/*ONLY FOR CLI PURPOSES*/
  fp_get_pwr_led_cmd get_pwr_led_cmd;
  fp_get_vib_cmd get_vib_cmd;
/*ONLY FOR CLI PURPOSES END*/
}u_ccDataAccessFunction;

bool cc_emit (e_ccSignalType, uint32_t);
e_ccSignalType cc_probe(u_ccDataAccessFunction *);

#endif /* !IC_CC_H */
