/**
 * @file    ic_e_alarm_api.c
 * @author  Paweł Kaźmierzewski <p.kazmierzewski@inteliclinic.com>
 * @author  Wojciech Węclewski <w.weclewski@inteliclinic.com>
 * @date    February, 2017
 * @brief   API for emergency alarm module.
 *
 *  Emergency alarm is a alarm feature used when Neuroon loses connection with application.
 *
 */

#include <string.h>
#include "ic_e_alarm_api.h"
#include "ic_cc.h"
#include "ic_rsp_emiter.h"
#include "ic_device_handle_api.h"

typedef enum{
  NO_CHECK = 0x00,
  NO_CONNECTION,
  CONNECTION_OK,
  CHECKING_TIMEOUT,
  CONNECTION_TIMEOUT
}e_connectionState;

static struct {
  struct {
    e_connectionState state;
    uint32_t tout_chk_end_time;
    uint16_t timeout;
  }connection_info;
  bool alarm_on;
  e_alarmType type_of_alarm;
  uint32_t time_of_alarm;
  uint32_t time_to_alarm;
}alarm_info;

static void priv_alarm_set(e_alarmType type, uint32_t time, uint16_t timeout);
static void priv_alarm_turn_off(void);
static void priv_alarm_build_rsp_frame(s_alarmRsp *alarm_rsp_frame, s_alarmCmd *alarm_cmd_data, bool state_code);
static void priv_alarm_emit_signal(s_alarmRsp *alarm_rsp_frame);
static void priv_alarm_start(uint32_t timer);
static void priv_alarm_start_soft(uint32_t timer);
static void priv_alarm_start_medium(uint32_t timer);
static void priv_alarm_start_hard(uint32_t timer);

void alarm_module_init(void){
  alarm_info.alarm_on = false;
  alarm_info.type_of_alarm = ALARM_OFF;
  alarm_info.time_of_alarm = 0;
  alarm_info.time_to_alarm = 0;
  alarm_info.connection_info.state = NO_CHECK;
  alarm_info.connection_info.tout_chk_end_time = 0;
  alarm_info.connection_info.timeout = 0;
}

void alarm_cmd_handle(s_alarmCmd *alarm_cmd_data){
  s_alarmRsp alarm_rsp_frame;
  memset(&alarm_rsp_frame, 0, sizeof(s_alarmRsp));

  if(alarm_cmd_data->type == ALARM_OFF){
    priv_alarm_turn_off();
  }else{
    priv_alarm_set(alarm_cmd_data->type, alarm_cmd_data->time_to_alarm, alarm_cmd_data->timeout);
  }
  priv_alarm_build_rsp_frame(&alarm_rsp_frame, alarm_cmd_data, true);
  priv_alarm_emit_signal(&alarm_rsp_frame);
}

void alarm_update(bool is_connected, uint32_t timer, uint32_t timer_for_device){
  if(alarm_info.alarm_on && !is_connected){
    switch(alarm_info.connection_info.state){
      case CONNECTION_OK:
        alarm_info.connection_info.state = CHECKING_TIMEOUT;
        alarm_info.connection_info.tout_chk_end_time = timer + alarm_info.connection_info.timeout;
        alarm_info.time_of_alarm = timer + alarm_info.time_to_alarm;
        break;
      case CHECKING_TIMEOUT:
        if(timer >= alarm_info.connection_info.tout_chk_end_time){
          alarm_info.connection_info.state = CONNECTION_TIMEOUT;
        }
        break;
      case CONNECTION_TIMEOUT:
        if(timer >= alarm_info.time_of_alarm){
          alarm_info.connection_info.state = NO_CONNECTION;
          priv_alarm_start(timer_for_device);
        }
        break;
      default:
        break;
    }
  }else if(is_connected){
    alarm_info.connection_info.state = CONNECTION_OK;
  }
}

static void priv_alarm_set(e_alarmType type, uint32_t time, uint16_t timeout){
  alarm_info.alarm_on = time > 0 ? true : false;
  alarm_info.type_of_alarm = type;
  alarm_info.time_of_alarm = 0;
  alarm_info.time_to_alarm = time;
  alarm_info.connection_info.state = CONNECTION_OK;
  alarm_info.connection_info.tout_chk_end_time = 0;
  alarm_info.connection_info.timeout = timeout;
}

static void priv_alarm_turn_off(void){
  alarm_info.alarm_on = false;
}

static void priv_alarm_build_rsp_frame(s_alarmRsp *alarm_rsp_frame, s_alarmCmd *alarm_cmd_data, bool state_code){
  memcpy(alarm_rsp_frame, alarm_cmd_data, sizeof(s_alarmRsp));
  alarm_rsp_frame->state_code = state_code;
}

static void priv_alarm_emit_signal(s_alarmRsp *alarm_rsp_frame){
  add_rsp_frame((u_BLECmdPayload)*alarm_rsp_frame);
  cc_emit(CC_E_ALARM_RSP, (uint32_t)get_rsp_frame);
}

/*========================SOFT CONFIG=========================*/
#define SA_RGB_FUN_TYPE   FUN_TYPE_SIN_WAVE
#define SA_RGB_AMPLITUDE  63 //max
#define SA_RGB_DURATION   0 //INFINITY!!!
#define SA_RGB_PERIOD     2000 //2 s
/*=======================MEDIUM CONFIG========================*/
#define MA_RGB_FUN_TYPE   FUN_TYPE_SIN_WAVE
#define MA_RGB_AMPLITUDE  63 //max
#define MA_RGB_DURATION   0 //INFINITY!!!
#define MA_RGB_PERIOD     1000 //1 s
#define MA_VIB_FUN_TYPE   FUN_TYPE_SIN_WAVE
#define MA_VIB_AMPLITUDE  50
#define MA_VIB_DURATION   0 //INFINITY!!!
#define MA_VIB_PERIOD     2000 //2 s
/*========================HARD CONFIG=========================*/
#define HA_RGB_FUN_TYPE   FUN_TYPE_SIN_WAVE
#define HA_RGB_AMPLITUDE  63 //max
#define HA_RGB_DURATION   0 //INFINITY!!!
#define HA_RGB_PERIOD     1000 //1 s
#define HA_VIB_FUN_TYPE   FUN_TYPE_SQUARE
#define HA_VIB_AMPLITUDE  63 //max
#define HA_VIB_DURATION   0 //INFINITY!!!
#define HA_VIB_PERIOD     1000 //1 s
#define HA_PWR_FUN_TYPE   FUN_TYPE_ON
#define HA_PWR_DURATION   0 //INFINITY!!!
#define HA_PWR_PERIOD     500 //0.5 s

static void priv_alarm_start(uint32_t timer){
  switch(alarm_info.type_of_alarm){
    case ALARM_SOFT:    priv_alarm_start_soft(timer); break;
    case ALARM_MEDIUM:  priv_alarm_start_medium(timer); break;
    case ALARM_HARD:    priv_alarm_start_hard(timer); break;
    default:            break;
  }
}

static void priv_alarm_start_soft(uint32_t timer){
  dev_set_new_parameters(RIGHT_RED_LED, SA_RGB_FUN_TYPE, SA_RGB_AMPLITUDE, SA_RGB_DURATION, SA_RGB_PERIOD, timer);
  dev_set_new_parameters(RIGHT_GREEN_LED, SA_RGB_FUN_TYPE, SA_RGB_AMPLITUDE, SA_RGB_DURATION, SA_RGB_PERIOD, timer);
  dev_set_new_parameters(RIGHT_BLUE_LED, SA_RGB_FUN_TYPE, SA_RGB_AMPLITUDE, SA_RGB_DURATION, SA_RGB_PERIOD, timer);
  dev_set_new_parameters(LEFT_RED_LED, SA_RGB_FUN_TYPE, SA_RGB_AMPLITUDE, SA_RGB_DURATION, SA_RGB_PERIOD, timer);
  dev_set_new_parameters(LEFT_GREEN_LED, SA_RGB_FUN_TYPE, SA_RGB_AMPLITUDE, SA_RGB_DURATION, SA_RGB_PERIOD, timer);
  dev_set_new_parameters(LEFT_BLUE_LED, SA_RGB_FUN_TYPE, SA_RGB_AMPLITUDE, SA_RGB_DURATION, SA_RGB_PERIOD, timer);
}

static void priv_alarm_start_medium(uint32_t timer){
  dev_set_new_parameters(RIGHT_RED_LED, MA_RGB_FUN_TYPE, MA_RGB_AMPLITUDE, MA_RGB_DURATION, MA_RGB_PERIOD, timer);
  dev_set_new_parameters(RIGHT_GREEN_LED, MA_RGB_FUN_TYPE, MA_RGB_AMPLITUDE, MA_RGB_DURATION, MA_RGB_PERIOD, timer);
  dev_set_new_parameters(RIGHT_BLUE_LED, MA_RGB_FUN_TYPE, MA_RGB_AMPLITUDE, MA_RGB_DURATION, MA_RGB_PERIOD, timer);
  dev_set_new_parameters(LEFT_RED_LED, MA_RGB_FUN_TYPE, MA_RGB_AMPLITUDE, MA_RGB_DURATION, MA_RGB_PERIOD, timer);
  dev_set_new_parameters(LEFT_GREEN_LED, MA_RGB_FUN_TYPE, MA_RGB_AMPLITUDE, MA_RGB_DURATION, MA_RGB_PERIOD, timer);
  dev_set_new_parameters(LEFT_BLUE_LED, MA_RGB_FUN_TYPE, MA_RGB_AMPLITUDE, MA_RGB_DURATION, MA_RGB_PERIOD, timer);
  dev_set_new_parameters(VIBRATOR, MA_VIB_FUN_TYPE, MA_VIB_AMPLITUDE, MA_VIB_DURATION, MA_VIB_PERIOD, timer);
}

static void priv_alarm_start_hard(uint32_t timer){
  dev_set_new_parameters(POWER_LED, HA_PWR_FUN_TYPE, 0, HA_PWR_DURATION, HA_PWR_PERIOD, timer);
  dev_set_new_parameters(RIGHT_RED_LED, HA_RGB_FUN_TYPE, HA_RGB_AMPLITUDE, HA_RGB_DURATION, HA_RGB_PERIOD, timer);
  dev_set_new_parameters(RIGHT_GREEN_LED, HA_RGB_FUN_TYPE, HA_RGB_AMPLITUDE, HA_RGB_DURATION, HA_RGB_PERIOD, timer);
  dev_set_new_parameters(RIGHT_BLUE_LED, HA_RGB_FUN_TYPE, HA_RGB_AMPLITUDE, HA_RGB_DURATION, HA_RGB_PERIOD, timer);
  dev_set_new_parameters(LEFT_RED_LED, HA_RGB_FUN_TYPE, HA_RGB_AMPLITUDE, HA_RGB_DURATION, HA_RGB_PERIOD, timer);
  dev_set_new_parameters(LEFT_GREEN_LED, HA_RGB_FUN_TYPE, HA_RGB_AMPLITUDE, HA_RGB_DURATION, HA_RGB_PERIOD, timer);
  dev_set_new_parameters(LEFT_BLUE_LED, HA_RGB_FUN_TYPE, HA_RGB_AMPLITUDE, HA_RGB_DURATION, HA_RGB_PERIOD, timer);
  dev_set_new_parameters(VIBRATOR, HA_VIB_FUN_TYPE, HA_VIB_AMPLITUDE, HA_VIB_DURATION, HA_VIB_PERIOD, timer);
}
