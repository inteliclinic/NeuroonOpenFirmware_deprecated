/**
 * @file    ic_afe4400_api.h
 * @author  Paweł Kaźmierzewski <p.kazmierzewski@inteliclinic.com>
 * @author  Wojciech Węclewski <w.weclewski@inteliclinic.com>
 * @date    February, 2017
 * @brief   API for AFE4400 (command handling).
 *
 *  AFE4400 is a integrated analog front-end for pulse oximeter.
 */

#include <string.h>
#include "ic_afe4400_api.h"
#include "ic_afe4400.h"
#include "ic_rsp_emiter.h"
#include "ic_cc.h"

static void priv_afe4400_set_register(s_poxCmd *pox_cmd_data);
static void priv_afe4400_execute_function(e_poxFuncType function);
static void priv_afe4400_wrap_rsp_frame(s_poxRsp *pox_rsp_frame, uint16_t id, e_reqMode mode);
static void priv_afe4400_set_rsp_frame_reg_service(s_poxRsp *pox_rsp_frame, t_afe4400Register reg, t_afe4400RegisterConf reg_val);
static void priv_afe4400_set_rsp_frame_function(s_poxRsp *pox_rsp_frame, e_poxFuncType function);
static void priv_afe4400_set_rsp_frame_state_code(s_poxRsp *pox_rsp_frame, bool rsp_state_code);
static void priv_afe4400_emit_signal(s_poxRsp *pox_rsp_frame);

void afe4400_cmd_handle(s_poxCmd *pox_cmd_data){
  bool rsp_state_code = true;
  s_poxRsp pox_rsp_frame;
  memset(&pox_rsp_frame, 0, sizeof(s_poxRsp));

  priv_afe4400_wrap_rsp_frame(&pox_rsp_frame, pox_cmd_data->id, pox_cmd_data->mode);

  switch(pox_cmd_data->mode){
    case READ_REG:
      afe4400_read_selected_r(pox_cmd_data->request.reg_service.reg);
      priv_afe4400_set_rsp_frame_reg_service(&pox_rsp_frame, pox_cmd_data->request.reg_service.reg, afe4400_get_selected_r(pox_cmd_data->request.reg_service.reg));
      break;
    case WRITE_REG:
      priv_afe4400_set_register(pox_cmd_data);
      priv_afe4400_set_rsp_frame_reg_service(&pox_rsp_frame, pox_cmd_data->request.reg_service.reg, pox_cmd_data->request.reg_service.reg_val);
      break;
    case EXEC_FUNC:
      priv_afe4400_execute_function(pox_cmd_data->request.function);
      priv_afe4400_set_rsp_frame_function(&pox_rsp_frame, pox_cmd_data->request.function);
      break;
    default:
      rsp_state_code = false;
  }
  priv_afe4400_set_rsp_frame_state_code(&pox_rsp_frame, rsp_state_code);
  priv_afe4400_emit_signal(&pox_rsp_frame);
}

static void priv_afe4400_set_register(s_poxCmd *pox_cmd_data){
  switch(pox_cmd_data->request.reg_service.reg){
    case CONTROL0:
      afe4400_set_control0_r(pox_cmd_data->request.reg_service.reg_val);
      break;
    case CONTROL1:
      afe4400_set_control1_r(pox_cmd_data->request.reg_service.reg_val);
      break;
    case TIA_AMB_GAIN:
      afe4400_set_tia_amb_gain_r(pox_cmd_data->request.reg_service.reg_val);
      break;
    case LEDCNTRL:
      afe4400_set_ledcntrl_r(pox_cmd_data->request.reg_service.reg_val);
      break;
    case CONTROL2:
      afe4400_set_control2_r(pox_cmd_data->request.reg_service.reg_val);
      break;
    case ALARM:
      afe4400_set_alarm_r(pox_cmd_data->request.reg_service.reg_val);
      break;
    default: /*Other registers (timer module)*/
      afe4400_set_selected_timer_module_r(pox_cmd_data->request.reg_service.reg, pox_cmd_data->request.reg_service.reg_val);
  }
}

static void priv_afe4400_execute_function(e_poxFuncType function){
  switch(function){
    case HDW_INIT:
      afe4400_hdw_init();
      break;
    case STD_VAL_INIT:
      afe4400_std_val_init();
      break;
    case POWERDOWN_ON:
      afe4400_powerdown_on();
      break;
    case POWERDOWN_OFF:
      afe4400_powerdown_off();
      break;
    case SELF_TEST:
      afe4400_SelfTest();
      break;
  }
}

static void priv_afe4400_wrap_rsp_frame(s_poxRsp *pox_rsp_frame, uint16_t id, e_reqMode mode){
  pox_rsp_frame->id = id;
  pox_rsp_frame->mode = mode;
}

static void priv_afe4400_set_rsp_frame_reg_service(s_poxRsp *pox_rsp_frame, t_afe4400Register reg, t_afe4400RegisterConf reg_val){
  pox_rsp_frame->request.reg_service.reg = reg;
  pox_rsp_frame->request.reg_service.reg_val = reg_val;
}

static void priv_afe4400_set_rsp_frame_function(s_poxRsp *pox_rsp_frame, e_poxFuncType function){
  pox_rsp_frame->request.function = function;
}

static void priv_afe4400_set_rsp_frame_state_code(s_poxRsp *pox_rsp_frame, bool rsp_state_code){
  pox_rsp_frame->state_code = rsp_state_code;
}

static void priv_afe4400_emit_signal(s_poxRsp *pox_rsp_frame){
  add_rsp_frame((u_BLECmdPayload)*pox_rsp_frame);
  cc_emit(CC_PULSEOXIMETER_RSP, (uint32_t)get_rsp_frame);
}
