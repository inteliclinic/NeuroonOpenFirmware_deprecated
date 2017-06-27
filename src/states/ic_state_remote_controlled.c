/**
 * @file    ic_state_remote_controlled.h
 * @author  Paweł Kaźmierzewski <p.kazmierzewski@inteliclinic.com>
 * @author  Wojciech Weclewski <w.weclewski@inteliclinic.com>
 * @date    July, 2016
 * @brief   State for Neuroon control.
 *
 *  Neuroon is in this state right after pairing.
 *
 */

#include <stdio.h>
#include <string.h>
#include "nrf.h"
#include "ic_state_remote_controlled.h"
#include "ic_bluetooth.h"
#include "ic_cli.h"
#include "ic_delay.h"
#include "ic_eeg.h"
#include "ic_pulse_engine.h"
#include "ic_timers.h"
#include "ic_cc.h"
#include "ic_frame_handle.h"
#include "ic_low_level_control.h"
#include "ic_device_handle_api.h"
#include "ic_device_driver_api.h"
#include "ic_e_alarm_api.h"
#include "ic_afe4400_api.h"
#include "ic_uart_log.h"
#include "ic_ltc_devices_cli_procedures.h"
#include "ic_afe4400_cli_procedures.h"
#include "ic_e_alarm_cli_procedures.h"
#include "ic_mixed_cli_procedures.h"

/** Useful macros for DFU enter*/
#define IRQ_ENABLED             			0x01
#define MAX_NUMBER_INTERRUPTS   			32
#define GPREGRET_DFU_START_CMD				0x80

typedef struct{
  enum{
    SDS_IR_MEASURED,
    SDS_RED_MEASURED,
    SDS_ACC_MEASURED,
    SDS_TEMP_MEASURED
  }type;

  union{
    s_acc_data_timestamp acc;
    s_int32_timestamp ir_sample;
    s_int32_timestamp red_sample;
    uint16_t  temp_sample;
  }data;
}s_stream1DataSet;


static void disableInterrupts(uint32_t mask) {
  uint32_t interrupt_setting_mask;
  uint8_t  irq = 0;

  /// Fetch the current interrupt settings.
  interrupt_setting_mask = NVIC->ISER[0];

  for (; irq < MAX_NUMBER_INTERRUPTS; irq++)
  {
    if ((interrupt_setting_mask & (IRQ_ENABLED << irq)) &&  (mask & (IRQ_ENABLED << irq)))
    {
      NVIC_DisableIRQ((IRQn_Type) irq);
    }
  }
}

static void send_ble_data_stream0(s_int16_timestamp data){
  static uint8_t eeg_data_ptr = 0;
  static u_eegDataFrameContainter stream0_output_frame;

  if(eeg_data_ptr == 0){
    stream0_output_frame.frame.time_stamp = data.timestamp;
  }

  stream0_output_frame.frame.eeg_data[eeg_data_ptr++] = data.data;
  if(eeg_data_ptr >= 8){
    eeg_data_ptr = 0;
    if (ble_isCentralConnected()){
      ble_sendDataPacketStream0(stream0_output_frame.raw_data, 20);
    }
  }
}

static void send_ble_data_stream1(s_stream1DataSet *data){
  static bool ir_ready  = false;
  static bool red_ready = false;
  static bool acc_ready = false;

  static u_otherDataFrameContainer stream1_output_frame;

  if(ir_ready == false && red_ready == false && acc_ready == false){
    stream1_output_frame.frame.time_stamp = timer_get_ms_counter();
    switch(data->type){
      case SDS_ACC_MEASURED:
        stream1_output_frame.frame.time_stamp = data->data.acc.timestamp;
        break;
      case SDS_IR_MEASURED:
        stream1_output_frame.frame.time_stamp = data->data.ir_sample.timestamp;
        break;
      case SDS_RED_MEASURED:
        stream1_output_frame.frame.time_stamp = data->data.red_sample.timestamp;
        break;
      default:
        break;
    }
  }

  switch (data->type){
    case SDS_IR_MEASURED:
      ir_ready = true;
      stream1_output_frame.frame.ir_sample =data->data.ir_sample.data;
      break;
    case SDS_RED_MEASURED:
      red_ready = true;
      stream1_output_frame.frame.red_sample =data->data.red_sample.data;
      break;
    case SDS_ACC_MEASURED:
      acc_ready = true;
      *(acc_data *)&stream1_output_frame.frame.acc = data->data.acc.data;
      break;
    case SDS_TEMP_MEASURED:
      stream1_output_frame.frame.temp[0] = data->data.temp_sample&0xFF;
      stream1_output_frame.frame.temp[1] = (data->data.temp_sample>>8)&0xFF;
  }

  if (ir_ready&&red_ready&&acc_ready){
    ir_ready = red_ready = acc_ready = false;

    if (ble_isCentralConnected()){
      ble_sendDataPacketStream1(stream1_output_frame.raw_data,20);
    }
  }
}

static void send_ble_response(u_BLECmdPayload response_payload, e_cmd cmd_type){
  char response_output_frame[sizeof(u_cmdFrameContainer)];
  size_t len = sizeof(u_cmdFrameContainer);
  bool ret_val = false;

  ret_val = resp_frame_copy_func(response_output_frame, &len, (char*)&response_payload, cmd_type);

  for(uint8_t i = 0; i < len; i++) {
    print_cli("0x%02X ", (uint8_t)response_output_frame[i]);
  }
  print_cli("\n\r");

  if (ble_isCentralConnected() && ret_val){
    ble_sendResponse((uint8_t*)response_output_frame, (uint16_t)len);
  }
}

static void send_ble_status(uint16_t status_cmd_id){
  char status_output_frame[sizeof(u_cmdFrameContainer)];
  size_t len = sizeof(u_cmdFrameContainer);
  bool ret_val = false;
  s_devsFunc current_devs_func = dev_get_all_functions();
  uint8_t active_data_streams = 0; /*TODO: there is no such funcionality for now*/

  ret_val = status_rsp_gen_func(status_output_frame, &len, current_devs_func, active_data_streams, status_cmd_id);

  for(uint8_t i = 0; i < len; i++) {
    print_cli("0x%02X ", (uint8_t)status_output_frame[i]);
  }
  print_cli("\n\r");
  if (ble_isCentralConnected() && ret_val){
    ble_sendDataPacketStream2((uint8_t*)status_output_frame, (uint16_t)len);
  }
}

static void probe_signals(u_ccDataAccessFunction *fp){
  s_deviceCmd device_cmd;
  s_poxCmd pox_cmd;
  s_alarmCmd alarm_cmd;
  uint8_t battery_state;
  s_stream1DataSet stream1_data;

  switch (cc_probe(fp)){
    case CC_NO_DATA:
    case CC_RGB_CMD:
    case CC_POWER_CMD:
    case CC_VIBRA_CMD:
      break;
    case CC_CLI_PARSE:
      parse_command();
      break;
    case CC_DEVICE_CMD:
      device_cmd = fp->get_device_cmd();
      dev_all_set_new_parameters(&device_cmd);
      break;
    case CC_DEVICE_RSP:
      send_ble_response(fp->get_rsp_frame(), DEVICE_CMD);
      break;
    case CC_PULSEOXIMETER_CMD:
      pox_cmd = fp->get_pox_cmd();
      afe4400_cmd_handle(&pox_cmd);
      break;
    case CC_PULSEOXIMETER_RSP:
      send_ble_response(fp->get_rsp_frame(), PULSEOXIMETER_CMD);
      break;
    case CC_E_ALARM_CMD:
      alarm_cmd = fp->get_alarm_cmd();
      alarm_cmd_handle(&alarm_cmd);
      break;
    case CC_E_ALARM_RSP:
      send_ble_response(fp->get_rsp_frame(), E_ALARM_CMD);
      break;
    case CC_STATUS_CMD:
      send_ble_status(fp->get_status_cmd_id());
      break;
    case CC_EEG_MEASURED:
      send_ble_data_stream0(fp->get_i16_time());
      break;
    case CC_IR_MEASURED:
      stream1_data.type = SDS_IR_MEASURED;
      stream1_data.data.ir_sample = fp->get_i32_time();
      send_ble_data_stream1(&stream1_data);
      break;
    case CC_RED_MEASURED:
      stream1_data.type = SDS_RED_MEASURED;
      stream1_data.data.red_sample = fp->get_i32_time();
      send_ble_data_stream1(&stream1_data);
      break;
    case CC_ACC_MEASURED:
      stream1_data.type = SDS_ACC_MEASURED;
      stream1_data.data.acc = fp->get_acc_data_time();
      send_ble_data_stream1(&stream1_data);
      break;
    case CC_TEMP_MEASURED:
      stream1_data.type = SDS_TEMP_MEASURED;
      stream1_data.data.temp_sample = fp->get_u16();
      send_ble_data_stream1(&stream1_data);
      break;
    case CC_BLE_CONNECTED:
      bye_procedure();
      break;
    case CC_BLE_DISCONNECTED:
      hello_procedure();
      break;
    case CC_UPDATE_POWER:
      drv_power_led_set_val(fp->dev_power_led_get_function());
      break;
    case CC_UPDATE_RR_LED:
      drv_dev_set_val(fp->dev_get_selected_output(RIGHT_RED_LED));
      break;
    case CC_UPDATE_RG_LED:
      drv_dev_set_val(fp->dev_get_selected_output(RIGHT_GREEN_LED));
      break;
    case CC_UPDATE_RB_LED:
      drv_dev_set_val(fp->dev_get_selected_output(RIGHT_BLUE_LED));
      break;
    case CC_UPDATE_LR_LED:
      drv_dev_set_val(fp->dev_get_selected_output(LEFT_RED_LED));
      break;
    case CC_UPDATE_LG_LED:
      drv_dev_set_val(fp->dev_get_selected_output(LEFT_GREEN_LED));
      break;
    case CC_UPDATE_LB_LED:
      drv_dev_set_val(fp->dev_get_selected_output(LEFT_BLUE_LED));
      break;
    case CC_UPDATE_VIBRATOR:
      drv_dev_set_val(fp->dev_get_selected_output(VIBRATOR));
      break;
    case CC_DFU_CMD:
      if (fp->get_bool()==true){
        ble_disableRadioCommunication();
        //Go into DFU state ... black hole (system reset)
        sd_power_gpregret_clr(0xFF);
        sd_power_gpregret_set(GPREGRET_DFU_START_CMD);
        disableInterrupts(0xFFFFFFFF);
        NVIC_SystemReset();
      }
      break;
    case CC_BATTERY_MEASURE:
      battery_state = bq_battery_measurement();
      ble_sendBatteryState(&battery_state, 1);
      break;
    case CC_NO_OF_SIGNALS:
      break;
  }
}

void cli_bq_program(int c, int *args) {
  print_cli("Programming bq... ");
  bq27742_program_flash();
}
void cli_bq_battery(int c, int *args) {
  uint16_t params[4];
  bq_battery_cap_params(params);
  print_cli(
      "\n\rRemaining cap: %d\n\rFull charge cap: %d\n\r"\
      "Nominal available cap: %d\n\rFull available cap: %d\n\r",
      params[0], params[1], params[2], params[3]);
  print_cli("battery state is: %d%%\n\r", 100*params[0]/params[1]);
}

static void HARDware_init(void){
  cli_init();
  register_cmd("rgb", cli_rgb_led_set_func);
  register_cmd("vib", cli_vibra_set_func);
  register_cmd("rgb_ramp", cli_rgb_ramp_func);
  register_cmd("vib_ramp", cli_vibra_ramp_func);
  register_cmd("vibon", cli_vibra_ON);
  register_cmd("pwr", cli_pwr_led_set_func);
  register_cmd("dev_demo", cli_signal_collision_demo_func);
  register_cmd("bqp", cli_bq_program);
  register_cmd("bat", cli_bq_battery);
  register_cmd("pox_hdw_init", cli_afe4400_hdw_init);
  register_cmd("pox_std_init", cli_afe4400_std_val_init);
  register_cmd("pox_pd_on", cli_afe4400_powerdown_on);
  register_cmd("pox_pd_off", cli_afe4400_powerdown_off);
  register_cmd("pox_set_r", cli_afe4400_set_selected_r);
  register_cmd("pox_get_r", cli_afe4400_get_read_selected_r);
  register_cmd("pox_diag", cli_afe4400_self_test);
  register_cmd("alarm_set", cli_alarm_set);
  register_cmd("alarm_off", cli_alarm_off);
  register_cmd("mix_dev", cli_signal_collision_demo_func);
  register_cmd("mix_rsp", cli_different_rsp_check_func);
  register_cmd("mix_status", cli_get_status);
  /*IF TWI/I2C NEEDED (BQ, LTC, ADS, LIS)*/
  TWI_PIN_SELECT(GPIO_SCL_PIN, GPIO_SDA_PIN);
  TWI_Init(K400);
  /*IF YOU USE ANY DIGITAL INTERFACES or TEMP_MEASURE*/
  SetBit(GPIO_POWER_DIGITAL);
  /*IF YOU USE LEDS*/
  SetBit(GPIO_LEDS_ON);

  ltc3220_clear_async();

  SetBit(GPIO_POWER_ANALOG);
  ads_init();
  eeg_clearModule();

  /* PULSOXIMETER */
  PE_ClearAndInit();

  /** AFE4400 init */
  SetBit(GPIO_AFE_PDN);
  afe4400_hdw_init();
  afe4400_std_val_init();

  /* THERMOMETER */
  analog_thermometer_init();

  while(!MOVE_init());
  MOVE_Start();

  /*IF YOU NEED RTC IRQ*/
  timer_RTC_init();

  timer_startSection(SECTION_EEG);
  timer_startSection(SECTION_PULSE);
  timer_startSection(SECTION_TEMP);
  timer_startSection(SECTION_ACC);
  timer_startSection(SECTION_BATTERY);

  /*HIGHLY RECOMMENDED IN ALL INIT SECTIONS*/
  button_clear_state();
  charger_init();

  alarm_module_init();
}

static void HARDware_deinit(){
  cli_deinit();
  timer_busDeinit();
  /*CHOOSE USED Sections*/
  timer_stopSection(SECTION_EEG);
  timer_stopSection(SECTION_PULSE);
  timer_stopSection(SECTION_TEMP);
  timer_stopSection(SECTION_ACC);
  timer_stopSection(SECTION_BATTERY);


  /*IF YOU USED LEDS (LTC)*/
  ResetBit(GPIO_LEDS_ON);

  /*IF YOU USED LEDS (LTC)*/
  ltc3220_clear_async();

  ic_delay_ms(3);
  ResetBit(GPIO_LEDS_ON);
  ic_delay_ms(2);

  /*IF YOU USE ADS (EEG_SIGNAL)*/
  ResetBit(GPIO_POWER_ANALOG);
  ads_deinit();

  /*IF YOU USED PULSOXIMETER */
  afe4400_stop();


  /*IF YOU USED THERMOMETER */
  analog_thermometer_deinit();

  /*IF YOU USED ACCELEROMETER */
  MOVE_Stop();


  /*IF RTC IRQ WAS USED*/
  timer_RTC_deinit();

  /*IF TWI/I2C WAS USED (BQ, LTC, ADS, LIS)*/
  TWI_Deinit();
  /*IF YOU USED ANY DIGITAL INTERFACES or TEMP_MEASURE*/
  ResetBit(GPIO_POWER_DIGITAL);

  /*HIGHLY RECOMMENDED IN ALL DEINIT SECTIONS*/
  button_clear_state();
  charger_init();

}

#define TEST_TOOL_CMD_SIZE 20 /*TODO: that crap have to be used (FT Tester needs it)*/
state_exit_code state_rc(void){
  state_exit_code ret_value;
  u_ccDataAccessFunction fp;
  uint8_t test_tool_ble_buffer[TEST_TOOL_CMD_SIZE] = { 0 }; /*TODO: that crap have to be used (FT Tester needs it)*/
  size_t tt_ble_buff_len = sizeof(test_tool_ble_buffer);

  HARDware_init();

  LIS3DH_INT1_Config();

  dev_all_handlers_register();

  hello_procedure();

  ble_enableRadioCommunication();

  timer_busInit(250);

  while (1){
    probe_signals(&fp);

    /*==============TESTS by CLI=================*/
    cli_signal_collision_demo_update();
    cli_different_rsp_check_update();
    /*============end TESTS by CLI===============*/

    dev_all_update_when_active();
    drv_all_dev_update(timer_get_ms_counter());
    alarm_update(ble_isCentralConnected(), timer_get_RTC_counter(), timer_get_ms_counter());

    if(ble_isCentralConnected() && ble_getTestToolCmd(test_tool_ble_buffer, tt_ble_buff_len)){ /*TODO: that crap must be used (FT Tester needs it)*/
      return RC_SM_MESSAGE_TESTTOOL;
    }
    if (button_is_pressed()){
      button_clear_state();
      ret_value = RC_SM_BUTTON_PRESSED;
      break;
    }
    if (charger_is_plugged()){
      ret_value = RC_SM_CHARGER_PLUGGED;
      break;
    }
    WDT_RR();
  }

  dev_unregister_all();
  ble_disableRadioCommunication();
  bye_procedure();
  HARDware_deinit();
  return ret_value;
}
