//---------------------------------------------------------------------------------------------------
/*
 * ic_timers.c
 *
 *  Created on: 8 wrz 2015
 *      Author: kn
 */
//---------------------------------------------------------------------------------------------------
#include "ic_timers.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "ic_uart_log.h"
#include "ic_ads.h"
#include "ic_eeg.h"
#include "ic_log.h"
#include "ic_pulse_engine.h"
#include "ic_bq27742.h"
#include "ic_flash_enclosed.h"
#include "ic_ltc3220.h"
#include "ic_pulse.h"
#include "ic_cc.h"

#include "ic_bluetooth.h"
//---------------------------------------------------------------------------------------------------
static timer_state timer2;
//---------------------------------------------------------------------------------------------------
bool section[7]={
		false, false, false, false, false, false, false
};

//---------------------------------------------------------------------------------------------------
#define RTC_PRESCALER   1023
#define RTC_CC_VAL(v)   (32768/(RTC_PRESCALER+1))*(v) /* Interrupt will occure every v second. */
static volatile struct {
  uint32_t rtc_timer;
  uint32_t ms_timer;
} systime;
//---------------------------------------------------------------------------------------------------
uint32_t timer1Counter=0;
// uint8_t buffor_for_flash[256]={1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20};
//---------------------------------------------------------------------------------------------------

//========================TEST VARIABLES=====================
volatile uint8_t battery_state;
//=====================END OF TEST VARIABLES=================
/**
 * @fn TIMER1_IRQHandler ()
 * @brief TIMER 1 interrupt handler for I2C & SPI communication queuing (EEG + PULSE + SATURATION + ACC + FLASH)
 */

//----------------------------------------------- AD ------------------------------------------------
static uint32_t timer1_ref_capture = 0;
static uint32_t timer1_new_capture = 0;
static uint32_t timer1_2Hz_counter = 0;
static uint8_t cycles_2Hz = 0;
//---------------------------------------------------------------------------------------------------
static s_int32_timestamp redSample;
static s_int32_timestamp irSample;
static s_acc_data_timestamp accSample = {
  .timestamp = 0,
  .data.ACC_X = 0,
  .data.ACC_Y = 0,
  .data.ACC_Z = 0
};

static int8_t temp1, temp2;

#define EEG_FIFO_SIZE   16
static s_int16_timestamp eeg_fifo[EEG_FIFO_SIZE];
static uint8_t eeg_fifo_tail = 0;
static uint8_t eeg_fifo_ptr = 0;

uint32_t timer_get_ms_counter()
{
  NRF_TIMER1->TASKS_CAPTURE[2] = 1;
  return systime.ms_timer + (NRF_TIMER1->CC[2]/125);
}


s_int16_timestamp get_eegSample()
{
  s_int16_timestamp ret_val = eeg_fifo[eeg_fifo_ptr++];
  if(eeg_fifo_ptr == EEG_FIFO_SIZE) eeg_fifo_ptr = 0;
  if(eeg_fifo_ptr != eeg_fifo_tail)
    cc_emit(CC_EEG_MEASURED, (uint32_t)get_eegSample);
  return ret_val;
}

void add_eegSample(int16_t s, uint32_t time)
{
  if(eeg_fifo_ptr == eeg_fifo_tail)
    cc_emit(CC_EEG_MEASURED, (uint32_t)get_eegSample);
  eeg_fifo[eeg_fifo_tail].timestamp = time;
  eeg_fifo[eeg_fifo_tail++].data = s;
  if(eeg_fifo_tail == EEG_FIFO_SIZE) eeg_fifo_tail = 0;
}

s_int32_timestamp get_redSample()
{
  return redSample;
}

s_int32_timestamp get_irSample()
{
  return irSample;
}

s_acc_data_timestamp get_accSample(void){
  return accSample;
}

uint8_t get_dummy(){
  return 0xEE;
}

uint16_t get_temp(){
  return (temp1&0xFF)|((temp2<<8)&0xFF00);
}

void TIMER1_IRQHandler ()
{
  if(NRF_TIMER1->EVENTS_COMPARE[0]){
    NRF_TIMER1->EVENTS_COMPARE[0] = 0;
    systime.ms_timer += 4;
    timer1Counter++;
    if (timer_getSection(SECTION_EEG)&&timer1Counter%2==1) {//EEG SECTION
      int16_t eegSample = ads_get_value();
      add_eegSample(eegSample,systime.ms_timer);
      /*cc_emit(CC_EEG_MEASURED,(uint32_t)get_eegSample);*/
      eeg_addSample(eegSample);
    }
    if (timer_getSection(SECTION_PULSE)&&timer1Counter%10==0){//PULSOXIMETER
      afe4400_TakeData_LED2(&irSample.data);
      afe4400_TakeData_LED1(&redSample.data);
      irSample.timestamp = redSample.timestamp = systime.ms_timer;

      cc_emit(CC_RED_MEASURED,(uint32_t)get_redSample);
      cc_emit(CC_IR_MEASURED,(uint32_t)get_irSample);
      //PE_AddSample(sample);
      pulse_addSample(irSample.data);
      log_redled_byfifo(redSample.data);
      //pulse_addSample((int32_t)10000.0*sinf(2*M_PI/125*k/6));
    }
    if (timer_getSection(SECTION_TEMP)&&timer1Counter%125==64){//TEMPERATURE
      analog_thermometer_IRQHandler();
      uint16_t v;
      if (analog_thermometer_getLastResult(1,&v))
        temp1 = (v>>2)&0xFF;
      if (analog_thermometer_getLastResult(2,&v))
        temp2 = (v>>2)&0xFF;
      cc_emit(CC_TEMP_MEASURED, (uint32_t)get_temp);
    }
    if (timer_getSection(SECTION_ACC)&&timer1Counter%10==4){//ACCELEROMETER
      accSample.data = MOVE_IRQHandler();
      accSample.timestamp = systime.ms_timer;
      cc_emit (CC_ACC_MEASURED, (uint32_t)get_accSample);
    }
    if (timer_getSection(SECTION_BATTERY)&&timer1Counter%2500==1249){//BQ Battery State
      /*battery_state=bq_battery_measurement();*/
      cc_emit (CC_BATTERY_MEASURE, (uint32_t)get_dummy);
      //BQ HAS CLOCK STRECHING; IT'S HIGHLY RECOMMEND TO AVOID IT DURING MEASUREMENTS
    }

    //AD powerled
    if (((cycles_2Hz == 0) || (cycles_2Hz == 2)) && (timer1Counter%125 == 62)){
      timer1_2Hz_counter++;
      cycles_2Hz++;

    }else if ((cycles_2Hz==1) && (timer1Counter%125==124)) {
      timer1_2Hz_counter++;
      cycles_2Hz++;

    }else if ((cycles_2Hz==3) && (timer1Counter%125==0)) {
      timer1_2Hz_counter++;
      cycles_2Hz++;
    }

    cycles_2Hz = cycles_2Hz%4; // [0..3] values allowed

    //eeg_computeNextSample();
    //ResetBit(19);

  }



  //NRF_TIMER1->TASKS_CAPTURE[1] = 1;
  //log_bustimer_byfifo((uint16_t)(NRF_TIMER1->CC[1]));
}

uint32_t get_timer1Counter(void)
{
	return timer1Counter;
}
//---------------------------------------------------------------------------------------------------
void timer_busInit(unsigned int  freq)
{
//	BUS_I2C_SPI_DRIVER_TIM->MODE = TIMER_MODE_MODE_Timer << TIMER_MODE_MODE_Pos;
//	BUS_I2C_SPI_DRIVER_TIM->PRESCALER = ADS_ET_GEN_DRIVER_ADS_TIM_PRESC;
//	BUS_I2C_SPI_DRIVER_TIM->BITMODE = TIMER_BITMODE_BITMODE_16Bit << TIMER_BITMODE_BITMODE_Pos;
//	BUS_I2C_SPI_DRIVER_TIM->CC[ADS_ET_GEN_DRIVER_ADS_TIM_CH] = ADS_ET_GEN_DRIVER_ADS_TIM_CC;
//	BUS_I2C_SPI_DRIVER_TIM->SHORTS =(1<<ADS_ET_GEN_DRIVER_ADS_TIM_CH);

 	//BUS_I2C_SPI_DRIVER_TIM->TASKS_STOP = 1;
	//BUS_I2C_SPI_DRIVER_TIM->TASKS_CLEAR = 1;

	BUS_I2C_SPI_DRIVER_TIM->MODE = TIMER_MODE_MODE_Timer << TIMER_MODE_MODE_Pos;
	BUS_I2C_SPI_DRIVER_TIM->BITMODE = TIMER_BITMODE_BITMODE_16Bit << TIMER_BITMODE_BITMODE_Pos;

	if (freq == 125) {
		BUS_I2C_SPI_DRIVER_TIM->PRESCALER = 7;
		BUS_I2C_SPI_DRIVER_TIM->CC[0] = 1000;
                /*BUS_I2C_SPI_DRIVER_TIM->CC[1] = 125;*/
	} else if (freq == 200) {
		BUS_I2C_SPI_DRIVER_TIM->PRESCALER = 7;
		BUS_I2C_SPI_DRIVER_TIM->CC[0] = 625;
	} else if (freq == 250) {
		BUS_I2C_SPI_DRIVER_TIM->PRESCALER = 7;
		BUS_I2C_SPI_DRIVER_TIM->CC[0] = 500;
	} else {
		BUS_I2C_SPI_DRIVER_TIM->TASKS_SHUTDOWN = 1;
		return;
	}
        BUS_I2C_SPI_DRIVER_TIM->SHORTS = (TIMER_SHORTS_COMPARE0_CLEAR_Enabled << TIMER_SHORTS_COMPARE0_CLEAR_Pos);
	BUS_I2C_SPI_DRIVER_TIM->EVENTS_COMPARE[0] = 0;

	BUS_I2C_SPI_DRIVER_TIM->INTENSET = (TIMER_INTENSET_COMPARE0_Enabled<<TIMER_INTENSET_COMPARE0_Pos);

	//sd_nvic_ClearPendingIRQ(TIMER1_IRQn);
	sd_nvic_SetPriority(TIMER1_IRQn, APP_IRQ_PRIORITY_LOW);
	sd_nvic_EnableIRQ(TIMER1_IRQn);

	timer1Counter = 0;
	timer1_2Hz_counter = 0; // AD

	BUS_I2C_SPI_DRIVER_TIM->TASKS_START = 1;
}
//---------------------------------------------------------------------------------------------------
/**
 * @brief deinit timer measurement module
 */
void timer_busDeinit ()
{
	timer_measurement_stop();
	sd_nvic_DisableIRQ(TIMER1_IRQn);

	BUS_I2C_SPI_DRIVER_TIM->TASKS_STOP = 1;
	BUS_I2C_SPI_DRIVER_TIM->TASKS_CLEAR = 1;
	BUS_I2C_SPI_DRIVER_TIM->MODE = 0;
	BUS_I2C_SPI_DRIVER_TIM->PRESCALER = 0;
	BUS_I2C_SPI_DRIVER_TIM->BITMODE = 0;
	BUS_I2C_SPI_DRIVER_TIM->SHORTS = 0;
}
//---------------------------------------------------------------------------------------------------
void timer_startSection(SECTION_INDEX ind) {
	switch(ind){
	case SECTION_EEG:
		section[0] = true;
		break;
	case SECTION_PULSE:
		section[1] = true;
		break;
	case SECTION_TEMP:
		section[2] = true;
		break;
	case SECTION_ACC:
		section[3] = true;
		break;
	case SECTION_FLASH_WRITE:
		section[4] = true;
		break;
	case SECTION_BATTERY:
		section[5] = true;
		break;
	case SECTION_LTC_REFRESH:
		section[6] = true;
		break;
	}
}
//---------------------------------------------------------------------------------------------------
void timer_stopSection(SECTION_INDEX ind) {
	switch(ind){
	case SECTION_EEG:
		section[0] = false;
		break;
	case SECTION_PULSE:
		section[1] = false;
		break;
	case SECTION_TEMP:
		section[2] = false;
		break;
	case SECTION_ACC:
		section[3] = false;
		break;
	case SECTION_FLASH_WRITE:
		section[4] = false;
		break;
	case SECTION_BATTERY:
		section[5] = false;
		break;
	case SECTION_LTC_REFRESH:
		section[6] = false;
		break;
	}
}
//---------------------------------------------------------------------------------------------------
bool timer_getSection(SECTION_INDEX ind) {
	switch(ind){
	case SECTION_EEG:
		return section[0];
		break;
	case SECTION_PULSE:
		return section[1];
		break;
	case SECTION_TEMP:
		return section[2];
		break;
	case SECTION_ACC:
		return section[3];
		break;
	case SECTION_FLASH_WRITE:
		return section[4];
		break;
	case SECTION_BATTERY:
		return section[5];
		break;
	case SECTION_LTC_REFRESH:
		return section[6];
		break;
	}
	return false;
}
//---------------------------------------------------------------------------------------------------
void RTC1_IRQHandler(void){
  NRF_RTC1->TASKS_CLEAR = 1;
  NRF_RTC1->EVENTS_COMPARE[0] = 0;

  systime.rtc_timer++;
  /*ChangeBit(20);*/
}
/**
 * @fn TIMER2_IRQHandler()
 * @brief handles LEDs and Vibrations
 */
uint8_t button_charger_cnt = 0;
void TIMER2_IRQHandler()
{
	bool tmp=false;
	if(LED_DRIVER_TIM->EVENTS_COMPARE[LED_DRIVER_TIM_CC_CH]) {
		LED_DRIVER_TIM->EVENTS_COMPARE[LED_DRIVER_TIM_CC_CH] = 0;

                /*ChangeBit(20);*/
                  button_charger_cnt = 0;
                  //BUTTON LOGIC
                  tmp = button_timer_irq_handler();
                  //END OF BUTTON LOGIC

                  //CHARGER LOGIC
                  tmp |= charger_timer_irq_handler();
                  if (tmp==false)
                    timer_led_driver_hard_deinit();
                  //END OF CHARGER LOGIC
	}
}
//---------------------------------------------------------------------------------------------------
/**
 *	timer initialization for LED/Vibra driver and button / cahrger check
 */
void timer_led_driver_init() {

	if (get_LED_timer_state()->init_flag==0){
                LED_DRIVER_TIM->SHORTS = 1 << LED_DRIVER_TIM_CC_CH;
		LED_DRIVER_TIM->INTENSET = 1<<(TIMER_INTENSET_COMPARE0_Pos +  LED_DRIVER_TIM_CC_CH);
		LED_DRIVER_TIM->BITMODE = TIMER_BITMODE_BITMODE_16Bit;
		LED_DRIVER_TIM->PRESCALER = LED_DRIVER_TIM_PRESC;
		LED_DRIVER_TIM->CC[LED_DRIVER_TIM_CC_CH] = LED_DRIVER_TIM_CC;
		LED_DRIVER_TIM->EVENTS_COMPARE[LED_DRIVER_TIM_CC_CH] = 0;

		uint32_t err = 0;
		err = sd_nvic_SetPriority(LED_DRIVER_TIM_IRQn, APP_IRQ_PRIORITY_LOW);
		APP_ERROR_CHECK(err);
		err = sd_nvic_EnableIRQ(LED_DRIVER_TIM_IRQn);
		APP_ERROR_CHECK(err);
		get_LED_timer_state()->init_flag=1;
		get_LED_timer_state()->how_many_users=1;
		timer_led_driver_start();
	}
	else
		get_LED_timer_state()->how_many_users++;
}
//---------------------------------------------------------------------------------------------------
/**
 * Disable timer interrupt
 */
void timer_led_driver_deinit()
{
	get_LED_timer_state()->how_many_users--;
	if (get_LED_timer_state()->how_many_users==0){
		timer_led_driver_stop();
		LED_DRIVER_TIM->INTENCLR = 1<<(TIMER_INTENSET_COMPARE0_Pos +  LED_DRIVER_TIM_CC_CH);
		sd_nvic_DisableIRQ(LED_DRIVER_TIM_IRQn);
		get_LED_timer_state()->init_flag=0;
	}
}

void timer_led_driver_hard_deinit()
{
	get_LED_timer_state()->how_many_users=0;
	timer_led_driver_stop();
	LED_DRIVER_TIM->INTENCLR = 1<<(TIMER_INTENSET_COMPARE0_Pos +  LED_DRIVER_TIM_CC_CH);
	sd_nvic_DisableIRQ(LED_DRIVER_TIM_IRQn);
	get_LED_timer_state()->init_flag=0;
}


//---------------------------------------------------------------------------------------------------
void timer_RTC_init(void){
		NRF_RTC1->TASKS_STOP = 1;
		ic_delay_ms(1);
		NRF_RTC1->TASKS_CLEAR = 1;
		ic_delay_ms(1);

		systime.rtc_timer = 0;
	    NRF_RTC1->PRESCALER = RTC_PRESCALER;
	    NRF_RTC1->CC[0] = RTC_CC_VAL(1);
	    NRF_RTC1->INTENSET = RTC_INTENSET_COMPARE0_Msk;

		NRF_RTC1->EVENTS_COMPARE[0] = 0;
	    NVIC_SetPriority(RTC1_IRQn, APP_IRQ_PRIORITY_LOW);
	    NVIC_ClearPendingIRQ(RTC1_IRQn);
	    NVIC_EnableIRQ(RTC1_IRQn);
	    NRF_RTC1->TASKS_START = 1;
}
//---------------------------------------------------------------------------------------------------
void timer_RTC_deinit(void){
	NVIC_DisableIRQ(RTC1_IRQn);

    NRF_RTC1->EVTENCLR = RTC_EVTEN_COMPARE0_Msk;
    NRF_RTC1->INTENCLR = RTC_INTENSET_COMPARE0_Msk;

    NRF_RTC1->TASKS_STOP = 1;
    ic_delay_ms(1);

    NRF_RTC1->TASKS_CLEAR = 1;
    ic_delay_ms(1);
}
//---------------------------------------------------------------------------------------------------
uint32_t timer_get_RTC_counter(void){
  return systime.rtc_timer;
}

//---------------------------------------------- AD ------------------------------------------------

uint32_t timer1_capture(){

	NRF_TIMER1->TASKS_CAPTURE[3] = 1;
	return NRF_TIMER1->CC[3]&0xFFFF; // Read 16 lsb of CC[3] register (16 us/bit)
}


bool timer1_capture_elapsed(uint32_t threshold){

	// time elapsed = timer1Counter * 8 [ms] + CC[3] * 0.016 [ms]
	timer1_new_capture = (timer1Counter<<3) + (timer1_capture()>>6);

	if ((timer1_new_capture - timer1_ref_capture) >= threshold){	// time elapsed since last capture above threshold
		timer1_ref_capture = timer1_new_capture; 					// Set this capture as reference
		return 1;
	}else{
		return 0;
	}
}

void timer1_update_reference(){
	// timer1_ref_capture = timer1Counter * 8 [ms] + CC[3] * 0.016 [ms]
	timer1_ref_capture = (timer1Counter<<3) + (timer1_capture()>>6);
}

uint32_t timer1_2Hz_get(){
	return timer1_2Hz_counter;
}

void timer1_2Hz_clear(){
	timer1_2Hz_counter = 0;
}


//---------------------------------------------------------------------------------------------------


//---------------------------------------------PRIVATE-----------------------------------
timer_state_p get_LED_timer_state(){
	return &timer2;
}
void timer_led_driver_start() {
	LED_DRIVER_TIM->TASKS_START = 1;
}
void timer_led_driver_stop() {
	LED_DRIVER_TIM->TASKS_STOP = 1;
}

void timer_measurement_start ()
{
}

void timer_measurement_stop ()
{
	BUS_I2C_SPI_DRIVER_TIM->TASKS_STOP = 1;
	BUS_I2C_SPI_DRIVER_TIM->INTENCLR = (1<<(TIMER_INTENCLR_COMPARE0_Pos));
}
