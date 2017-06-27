//---------------------------------------------------------------------------------------------------
/**
 * @file
 * @author Intelclinic Sp. z o.o.
 * @brief NeuroOn in consumer version main application
 * @version 2.0
 * @section LICENSE
 *
 * @section DESCRIPTION
 * Main application function for consumer version of NeuroOn mask.
 */
//---------------------------------------------------------------------------------------------------
#include <global_conf.h>
//---------------------------------------------------------------------------------------------------
#include "ic_hardware.h"
#include "ic_bluetooth.h"
//---------------------------------------------------------------------------------------------------
#include "ic_button.h"
#include "ic_charger.h"
#include "ic_timers.h"
#include "ic_state_machine.h"
#include "ic_state_test.h"
#include "ic_ltc3220.h"
#include "ic_flash_enclosed.h"
//---------------------------------------------------------------------------------------------------
static void start_needle_testtool(void);
//---------------------------------------------------------------------------------------------------


int main ()
{
  hardware_clockInit();
  sd_softdevice_vector_table_base_set(INT_FLASH_APP_BASE);
  ble_enableSoftDevice();
  hardware_gpioInit();
  button_init();

  spi_master_init();
  /** Interrupt enable */
  hardware_enableIrqGPIOTE();

  /** charger init*/
  charger_init();

  uart_init();

  //uint8_t i=0,j=0;
  STATE_MACHINE sm;


  WDT_RR();

  system_on_notify ();

  WDT_RR();

  //WEJŚCIE DO PROCEDUTY TESTUJĄCEJ PO UARCIE
  start_needle_testtool();
  // KONIEC WEJŚCIE DO PROCEDUTY TESTUJĄCEJ PO UARCIE

  SetBit(GPIO_POWER_DIGITAL);

  #ifndef NO_EX_FLASH
  while(!FLASH_ENCLOSED_init_after_restart());
  FLASH_ENCLOSED_raw_data_space_init();
  #endif

  ResetBit(GPIO_POWER_DIGITAL);
  ResetBit(GPIO_AFE_PDN);

  /*#define BQ_TEST*/
#ifdef BQ_TEST
  TWI_PIN_SELECT(GPIO_SCL_PIN, GPIO_SDA_PIN);
  TWI_Init(K400);
  ResetBit(GPIO_POWER_DIGITAL);
  ic_delay_ms(100);
  SetBit(GPIO_POWER_DIGITAL);
  bq27742_program_flash();
  while(1)
  {
    WDT_RR();
  }
#else

  init_state_machine(&sm);
  while(1) {
    __NOP();

    process_next_state(&sm);

    WDT_RR();
  }
#endif
}

//---------------------------------------------------------------------------------------------------
//---------- NEEDLE TESTTOOL SECTION -----------//
#define NEEDLE_TT_TIMEOUT 100
#define NEEDLE_TT_RDY_FLAG 0x10 // Announce that device is ready for needle tests by UART.
#define NEEDLE_TT_RSP_OK 0x11 // Tester ready response.

static void start_needle_testtool(void){
  uint8_t needle_tt_tout_cnt = 0;
  uint32_t needle_tt_start_tout = 0;

  while(needle_tt_tout_cnt < NEEDLE_TT_TIMEOUT){
    uart_send_u8(NEEDLE_TT_RDY_FLAG);
    needle_tt_start_tout = timer_get_ms_counter();

    while(timer_get_ms_counter() < needle_tt_start_tout && !NRF_UART0->EVENTS_RXDRDY){};

    if((uint8_t)NRF_UART0->RXD == NEEDLE_TT_RSP_OK && NRF_UART0->EVENTS_RXDRDY){
      NRF_UART0->EVENTS_RXDRDY = 0;
      state_test_init();
      state_test_loop();
      break;
    }else{
      needle_tt_tout_cnt++;
    }

    WDT_RR();
  }

  // Handle errors.
  if (NRF_UART0->EVENTS_ERROR != 0){
    NRF_UART0->EVENTS_ERROR = 0; // Clear UART ERROR event flag.
  }
}
//---------- UART TESTTOOL SECTION END -----------//
