//---------------------------------------------------------------------------------------------------
#include "ic_uart_log.h"
#include "nrf.h"
#include "nrf_soc.h"
#include "ic_flash_enclosed.h"
#include "global_conf.h"
#include <stddef.h>
//---------------------------------------------------------------------------------------------------
///UART internal FIFO
static uint8_t buff[UART_LOGGER_FIFO_BUFFER_SIZE];
//static FIFO_u8 *uart_fifo = &(FIFO_u8)FIFO_INIT(buff, UART_LOGGER_FIFO_BUFFER_SIZE);
static FIFO_u8 uart_fifo__ = FIFO_INIT(buff, UART_LOGGER_FIFO_BUFFER_SIZE);
static FIFO_u8 *uart_fifo = &uart_fifo__;
static bool useUart			= true;
static bool useFlash		= false;
static bool useUartDefault	= true;
static bool useFlashDefault	= false;
static bool echo_enable         = false;
void(*charater_callback)(char c) = NULL;

//---------------------------------------------------------------------------------------------------
///UART TX data transfer complete
volatile uint8_t UART_TX_rdy_flag = 1;
//---------------------------------------------------------------------------------------------------

#ifdef UART_CONTROLED_NEUROON_ON
	extern vint8_t uart_flag_mask_program;
#endif

void UARTLOG_Init(bool echo, void(*rx_callback)(char c)) {
  echo_enable = echo;
  charater_callback = rx_callback;
	UART_LOGGER_NRF_PERIPHERAL->PSELTXD = UART_LOGGER_TX_PIN_NUMBER;
        UART_LOGGER_NRF_PERIPHERAL->PSELRXD = UART_LOGGER_RX_PIN_NUMBER;
	UART_LOGGER_NRF_PERIPHERAL->CONFIG = (UART_CONFIG_HWFC_Disabled << UART_CONFIG_HWFC_Pos) | (UART_CONFIG_PARITY_Excluded << UART_CONFIG_PARITY_Pos);

	UART_LOGGER_NRF_PERIPHERAL->BAUDRATE = UART_LOGGER_BAUDRATE;

	UART_LOGGER_NRF_PERIPHERAL->INTENSET = (UART_INTENSET_RXDRDY_Enabled <<
            UART_INTENSET_RXDRDY_Pos) | (UART_INTENSET_TXDRDY_Enabled << UART_INTENSET_TXDRDY_Pos );

	UART_LOGGER_NRF_PERIPHERAL->TASKS_STARTRX = 1;
	UART_LOGGER_NRF_PERIPHERAL->TASKS_STARTTX = 1;

	UART_LOGGER_NRF_PERIPHERAL->POWER = UART_POWER_POWER_Enabled;
	UART_LOGGER_NRF_PERIPHERAL->ENABLE = UART_ENABLE_ENABLE_Enabled;

	sd_nvic_SetPriority(UART0_IRQn, APP_IRQ_PRIORITY_LOW); //APP_IRQ_PRIORITY_HIGH = 1
	sd_nvic_EnableIRQ(UART0_IRQn);

	///FIFO cleaning (synchronized method)
	fifou8_sclear(uart_fifo);
}

void UARTLOG_Deinit(){
	sd_nvic_DisableIRQ(UART0_IRQn);

	UART_LOGGER_NRF_PERIPHERAL->INTENSET = (UART_INTENSET_RXDRDY_Disabled << UART_INTENSET_RXDRDY_Pos) | (UART_INTENSET_TXDRDY_Disabled << UART_INTENSET_TXDRDY_Pos );

	UART_LOGGER_NRF_PERIPHERAL->TASKS_STARTTX = 0;
	UART_LOGGER_NRF_PERIPHERAL->TASKS_STARTRX = 0;

	UART_LOGGER_NRF_PERIPHERAL->POWER = UART_POWER_POWER_Disabled;
	UART_LOGGER_NRF_PERIPHERAL->ENABLE = UART_ENABLE_ENABLE_Disabled;

	//NRF_GPIO->PIN_CNF[UART_LOGGER_TX_PIN_NUMBER] = (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) | (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos);
}

void UARTLOG_setStreams(bool uart, bool flash) {
	useUart = uart;
	useFlash = flash;
}

void UARTLOG_setDefaultStreams(bool uart, bool flash) {
	useUartDefault = uart;
	useFlashDefault = flash;
}

void UARTLOG_useDefaultStreams() {
	useUart = useUartDefault;
	useFlash = useUartDefault;
}

synchronized bool UARTLOG_send(const uint8_t *buf, uint16_t len) {
	if (useUart) {
		if (fifou8_saddArray(uart_fifo, buf, len) == false)
		return false;
		if (UART_TX_rdy_flag) {
			UART_TX_rdy_flag = 0;
			UART_LOGGER_NRF_PERIPHERAL->TXD = fifou8_sget(uart_fifo);
		}
	}
	if (useFlash) {
		if (FLASH_ENCLOSED_add_raw_data_array_to_fifo((uint8_t *)buf, len) == false)
			return false;
	}

	return true;
}

synchronized bool UARTLOG_sendByte(uint8_t byte) {
	if (useUart) {
		if (fifou8_sadd(uart_fifo, byte) == false)
			return false;
		if (UART_TX_rdy_flag) {
			UART_TX_rdy_flag = 0;
			UART_LOGGER_NRF_PERIPHERAL->TXD = fifou8_sget(uart_fifo);
		}
	}
	if (useFlash) {
		if (FLASH_ENCLOSED_add_raw_data_sample_to_fifo(byte) == false)
			return false;
	}
	return true;
}

synchronized uint16_t UARTLOG_getFreeSpace() {
	return UART_LOGGER_FIFO_BUFFER_SIZE - fifou8_sgetCount(uart_fifo);
}

synchronized uint16_t UARTLOG_getNumberOfBytesToSend() {
	return fifou8_sgetCount(uart_fifo);
}
//---------------------------------------------------------------------------------------------------
//FOR IC TEST
void UART0_IRQHandler ()
{
	if (UART_LOGGER_NRF_PERIPHERAL->EVENTS_TXDRDY != 0){
		UART_LOGGER_NRF_PERIPHERAL->EVENTS_TXDRDY = 0;
		if (!fifou8_isEmpty(uart_fifo)) {
			UART_LOGGER_NRF_PERIPHERAL->TXD = fifou8_get(uart_fifo);
		} else {
			UART_TX_rdy_flag = 1;
		}
	}

        if (UART_LOGGER_NRF_PERIPHERAL->EVENTS_RXDRDY != 0)
        {
          UART_LOGGER_NRF_PERIPHERAL->EVENTS_RXDRDY = 0;
          uint32_t character = UART_LOGGER_NRF_PERIPHERAL->RXD;
          if(echo_enable) UARTLOG_sendByte(character);
          if(charater_callback != NULL) charater_callback(character);
        }
#ifdef UART_CONTROLED_NEUROON_ON
/*
 *        uint8_t cmd;
 *        const char button[]="Button";
 *        const char charger[]="Charger";
 *        if (UART_LOGGER_NRF_PERIPHERAL->EVENTS_RXDRDY != 0){
 *                UART_LOGGER_NRF_PERIPHERAL->EVENTS_RXDRDY = 0;
 *                cmd = NRF_UART0->RXD;
 *                if (cmd=='b' || cmd=='B'){
 *                        button_set_state();
 *                        UARTLOG_setStreams(true, false);
 *                        logf_sendStr(button);
 *                        UARTLOG_useDefaultStreams();
 *                }
 *                else if (cmd=='c' || cmd=='C'){
 *                        charger_set_flag();
 *                        UARTLOG_setStreams(true, false);
 *                        logf_sendStr(charger);
 *                        UARTLOG_useDefaultStreams();
 *                }
 *                else if (cmd=='s' || cmd=='S'){
 *                        uart_flag_mask_program = 's';
 *                }
 *                else if (cmd=='j' || cmd=='J'){
 *                        uart_flag_mask_program = 'j';
 *                }
 *                else if (cmd=='n' || cmd=='N'){
 *                        uart_flag_mask_program = 'n';
 *                }
 *                else if (cmd=='t' || cmd=='T'){
 *                        uart_flag_mask_program = 't';
 *                }
 *
 *        }
 */
#endif

//	app_managers_get_app_config()->test_cmd_recived =true;
//	app_managers_get_app_config()->test_cmd = (uint8_t)(NRF_UART0->RXD);
}
//---------------------------------------------------------------------------------------------------
