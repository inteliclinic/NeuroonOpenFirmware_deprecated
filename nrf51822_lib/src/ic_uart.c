/**
 * @file
 * @brief UART communication methods
 */

#include "ic_uart.h"
#ifdef USE_IC_UART
/**
 * @fn uart_init()
 * @brief UART bus initialization
 */
void uart_init ()
{
	NRF_UART0->PSELTXD = UART_TXD; //setting gpio
	NRF_UART0->PSELRXD = UART_RXD; //setting gpio
#ifdef UART_HWF_CONTROL_ENABLE
	NRF_UART0->PSELCTS = UART_CTS;
	NRF_UART0->PSELRTS = UART_RTS;
	NRF_UART0->CONFIG = (UART_CONFIG_HWFC_Enabled << UART_CONFIG_HWFC_Pos) | (UART_CONFIG_PARITY_Excluded << UART_CONFIG_PARITY_Pos);
#else
	NRF_UART0->CONFIG = (UART_CONFIG_HWFC_Disabled << UART_CONFIG_HWFC_Pos) | (UART_CONFIG_PARITY_Excluded << UART_CONFIG_PARITY_Pos);
#endif

	//NRF_UART0->BAUDRATE = UART_BAUDRATE;
	//na potrzeby komunikacji stm-maska
	NRF_UART0->BAUDRATE = UART_BAUDRATE_BAUDRATE_Baud115200;

	//NRF_UART0->INTENSET = (UART_INTENSET_RXDRDY_Enabled << UART_INTENSET_RXDRDY_Pos) | (UART_INTENSET_TXDRDY_Disabled << UART_INTENSET_TXDRDY_Pos );
	NRF_UART0->TASKS_STARTTX = 1;
	NRF_UART0->TASKS_STARTRX = 1;

	NRF_UART0->POWER = UART_POWER_POWER_Enabled;
	NRF_UART0->ENABLE = UART_ENABLE_ENABLE_Enabled;

}

/**
 * @fn uart_send_u8()
 * @brief send 1B data
 * @param data value
 */
void uart_send_u8(uint8_t data)
{

	NRF_UART0->EVENTS_TXDRDY = 0;
	NRF_UART0->TXD = data; //when value assigned transmission starts
	while(NRF_UART0->EVENTS_TXDRDY == 0); //w8 until byte is successfully transmitted
	NRF_UART0->EVENTS_TXDRDY = 0;
}

/**
 * @fn uart_send_tab()
 * @brief send data frame
 * @param length of frame
 * @param pointer to data frame
 */
void uart_send_tab(uint32_t len, uint8_t* data)
{
  for(uint32_t i=0; i<len;i++)
    uart_send_u8(*data++);
}

/**
 * @fn uart_send_rtab()
 * @brief send reversed data frame
 * @param length of frame
 * @param pointer to data frame
 */
void uart_send_rtab(uint8_t len, uint8_t* data)
{
	uint8_t i;

	for(i=len; i!=0; i--)
	{
		uart_send_u8(data[i-1]);
	}
}
#endif
