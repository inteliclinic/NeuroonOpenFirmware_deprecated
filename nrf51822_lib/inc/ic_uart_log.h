//---------------------------------------------------------------------------------------------------
#ifndef NRF51822_LIB_INC_IC_UART_LOG_H_
#define NRF51822_LIB_INC_IC_UART_LOG_H_
//---------------------------------------------------------------------------------------------------
#include <synchronized.h>
#include <stdbool.h>
#include <ic_fifo.h>
//---------------------------------------------------------------------------------------------------
#define UART_LOGGER_FIFO_BUFFER_SIZE	512
#define UART_LOGGER_RX_PIN_NUMBER		19
#define UART_LOGGER_TX_PIN_NUMBER		20
#define UART_LOGGER_NRF_PERIPHERAL		NRF_UART0
#define UART_LOGGER_BAUDRATE			UART_BAUDRATE_BAUDRATE_Baud460800
//---------------------------------------------------------------------------------------------------
/**
 * @brief	Initialize UART for logging.
 *			This function:
 *				1) configure GPIO for valid UART port
 *				2) enable UART peripheral
 *				3) enable UART interrupt
 *				4) clear UART internal FIFO
 */
void UARTLOG_Init(bool echo, void(*charater_callback)(char c));
//---------------------------------------------------------------------------------------------------
/**
 * @brief	Deinitialize UART logger
 * 			This function:
 *				1) disable UART interrupt
 *				2) disable UART peripheral
 *				3) reconfigure UART GPIO to input (floating) port
 */
void UARTLOG_Deinit();
//---------------------------------------------------------------------------------------------------
/**
 * @brief	Set logger options (enable streams).
 * @param	uart	Set true to enable stream to UART.
 * @param	flash	Set true to enable stream to raw-flash-region.
 */
void UARTLOG_setStreams(bool uart, bool flash);
//---------------------------------------------------------------------------------------------------
/**
 * @brief	Set default streams logger option.
 * @param	uart	Set true to enable stream to UART.
 * @param	flash	Set true to enable stream to raw-flash-region.
 */
void UARTLOG_setDefaultStreams(bool uart, bool flash);
//---------------------------------------------------------------------------------------------------
/**
 * @brief	Use default setting to configure streams.
 */
void UARTLOG_useDefaultStreams();
//---------------------------------------------------------------------------------------------------
/**
 * @brief	Add byte to send via UARTLOG module
 * 			This is a synchronized method. Internally only synchronized FIFO methods are used.
 * @param	byte	data to send
 * @return	true if byte is copied to internal fifo, false otherwise
 * 			(e.g. not enough space in fifo)
 */
synchronized bool UARTLOG_sendByte(uint8_t byte);
//---------------------------------------------------------------------------------------------------
/**
 * @brief	Add data to send via UARTLOG module
 * 			This is a synchronized method. Internally only synchronized FIFO methods are used.
 * @param	buf pointer to send buffer array
 * @param	len length of send buffer array
 * @return	true if all data from buffer is copied to internal fifo, false otherwise
 * 			(e.g. not enough space in fifo)
 */
synchronized bool UARTLOG_send(const uint8_t *buf, uint16_t len);
//---------------------------------------------------------------------------------------------------
/**
 * @brief	Get free space in internal FIFO.
 * 			This is a synchronized method. Internally only synchronized FIFO methods are used.
 * @return	Number of free bytes in internal FIFO (max specified by UART_LOGGER_FIFO_BUFFER_SIZE).
 */
synchronized uint16_t UARTLOG_getFreeSpace();
//---------------------------------------------------------------------------------------------------
/**
 * @brief	Get number of bytes in internal FIFO (bytes to send).
 * 			This is a synchronized method. Internally only synchronized FIFO methods are used.
 * @return	Number of bytes in internal FIFO (bytes to send).
 */
synchronized uint16_t UARTLOG_getNumberOfBytesToSend();
//---------------------------------------------------------------------------------------------------
#endif /* NRF51822_LIB_INC_IC_UART_LOG_H_ */
//---------------------------------------------------------------------------------------------------
