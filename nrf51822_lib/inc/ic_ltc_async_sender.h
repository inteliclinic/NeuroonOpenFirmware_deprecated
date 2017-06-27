//---------------------------------------------------------------------------------------------------
#ifndef IC_LTC_ASYNC_SENDER_H_
#define IC_LTC_ASYNC_SENDER_H_
//---------------------------------------------------------------------------------------------------
#include <nrf.h>
#include <synchronized.h>
#include <stdbool.h>
//---------------------------------------------------------------------------------------------------
#define LTC_SENDER_FIFO_BUFFER_SIZE	10
#define LTC_SENDER_NRF_PERIPHERAL 	TWI_I2C
//---------------------------------------------------------------------------------------------------
/**
 * @brief	Clear LTCSEND internal FIFO
 */
void LTCSEND_intFIFOClear();

void LTCSEND_int_HARD_FIFOClear();
//---------------------------------------------------------------------------------------------------
/**
 * @brief	Add data to send to LTC via TWI I2C module
 * 			This is a synchronized method. Internally only synchronized FIFO methods are used.
 * @param	buf pointer to send buffer array
 * @param	len length of send buffer array
 * @return	true if all data from buffer is copied to internal fifo, false otherwise
 * 			(e.g. not enough space in fifo)
 */
synchronized bool LTCSEND_send(const uint16_t *buf, uint8_t len);

/**
 * @brief Add 2 byte data to send to LTC via TWI I2C module
 * @param channel ltc channel
 * @param level intensity of this channel
 * @return  true if data is copied to internal fifo, false otherwise
 *      (e.g. not enough space in fifo)
 */
bool LTCSEND_set_channel_level(uint8_t channel, uint8_t level);

/**
 * @fn LTCSEND_set_chargepump ()
 * @brief force charge pump value
 * @param charge pump mode
 * @return true if operation correct
 */
bool LTCSEND_set_chargepump(uint8_t mode);

//---------------------------------------------------------------------------------------------------
/**
 * @brief	Get data about LTC_SENDER's state of work.
 * 			This is a synchronized method. Internally only synchronized FIFO methods are used.
 * @return	true if TWI I2C interrupt is enable, false otherwise
 */
synchronized bool LTCSEND_getState();
//---------------------------------------------------------------------------------------------------
#endif /* IC_LTC_ASYNC_SENDER_H_ */
//---------------------------------------------------------------------------------------------------
