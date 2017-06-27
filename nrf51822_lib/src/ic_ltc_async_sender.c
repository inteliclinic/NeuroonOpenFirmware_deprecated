//---------------------------------------------------------------------------------------------------
#include <ic_ltc_async_sender.h>
#include <ic_fifo.h>
#include "ic_i2c.h"
#include "ic_ltc3220.h"
//---------------------------------------------------------------------------------------------------
/// internal FIFO for I2C (LTC)
static uint16_t buff[LTC_SENDER_FIFO_BUFFER_SIZE];
static FIFO_u16 ltcsend_fifo__ = FIFO_INIT(buff, LTC_SENDER_FIFO_BUFFER_SIZE);
static FIFO_u16 *ltcsend_fifo = &ltcsend_fifo__;
//---------------------------------------------------------------------------------------------------
///TWI TX data transfer complete
static volatile uint8_t TWI_TX_rdy_flag = 1;
///Did I send whole 16 bits?
static volatile uint8_t val_sended_flag = 0;
///Temporary data from FIFO
static volatile uint16_t data_for_ltc = 0;
//---------------------------------------------------------------------------------------------------
void LTCSEND_intFIFOClear() {
	///FIFO cleaning (synchronized method)
	fifou16_sclear(ltcsend_fifo);
}

void LTCSEND_int_HARD_FIFOClear() {
	///FIFO cleaning (synchronized method)
	fifou16_sclear(ltcsend_fifo);

	LTC_SENDER_NRF_PERIPHERAL->TASKS_STOP = 1;
	LTC_SENDER_NRF_PERIPHERAL->INTENSET = (TWI_INTENSET_TXDSENT_Disabled << TWI_INTENSET_TXDSENT_Pos);
	sd_nvic_DisableIRQ(SPI1_TWI1_IRQn);
	LTC_SENDER_NRF_PERIPHERAL->EVENTS_TXDSENT = 0;
	TWI_TX_rdy_flag = 1;

}

synchronized bool LTCSEND_send(const uint16_t *buf, uint8_t len) {
  if (len == 0) {
    return true;
  } else if (TWI_TX_rdy_flag) {
    if (len > 1)
      if (fifou16_addArray(ltcsend_fifo, buf + 1, len - 1) == false)
        return false;
    data_for_ltc = *buf;
    LTC_SENDER_NRF_PERIPHERAL->EVENTS_TXDSENT = 0;
    LTC_SENDER_NRF_PERIPHERAL->INTENSET = (TWI_INTENSET_TXDSENT_Enabled << TWI_INTENSET_TXDSENT_Pos);
    LTC_SENDER_NRF_PERIPHERAL->ADDRESS = LTC3220_TWI_ADDRESS;
    LTC_SENDER_NRF_PERIPHERAL->TXD = (uint8_t)(data_for_ltc >> 8);

    TWI_TX_rdy_flag = 0;

    sd_nvic_SetPriority(SPI1_TWI1_IRQn, APP_IRQ_PRIORITY_LOW); //APP_IRQ_PRIORITY_HIGH = 1
    sd_nvic_EnableIRQ(SPI1_TWI1_IRQn);

    LTC_SENDER_NRF_PERIPHERAL->TASKS_STARTTX = 1;
    val_sended_flag = 0;

    return true;
  } else if (fifou16_saddArray(ltcsend_fifo, buf, len) == false) {
    return false;
  }
}

bool LTCSEND_set_channel_level(uint8_t channel, uint8_t level) {
  uint16_t tmp = (channel << 8) | level;
  return LTCSEND_send(&tmp,1);
}

/**
 * @fn LTCSEND_set_chargepump ()
 * @brief force charge pump value
 * @param charge pump mode
 * @return true if operation correct
 */
bool LTCSEND_set_chargepump(uint8_t mode) {
  uint16_t tmp;
  if (mode == 1) //1x mode
    tmp = (LTC3220_COMMAND_REG << 8) | (LTC3220_FORCE1p5 | LTC3220_FORCE2);
  else if (mode == 2) //2x mode
    tmp = (LTC3220_COMMAND_REG << 8) | LTC3220_FORCE2;
  else if (mode == 3) //1.5x mode
    tmp = (LTC3220_COMMAND_REG << 8) | LTC3220_FORCE1p5;

  return LTCSEND_send(&tmp, 1);
}

synchronized bool LTCSEND_getState() {
	if ((LTC_SENDER_NRF_PERIPHERAL->INTENSET >> TWI_INTENSET_TXDSENT_Pos) & 1)
		return true;
	return false;
}
//---------------------------------------------------------------------------------------------------
/**
 * @fn SPI1_TWI1_IRQHandler ()
 * @brief I2C interrupt handler (LTC asynchronous sender).
 */
void SPI1_TWI1_IRQHandler () {
  if (LTC_SENDER_NRF_PERIPHERAL->EVENTS_TXDSENT == 1) {
    LTC_SENDER_NRF_PERIPHERAL->EVENTS_TXDSENT = 0;

    if (LTC_SENDER_NRF_PERIPHERAL->EVENTS_ERROR != 0) {
      TWI_Reset_Bus();
      LTCSEND_intFIFOClear();
      LTC_SENDER_NRF_PERIPHERAL->TASKS_STOP = 1;
      LTC_SENDER_NRF_PERIPHERAL->INTENSET = (TWI_INTENSET_TXDSENT_Disabled << TWI_INTENSET_TXDSENT_Pos);
      sd_nvic_DisableIRQ(SPI1_TWI1_IRQn);
      TWI_TX_rdy_flag = 1;
      ltc3220_refresh_all_channels();
      return;
    }

    if (!val_sended_flag) {
      val_sended_flag = 1;
      LTC_SENDER_NRF_PERIPHERAL->TXD = (uint8_t)(0xFF & data_for_ltc);
      //LTC_SENDER_NRF_PERIPHERAL->TASKS_STARTTX = 1;
      LTC_SENDER_NRF_PERIPHERAL->TASKS_STOP = 1;
    } else if (!fifou16_isEmpty(ltcsend_fifo)) {
      data_for_ltc = fifou16_get(ltcsend_fifo);
      val_sended_flag = 0;
      LTC_SENDER_NRF_PERIPHERAL->ADDRESS = LTC3220_TWI_ADDRESS;
      LTC_SENDER_NRF_PERIPHERAL->TXD = (uint8_t)(data_for_ltc >> 8);
      LTC_SENDER_NRF_PERIPHERAL->TASKS_STARTTX = 1;
    } else {
      LTC_SENDER_NRF_PERIPHERAL->TASKS_STOP = 1;
      LTC_SENDER_NRF_PERIPHERAL->INTENSET = (TWI_INTENSET_TXDSENT_Disabled << TWI_INTENSET_TXDSENT_Pos);
      sd_nvic_DisableIRQ(SPI1_TWI1_IRQn);
      TWI_TX_rdy_flag = 1;
    }
  }
}
//---------------------------------------------------------------------------------------------------
