//---------------------------------------------------------------------------------------------------
#ifndef INC_IC_CRC8_H_
#define INC_IC_CRC8_H_
//---------------------------------------------------------------------------------------------------
#include <stdint.h>
//---------------------------------------------------------------------------------------------------
/**
 * @brief	Compute crc8 checksum of array.
 *
 * @param	data	Pointer to array.
 * @param	len		Length of array.
 *
 * @return	Value of crc8 checksum.
 */
uint8_t crc8_tab(uint8_t *data, int len);
//---------------------------------------------------------------------------------------------------
/**
 * @brief	Compute crc8 checksum of array (extended version).
 *
 * @param	crc		Previous value of crc8 checksum.
 * @param	data	Pointer to array.
 * @param	len		Length of array.
 *
 * @return	Value of new crc8 checksum.
 */
uint8_t crc8_tabex(uint8_t crc, uint8_t *data, int len);
//---------------------------------------------------------------------------------------------------
/**
 * @brief	Compute next crc8 checksum value.
 *
 * @param	crc		Previous value of crc8 checksum.
 * @param	dat		New data element added to checksum.
 *
 * @return	Value of new crc8 checksum.
 *
 * @note	Please note that initial crc8 checksum value should be 0x00.
 */
uint8_t crc8_next(uint8_t crc, uint8_t dat);
//---------------------------------------------------------------------------------------------------
#endif /* INC_IC_CRC8_H_ */
//---------------------------------------------------------------------------------------------------
