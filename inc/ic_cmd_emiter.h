/**
 * @file    ic_cmd_emiter.h
 * @author  Paweł Kaźmierzewski <p.kazmierzewski@inteliclinic.com>
 * @author  Wojciech Weclewski <w.weclewski@inteliclinic.com>
 * @date    September, 2016
 * @brief   Command payload wrapper.
 *
 *  Bluetooth command frames parser.
 */

#ifndef IC_CMD_EMITER_H
#define IC_CMD_EMITER_H

#include <stdbool.h>
#include <stdint.h>

bool push_data_CMD0Frame(uint8_t *data, uint16_t len);

#endif /* !IC_CMD_EMITER_H */
