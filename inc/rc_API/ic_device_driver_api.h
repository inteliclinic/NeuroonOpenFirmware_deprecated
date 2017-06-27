/**
 * @file    ic_device_driver_api.h
 * @author  Paweł Kaźmierzewski <p.kazmierzewski@inteliclinic.com>
 * @author  Wojciech Węclewski <w.weclewski@inteliclinic.com>
 * @date    October, 2016
 * @brief   API for LTC channels and power LED drivers.
 */

#ifndef IC_DEVICE_DRIVER_API_H
#define IC_DEVICE_DRIVER_API_H

#include <stdint.h>
#include "ic_frame_handle.h"
#include "ic_ltc_common.h"
#include "ic_device_handle_api.h"

/** @defgroup DEVICE_DRIVER_API conrol module for LTC devices
 *
 * @{
 */

/**
 * @brief Sets a value (brightness intensity) of selected device (LTC channel).
 *
 * @param device structure (an element of @ref u_devVal) with byte length descriptor number and
 *               byte length value to set. Value should be unsigned and limited to 63 (LTC
 *               limitation).
 */
void drv_dev_set_val(u_devVal device);

/**
 * @brief Triggers POWER LED blink with selected period.
 *
 * @param device uint16_t (an element of @ref u_devVal) variable with period value.
 */
void drv_power_led_set_val(u_devVal device);

/**
 * @brief Periodically updates (emits silgnals to update LTC channel value) all devices drivers.
 *
 *        Period is set by DEV_DRIVER_UPDATE_DELAY in ic_device_driver_api.c.
 *
 * @param timer A timer set by user.
 */
void drv_all_dev_update(uint32_t timer);

#endif //IC_DEVICE_DRIVER_API_H
