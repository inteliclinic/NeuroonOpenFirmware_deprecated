/**
 * @file    ic_device_handle.h
 * @author  Paweł Kaźmierzewski <p.kazmierzewski@inteliclinic.com>
 * @author  Wojciech Weclewski <w.weclewski@inteliclinic.com>
 * @date    September, 2016
 * @brief   LTC devices high-level functions.
 *
 *  LTC devices are RGB LEDs, vibration motor and power LEDs.
 */

#ifndef IC_DEVICE_HANDLE_H
#define IC_DEVICE_HANDLE_H

#include <stdbool.h>
#include <stdint.h>

typedef uint8_t devDescriptor;
typedef uint8_t deviceOutput;

bool dev_register(uint8_t id, const char *name, devDescriptor *desc);
void dev_unregister_all(void);
bool dev_set_new_parameters(devDescriptor desc, uint8_t function, uint8_t amplitude, uint32_t duration, uint32_t period, uint32_t timer);
const char* dev_get_name(devDescriptor desc);
uint8_t dev_get_function(devDescriptor desc);
uint32_t dev_get_period(devDescriptor desc);
uint8_t dev_get_id(devDescriptor desc);
bool dev_is_active(devDescriptor desc);
bool dev_is_val_changed(devDescriptor desc);
deviceOutput dev_update(devDescriptor desc, uint32_t timer);

#endif /* !IC_DEVICE_HANDLE_H */
