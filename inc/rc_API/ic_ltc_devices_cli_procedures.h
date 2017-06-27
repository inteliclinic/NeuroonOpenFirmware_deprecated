/**
 * @file    ic_ltc_devices_cli_procedures.h
 * @author  Paweł Kaźmierzewski <p.kazmierzewski@inteliclinic.com>
 * @author  Wojciech Węclewski <w.weclewski@inteliclinic.com>
 * @date    November, 2016
 * @brief   Contains CLI procedures for LTC devices handlers and drivers testing.
 *
 *  For additional information about CLI, look into ic_cli module.
 */

#ifndef IC_LTC_DEVICES_CLI_PROCEDURES_H
#define IC_LTC_DEVICES_CLI_PROCEDURES_H

/**
 * @brief Set paremeters for RGB LEDs testing by CLI.
 *
 * It shuould work by writing in terminal "rgb args[0] args[1] args[2] args[3] args[4] args[5]
 * args[6]"
 * args:  0 - led side
 *        1 - function
 *        2 - colour
 *        3 - intensity
 *        4 - duration
 *        5 - period
 *        6 - frame ID (command counter)
 *
 * @param c Nothing interesting. For additional information about CLI, look into ic_cli module.
 * @param args Nothing interesting. For additional information about CLI, look into ic_cli module.
 */
void cli_rgb_led_set_func(int c, int *args);

/**
 * @brief Set parameters for vibrator testing by CLI.
 *
 * It shuould work by writing in terminal "vib args[0] args[1] args[2] args[3] args[4]"
 * args:  0 - function
 *        1 - intensity
 *        2 - period
 *        3 - duration
 *        4 - frame ID (command counter)
 *
 * @param c Nothing interesting. For additional information about CLI, look into ic_cli module.
 * @param args Nothing interesting. For additional information about CLI, look into ic_cli module.
 */
void cli_vibra_set_func(int c, int *args);

/**
 * @brief Set ramp function parameters for vibrator testing by CLI.
 *
 * It shuould work by writing in terminal "vib_ramp args[0] args[1]"
 * args:  0 - intensity
 *        1 - frame ID (command counter)
 *
 * @param c Nothing interesting. For additional information about CLI, look into ic_cli module.
 * @param args Nothing interesting. For additional information about CLI, look into ic_cli module.
 */
void cli_vibra_ramp_func(int c, int *args);

/**
 * @brief Set ramp function parameters for rgb testing by CLI.
 *
 * It shuould work by writing in terminal "rgb_ramp args[0] args[1] args[2] args[3]"
 * args:  0 - led side
 *        1 - colour
 *        2 - intensity
 *        3 - frame ID (command counter)
 *
 * @param c Nothing interesting. For additional information about CLI, look into ic_cli module.
 * @param args Nothing interesting. For additional information about CLI, look into ic_cli module.
 */
void cli_rgb_ramp_func(int c, int *args);

void cli_vibra_ON(int c, int *args);

/**
 * @brief Set parameters for power LEDs testing by CLI.
 *
 * It shuould work by writing in terminal "pwr args[0] args[1] args[2] args[3]"
 * args:  0 - function
 *        1 - period
 *        2 - duration
 *        3 - frame ID (command counter)
 *
 * @param c Nothing interesting. For additional information about CLI, look into ic_cli module.
 * @param args Nothing interesting. For additional information about CLI, look into ic_cli module.
 */
void cli_pwr_led_set_func(int c, int *args);

#endif /* !IC_LTC_DEVICES_CLI_PROCEDURES_H */
