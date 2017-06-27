/**
 * @file    ic_cli.h
 * @author  Paweł Kaźmierzewski <p.kazmierzewski@inteliclinic.com>
 * @author  Wojciech Weclewski <w.weclewski@inteliclinic.com>
 * @date    October, 2016
 * @brief   Command-line user interface module.
 *
 *  Allows to communicate and control Neuroon using UART.
 */

#ifndef IC_CLI_H
#define IC_CLI_H

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "ic_uart_log.h"

extern uint8_t string[100];
extern int string_len;

#define print_cli(format, ...) do{\
  string_len = snprintf((char *)string, sizeof(string), format, ##__VA_ARGS__); \
  if (string_len>0) UARTLOG_send(string, string_len);\
}while(0);

void parse_command();
void cli_init();
void cli_deinit();
bool register_cmd (const char *name, void(*code)(int cnt, int *args));

#endif /* !IC_CLI_H */
