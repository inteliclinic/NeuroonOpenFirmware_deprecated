/**
 * @file    ic_cli.c
 * @author  Paweł Kaźmierzewski <p.kazmierzewski@inteliclinic.com>
 * @author  Wojciech Weclewski <w.weclewski@inteliclinic.com>
 * @date    October, 2016
 * @brief   Command-line user interface module.
 *
 *  Allow to communicate and control Neuroon using UART.
 */

#include "ic_cli.h"
#include "ic_uart_log.h"
#include "ic_cc.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <wordexp.h>

#define PROMPT "NeuroOn> "

typedef void(*f_cmdCode)(int cnt, int *args);

typedef struct{
  const char *name;
  void(*code)(int cnt, int *args);
  void *next;
}s_cmdList;

uint8_t string[100];
int string_len;

static char rx_buffer[51];
static char rx_buffer_history[3][51] = {0};
static int histoty_last_ptr = 0;
static char history_cnt = 0;
static size_t rx_ptr = 0;
s_cmdList *cmd_list = NULL;

static bool execute_command(const char *cmd_name, char **arg_list, size_t arg_num);
static s_cmdList * last_cmd_list(s_cmdList *list);
static s_cmdList * next_cmd_list(s_cmdList *list);
static uint8_t escape_sequence = 0;

void rx_callback(char c){
  if(c == 0x1b){
    if(escape_sequence == 0){
      escape_sequence++;
      return;
    }
  }
  if(escape_sequence){
    if(escape_sequence == 1){
      if(c == 0x5b){
        escape_sequence++;
      }
      else{
        escape_sequence = 0;
      }
      return;
    }
    else if(escape_sequence == 2){
      escape_sequence = 0;
      if(c==0x41){
        if(--histoty_last_ptr<0) histoty_last_ptr = 2;
        strcpy(rx_buffer, rx_buffer_history[histoty_last_ptr]);
        print_cli("\n\r"PROMPT"%s", rx_buffer);
      }
      else if(c==0x42){
        strcpy(rx_buffer, rx_buffer_history[histoty_last_ptr]);
        print_cli("\n\r"PROMPT"%s", rx_buffer);
        if(++histoty_last_ptr>2) histoty_last_ptr = 0;
      }
      else{
        return;
      }

      rx_ptr = strlen(rx_buffer);
      return;
    }
  }
  if (c== 0x08){
    if(rx_ptr){
      rx_ptr--;
      UARTLOG_sendByte(c);
      UARTLOG_sendByte(' ');
      UARTLOG_sendByte(c);
    }
    return;
  }
  UARTLOG_sendByte(c);
  if(c=='\r'){
    if(rx_ptr){
      rx_buffer[rx_ptr] = '\0';
      strcpy(rx_buffer_history[histoty_last_ptr++], rx_buffer);
      if(histoty_last_ptr == 3) histoty_last_ptr = 0;
      if(history_cnt<3) history_cnt++;
      rx_ptr = 0;
      cc_emit(CC_CLI_PARSE, 1);
    }
    print_cli("\n"PROMPT);
    return;
  }
  if(rx_ptr == 51){
    rx_ptr = 0;
    print_cli(PROMPT"\n\rCommand is too long! (50<)\n\r");
    print_cli("\n"PROMPT);
    return;
  }
  rx_buffer[rx_ptr++] = c;
  /*print_cli("0x%x\n\r",c);*/
}

void cli_test_func(int c, int *args){
  print_cli("Hello! Args are:\n\r");
  for(int i = 0; i<c; ++i){
    print_cli("%d\t", args[i]);
  }
}

void print_registered(int c, int *args){
  c = c;
  args = args;
  s_cmdList *tmp = cmd_list;
  print_cli("\n\r");
  for(;;){
    if(tmp==NULL) return;
    print_cli("%s\n\r",tmp->name);
    tmp = (s_cmdList *)tmp->next;
  }
}

void cli_init(){
  UARTLOG_Init(false, rx_callback);
  register_cmd("test", cli_test_func);
  register_cmd("ls", print_registered);
  print_cli("\n\r"PROMPT);
}


void cli_deinit(){
  UARTLOG_Deinit();
  s_cmdList *tmp_ptr = cmd_list;
  s_cmdList *next_ptr;
  if(tmp_ptr == NULL)
    return;
  for(;;){
    next_ptr = (s_cmdList *)tmp_ptr->next;
    free(tmp_ptr);
    tmp_ptr = next_ptr;
    if(tmp_ptr == NULL)
      break;
  }
  cmd_list = NULL;
}

void parse_command(){
  bool ret_val;
  if(rx_buffer[0]=='\0')
    return;
  wordexp_t p;
  wordexp(rx_buffer, &p, 0);
  ret_val = execute_command (p.we_wordv[0], &p.we_wordv[1], p.we_wordc - 1);
  wordfree(&p);
  if(ret_val == false){
    print_cli("Unknown command: %s", rx_buffer);
  }
  print_cli("\n\r"PROMPT);
}

static bool execute_command(const char *cmd_name, char **arg_list, size_t arg_num){
  if(cmd_list == NULL)
    return false;

  int *p_args = malloc(sizeof(int)*arg_num);
  for(int i=0; i<(int)arg_num; ++i){
    p_args[i] = atoi(arg_list[i]);
  }

  s_cmdList *curr = cmd_list;
  do{
    if(strcmp(cmd_name, curr->name) == 0){
      curr->code(arg_num, p_args);
      return true;
    }
  }while((curr = next_cmd_list(curr)));

  free(p_args);
  return false;
}

bool register_cmd (const char *name, void(*code)(int cnt, int *args)){
  s_cmdList *temp = malloc(sizeof(s_cmdList));

  if (temp == NULL) return false;
  temp->name = name;
  temp->code = code;
  temp->next = NULL;

  if(cmd_list==NULL)
    cmd_list = temp;
  else
    last_cmd_list(cmd_list)->next = temp;

  return true;
}

static s_cmdList * last_cmd_list(s_cmdList *list){
  if(list->next == NULL)
    return list;
  return last_cmd_list((s_cmdList *)list->next);
}

static s_cmdList * next_cmd_list(s_cmdList *list){
  return (s_cmdList *)list->next;
}
