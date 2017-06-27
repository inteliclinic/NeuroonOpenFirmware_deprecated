/**
 * @file    ic_cc.c
 * @author  Paweł Kaźmierzewski <p.kazmierzewski@inteliclinic.com>
 * @author  Wojciech Weclewski <w.weclewski@inteliclinic.com>
 * @date    July, 2016
 * @brief   Command control module.
 *
 *  Command frames interpreter.
 *
 */

#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "ic_cc.h"

#define MAX_SIG_NUM   1

#define POPULATE_POINTER(x,y) do {switch (y){\
    case CC_CMD_MODULE:\
      x = &cmd_module;\
      break;\
    case CC_MEASURE_MODULE:\
      x = &measure_module;\
      break; \
  }} while(0)

typedef struct{
  struct {
    uint32_t data;
    int signal;
  } signal;
  void *next;
}s_ccSignalsList;

typedef struct{
  s_ccSignalsList *signals_ptr;
  s_ccSignalsList signals[MAX_SIG_NUM];
  uint8_t sig_counter;
}s_ccModuleData;

static uint8_t signal_ptr = 0;

static uint32_t signals_array[CC_NO_OF_SIGNALS] = {0};

static void add_signal(e_ccSignalType e, uint32_t fp){
  signals_array[e] = fp;
}

static e_ccSignalType pop_signal(u_ccDataAccessFunction *fp){
  for (int i = 0; i<CC_NO_OF_SIGNALS; i++){
    if (signal_ptr == CC_NO_OF_SIGNALS)
      signal_ptr = 0;
    if (signals_array[signal_ptr] != 0){
      *fp = (u_ccDataAccessFunction)signals_array[signal_ptr];
      signals_array[signal_ptr] = 0;
      return (e_ccSignalType)signal_ptr++;
    }
    signal_ptr++;
  }
  return 0;
}

bool cc_emit (e_ccSignalType e, uint32_t fp){
  add_signal(e, fp);
  return true;
}

e_ccSignalType cc_probe(u_ccDataAccessFunction *fp){
  return pop_signal(fp);
}
