/**
 * @file    wordexp.c
 * @author  Paweł Kaźmierzewski <p.kazmierzewski@inteliclinic.com>
 * @author  Wojciech Weclewski <w.weclewski@inteliclinic.com>
 * @date    October, 2016
 * @brief   Implementation of wordexp.
 *
 */
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#ifndef HAVE_WORDEXP
#include<stdio.h>
#include<stdlib.h>
#include<sys/syslimits.h>
#include<string.h>
#include<wordexp.h>

/** Mimimal wordexp() implementation that only handles "~"
 *
 * @param[in]  s     String to process
 * @param[out] w     List of hits, caller frees using wordfree()
 * @param[in] flags  Not implemented in this hack
 *
 * @return 0 on success
 */
int wordexp(const char *s, wordexp_t *w, int flags)
{
  int string_len = 0;
  int items_cnt = 1;
  const char *ptr;
  for(ptr=s; *ptr; ptr++){
    if(*ptr == ' '){
      items_cnt++;
    }
    string_len++;
  }
  w->we_wordc = items_cnt;
  w->we_wordv = malloc((items_cnt)*sizeof(char*));
  if(!w->we_wordv){
    return -1;
  }
  ptr = s;
  for (int i=0; i<items_cnt; ++i){
    const char *next_item = strchr(ptr, ' ');
    if(!next_item) next_item = strchr(ptr, '\0');
    size_t arg_container_size = next_item - ptr;
    w->we_wordv[i] = malloc(arg_container_size + 1);// +1 because of NULL charactker
    if(!w->we_wordv[i]){
      for (int j = i-1; j>=0; --j){
        free(w->we_wordv[j]);
      }
      return -1;
    }
    strncpy(w->we_wordv[i], ptr, arg_container_size);
    w->we_wordv[i][arg_container_size] = '\0';
    ptr = next_item + 1;
  }
  return 0;
}

/** free() a wordexp() result
 *
 */
void
wordfree(wordexp_t *p)
{
  size_t n;
  for (n = 0; n < p->we_wordc; n++) {
    free(p->we_wordv[n]);
  }
  free(p->we_wordv);
}

/* ---- Emacs Variables ----
 * Local Variables:
 * c-basic-offset: 2
 * indent-tabs-mode: nil
 * End:
 */

#endif


