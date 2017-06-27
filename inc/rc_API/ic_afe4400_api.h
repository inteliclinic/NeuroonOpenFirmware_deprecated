/**
 * @file    ic_afe4400_api.h
 * @author  Paweł Kaźmierzewski <p.kazmierzewski@inteliclinic.com>
 * @author  Wojciech Węclewski <w.weclewski@inteliclinic.com>
 * @date    February, 2017
 * @brief   API for AFE4400 (command handling).
 *
 *  AFE4400 is a integrated analog front-end for pulse oximeter.
 */

#ifndef IC_AFE4400_API_H
#define IC_AFE4400_API_H

#include "ic_frame_handle.h"

void afe4400_cmd_handle(s_poxCmd *pox_cmd_data);

#endif /* !IC_AFE4400_API_H */
