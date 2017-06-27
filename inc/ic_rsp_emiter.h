/**
 * @file    ic_rsp_emiter.h
 * @author  Paweł Kaźmierzewski <p.kazmierzewski@inteliclinic.com>
 * @author  Wojciech Weclewski <w.weclewski@inteliclinic.com>
 * @date    February, 2017
 * @brief   Response frame payload services.
 *
 *  Queues response payload in response frames FIFO. Creates signal for send response.
 */

#ifndef IC_RSP_EMITER_H
#define IC_RSP_EMITER_H

#include "ic_frame_handle.h"

u_BLECmdPayload get_rsp_frame();

void add_rsp_frame(u_BLECmdPayload response_payload);

#endif /* !IC_RSP_EMITER_H */
