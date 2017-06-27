/**
 * @file    ic_rsp_emiter.c
 * @author  Paweł Kaźmierzewski <p.kazmierzewski@inteliclinic.com>
 * @author  Wojciech Weclewski <w.weclewski@inteliclinic.com>
 * @date    February, 2017
 * @brief   Response frame payload services.
 *
 *  Queues response payload in response frames FIFO. Creates signal for send response.
 */

#include "ic_rsp_emiter.h"

// FIFO for response frame
#define FIFO_TEMPLATE_DEF
#define FIFO_UNDERFLOW_RET		(u_BLECmdPayload){0}
#define FIFO_TEMPLATE_DEC
#define FIFO_TEMPLATE_FUN_PREFIX	fiforsp
#define FIFO_TEMPLATE_TYPE		u_BLECmdPayload
#define	FIFO_TEMPLATE_STR_NAME		FIFO_RSP
#include "ic_fifo_template.h"
#undef FIFO_UNDERFLOW_RET
#undef FIFO_TEMPLATE_DEC
#undef FIFO_TEMPLATE_FUN_PREFIX
#undef FIFO_TEMPLATE_TYPE
#undef	FIFO_TEMPLATE_STR_NAME
#undef FIFO_TEMPLATE_DEF

#define RSP_FIFO_SIZE 3
static u_BLECmdPayload rsp_frame_buf[RSP_FIFO_SIZE];
static FIFO_RSP* rsp_frame_fifo = &(FIFO_RSP)FIFO_INIT(rsp_frame_buf, RSP_FIFO_SIZE);

u_BLECmdPayload get_rsp_frame() {
  return fiforsp_get(rsp_frame_fifo);
}

void add_rsp_frame(u_BLECmdPayload response_payload){
  fiforsp_add(rsp_frame_fifo, response_payload);
}

