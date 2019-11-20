#ifndef PACKET_H
#define PACKET_H
#include <stdint.h>
#include "mcp_can_controller.h"

void packet_build(uint8_t byte);

void packet_message_to_host(MCP_Message * p_message);

#endif
