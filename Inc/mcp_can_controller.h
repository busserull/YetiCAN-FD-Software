#ifndef MCP_CAN_CONTROLLER_H
#define MCP_CAN_CONTROLLER_H

#include <stdint.h>

typedef struct {
    uint8_t a;
} MCP_TransmitObject;

uint8_t mcp_conf_mode_configuration();

uint8_t mcp_conf_mode_internal_loopback();

uint8_t mcp_conf_mode_normal_can_fd();

uint8_t mcp_conf_clock_bypass_20MHz();

uint8_t mcp_register_read(uint16_t);

uint8_t mcp_send(uint8_t * payload, uint8_t size);



uint8_t mcp_conf_simple_read();

#endif
