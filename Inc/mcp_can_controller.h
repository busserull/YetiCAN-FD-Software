#ifndef MCP_CAN_CONTROLLER_H
#define MCP_CAN_CONTROLLER_H

#include <stdint.h>

uint8_t mcp_read_byte(uint16_t address);

void mcp_write_byte(uint16_t address, uint8_t data);

#endif
