#ifndef MCP_CAN_CONTROLLER_H
#define MCP_CAN_CONTROLLER_H

#include <stdint.h>

typedef struct {
    uint8_t a;
} MCP_TransmitObject;

/* void mcp_mode_nominal_can_fd(); */

/* void mcp_mode_configuration(); */

void mcp_init();

void mcp_send();

void mcp_write(uint16_t address, uint8_t * buffer, uint8_t size);

void mcp_read(uint16_t address, uint8_t * buffer, uint8_t size);

void mcp_set_reg(uint16_t, uint8_t, uint8_t);
uint8_t mcp_get_reg(uint16_t, uint8_t);

#endif
