#ifndef MCP_CAN_CONTROLLER_H
#define MCP_CAN_CONTROLLER_H

#include <stdint.h>
#include "stm32f0xx_hal.h"

void mcp_init(SPI_HandleTypeDef * p_spi_handle);

uint32_t mcp_read_register(uint16_t address);

uint8_t mcp_read_byte(uint16_t address);

void mcp_write_byte(uint16_t address, uint8_t data);

#endif