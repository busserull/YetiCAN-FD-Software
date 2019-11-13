#ifndef MCP_CAN_CONTROLLER_H
#define MCP_CAN_CONTROLLER_H

#include <stdint.h>

typedef enum {
    DATA_LENGTH_00_BYTES = 0x00,
    DATA_LENGTH_01_BYTES = 0x01,
    DATA_LENGTH_02_BYTES = 0x02,
    DATA_LENGTH_03_BYTES = 0x03,
    DATA_LENGTH_04_BYTES = 0x04,
    DATA_LENGTH_05_BYTES = 0x05,
    DATA_LENGTH_06_BYTES = 0x06,
    DATA_LENGTH_07_BYTES = 0x07,
    DATA_LENGTH_08_BYTES = 0x08,
    DATA_LENGTH_12_BYTES = 0x09,
    DATA_LENGTH_16_BYTES = 0x0a,
    DATA_LENGTH_20_BYTES = 0x0b,
    DATA_LENGTH_24_BYTES = 0x0c,
    DATA_LENGTH_32_BYTES = 0x0d,
    DATA_LENGTH_48_BYTES = 0x0e,
    DATA_LENGTH_64_BYTES = 0x0f
} MCP_FrameDataLength;

typedef struct {
    uint8_t use_fd_format;
    uint8_t use_bit_rate_switch;
    uint8_t use_extended_id;
    uint32_t frame_id;
    uint32_t sequence_number;
    MCP_FrameDataLength data_length;
    uint8_t * p_data;
} MCP_TransmitObject;

typedef struct {
    uint8_t fd_enabled;
    uint8_t bit_rate_switch_enabled;
    uint8_t extended_id_enabled;
    uint32_t frame_id;
    uint32_t sequence_number;

    MCP_FrameDataLength data_length;
    uint8_t timestamp_enabled;
    uint32_t timestamp;
} MCP_TransmitEvent;

uint8_t mcp_check_transmit_event(MCP_TransmitEvent * p_transmit_event);

void mcp_init();

uint8_t mcp_send();

void mcp_write(uint16_t address, uint8_t * buffer, uint8_t size);

void mcp_read(uint16_t address, uint8_t * buffer, uint8_t size);

uint8_t any_fifos();

#endif
