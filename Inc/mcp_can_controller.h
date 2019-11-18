#ifndef MCP_CAN_CONTROLLER_H
#define MCP_CAN_CONTROLLER_H

#include <stdint.h>

typedef enum {
    MCP_DATA_LENGTH_00_BYTES = 0x00,
    MCP_DATA_LENGTH_01_BYTES = 0x01,
    MCP_DATA_LENGTH_02_BYTES = 0x02,
    MCP_DATA_LENGTH_03_BYTES = 0x03,
    MCP_DATA_LENGTH_04_BYTES = 0x04,
    MCP_DATA_LENGTH_05_BYTES = 0x05,
    MCP_DATA_LENGTH_06_BYTES = 0x06,
    MCP_DATA_LENGTH_07_BYTES = 0x07,
    MCP_DATA_LENGTH_08_BYTES = 0x08,
    MCP_DATA_LENGTH_12_BYTES = 0x09,
    MCP_DATA_LENGTH_16_BYTES = 0x0a,
    MCP_DATA_LENGTH_20_BYTES = 0x0b,
    MCP_DATA_LENGTH_24_BYTES = 0x0c,
    MCP_DATA_LENGTH_32_BYTES = 0x0d,
    MCP_DATA_LENGTH_48_BYTES = 0x0e,
    MCP_DATA_LENGTH_64_BYTES = 0x0f
} MCP_DataLength;

typedef enum {
    MCP_PAYLOAD_08_BYTES = 0x00,
    MCP_PAYLOAD_12_BYTES = 0x01,
    MCP_PAYLOAD_16_BYTES = 0x02,
    MCP_PAYLOAD_20_BYTES = 0x03,
    MCP_PAYLOAD_24_BYTES = 0x04,
    MCP_PAYLOAD_32_BYTES = 0x05,
    MCP_PAYLOAD_48_BYTES = 0x06,
    MCP_PAYLOAD_64_BYTES = 0x07
} MCP_PayloadSize;

typedef struct {
    MCP_PayloadSize payload_size;
    uint8_t message_depth;
    uint8_t use_timestamp;
} MCP_FifoConfig;

typedef struct {
    /* TX RX */
    uint8_t use_fd_format;
    uint8_t use_bit_rate_switch;
    uint8_t use_extended_id;
    uint8_t error_active;

    /* TX RX */
    uint32_t frame_id;
    /* TX */
    uint32_t sequence_number;
    /* RX */
    uint8_t filter_hit;
    uint8_t timestamp_valid;
    uint32_t timestamp;

    /* TX RX */
    MCP_DataLength data_length;
    uint8_t * p_data;
} MCP_Message;


void mcp_init();

uint8_t mcp_send(MCP_Message * p_transmit_object);

uint8_t mcp_transmit_event_get(uint32_t * p_sequence, uint32_t * p_timestamp);

uint8_t mcp_receive(MCP_Message * p_receive_object, uint8_t fifo_number);

#endif
