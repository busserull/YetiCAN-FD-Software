#ifndef PARSE_H
#define PARSE_H
#include <stdint.h>
#include "mcp_can_controller.h"

uint8_t parse_bit_rate(
    uint8_t * buffer, uint8_t offset, uint8_t * p_seg1, uint8_t * p_seg2
);

uint8_t parse_flag(
    uint8_t * buffer, uint8_t offset, uint8_t * p_use_timestamp
);

uint8_t parse_message_depth(
    uint8_t * buffer, uint8_t offset, uint8_t * p_message_depth
);

uint8_t parse_payload_size(
    uint8_t * buffer, uint8_t offset, MCP_PayloadSize * p_payload
);

uint8_t parse_fifo_number(
    uint8_t * buffer, uint8_t offset, uint8_t * p_fifo
);

uint8_t parse_filter_number(
    uint8_t * buffer, uint8_t offset, uint8_t * p_filter
);

uint8_t parse_frame_type(
    uint8_t * buffer, uint8_t offset, MCP_FilterFrameType * p_frame_type
);

uint8_t parse_mask_or_object(
    uint8_t * buffer, uint8_t offset, uint32_t * p_mask_or_object
);

uint8_t parse_frame_id(
    uint8_t * buffer, uint8_t offset, uint32_t * p_frame_id
);

uint8_t parse_sequence(
    uint8_t * buffer, uint8_t offset, uint32_t * p_sequence
);

uint8_t parse_data_length(
    uint8_t * buffer, uint8_t offset, MCP_DataLength * p_data_length
);

#endif
