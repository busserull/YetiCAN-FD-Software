#include "parse.h"

uint8_t parse_bit_rate(
    uint8_t * buffer, uint8_t offset, uint8_t * p_seg1, uint8_t * p_seg2
){
    *p_seg1 = buffer[offset];
    *p_seg2 = buffer[offset + 1];
    return 0;
}

uint8_t parse_flag(
    uint8_t * buffer, uint8_t offset, uint8_t * p_use_timestamp
){
    *p_use_timestamp = buffer[offset] ? 1 : 0;
    return 0;
}

uint8_t parse_message_depth(
    uint8_t * buffer, uint8_t offset, uint8_t * p_message_depth
){
    uint8_t messages = buffer[offset];
    *p_message_depth = messages & 0x1f;

    if(messages < 1 || messages > 32){
        return 1;
    }
    return 0;
}

uint8_t parse_payload_size(
    uint8_t * buffer, uint8_t offset, MCP_PayloadSize * p_payload
){
    uint8_t requested_payload = buffer[offset];
    switch(requested_payload){
        case 8:
            *p_payload = MCP_PAYLOAD_08_BYTES;
            break;

        case 12:
            *p_payload = MCP_PAYLOAD_12_BYTES;
            break;

        case 16:
            *p_payload = MCP_PAYLOAD_16_BYTES;
            break;

        case 20:
            *p_payload = MCP_PAYLOAD_20_BYTES;
            break;

        case 24:
            *p_payload = MCP_PAYLOAD_24_BYTES;
            break;

        case 32:
            *p_payload = MCP_PAYLOAD_32_BYTES;
            break;

        case 48:
            *p_payload = MCP_PAYLOAD_48_BYTES;
            break;

        case 64:
            *p_payload = MCP_PAYLOAD_64_BYTES;
            break;

        default:
            return 1;
    }
    return 0;
}

uint8_t parse_fifo_number(
    uint8_t * buffer, uint8_t offset, uint8_t * p_fifo
){
    uint8_t fifo = buffer[offset];
    *p_fifo = fifo & 0x1f;

    if(fifo < 1 || fifo > 31){
        return 1;
    }
    return 0;
}

uint8_t parse_filter_number(
    uint8_t * buffer, uint8_t offset, uint8_t * p_filter
){
    uint8_t filter = buffer[offset];
    *p_filter = filter & 0x1f;

    if(filter > 31){
        return 1;
    }
    return 0;
}

uint8_t parse_frame_type(
    uint8_t * buffer, uint8_t offset, MCP_FilterFrameType * p_frame_type
){
    uint8_t frame_type = buffer[offset];
    switch(frame_type){
        case 0:
            *p_frame_type = MCP_FILTER_ACCEPT_ANY;
            break;

        case 1:
            *p_frame_type = MCP_FILTER_ACCEPT_EXTENDED_ONLY;
            break;

        case 2:
            *p_frame_type = MCP_FILTER_ACCEPT_STANDARD_ONLY;
            break;

        default:
            return 1;
    }
    return 0;
}

uint8_t parse_mask_or_object(
    uint8_t * buffer, uint8_t offset, uint32_t * p_mask_or_object
){
    uint32_t value = 0
        | (buffer[offset + 0] << 24)
        | (buffer[offset + 1] << 16)
        | (buffer[offset + 2] << 8)
        | (buffer[offset + 3])
        ;

    *p_mask_or_object = value;
    return 0;
}

uint8_t parse_frame_id(
    uint8_t * buffer, uint8_t offset, uint32_t * p_frame_id
){
    uint32_t frame_id = 0
        | (buffer[offset + 0] << 24)
        | (buffer[offset + 1] << 16)
        | (buffer[offset + 2] << 8)
        | (buffer[offset + 3])
        ;

    *p_frame_id = frame_id;

    uint8_t use_extended_id = buffer[offset - 2] ? 1 : 0;

    if(use_extended_id){
        return frame_id & (uint32_t)(~(0x1fffffff));
    }
    else{
        return frame_id & (uint32_t)(~(0x7ff));
    }
}

uint8_t parse_sequence(
    uint8_t * buffer, uint8_t offset, uint32_t * p_sequence
){
    uint32_t sequence = 0
        | (buffer[offset + 0] << 24)
        | (buffer[offset + 1] << 16)
        | (buffer[offset + 2] << 8)
        | (buffer[offset + 3])
        ;

    *p_sequence = sequence;

    return sequence & (uint32_t)(~(0x3fffff));
}

uint8_t parse_data_length(
    uint8_t * buffer, uint8_t offset, MCP_DataLength * p_data_length
){
    uint8_t requested_data_length = buffer[offset];
    switch(requested_data_length){
        case 0x00:
            *p_data_length = MCP_DATA_LENGTH_00_BYTES;
            break;

        case 0x01:
            *p_data_length = MCP_DATA_LENGTH_01_BYTES;
            break;

        case 0x02:
            *p_data_length = MCP_DATA_LENGTH_02_BYTES;
            break;

        case 0x03:
            *p_data_length = MCP_DATA_LENGTH_03_BYTES;
            break;

        case 0x04:
            *p_data_length = MCP_DATA_LENGTH_04_BYTES;
            break;

        case 0x05:
            *p_data_length = MCP_DATA_LENGTH_05_BYTES;
            break;

        case 0x06:
            *p_data_length = MCP_DATA_LENGTH_06_BYTES;
            break;

        case 0x07:
            *p_data_length = MCP_DATA_LENGTH_07_BYTES;
            break;

        case 0x08:
            *p_data_length = MCP_DATA_LENGTH_08_BYTES;
            break;

        case 0x09:
            *p_data_length = MCP_DATA_LENGTH_12_BYTES;
            break;

        case 0x0a:
            *p_data_length = MCP_DATA_LENGTH_16_BYTES;
            break;

        case 0x0b:
            *p_data_length = MCP_DATA_LENGTH_20_BYTES;
            break;

        case 0x0c:
            *p_data_length = MCP_DATA_LENGTH_24_BYTES;
            break;

        case 0x0d:
            *p_data_length = MCP_DATA_LENGTH_32_BYTES;
            break;

        case 0x0e:
            *p_data_length = MCP_DATA_LENGTH_48_BYTES;
            break;

        case 0x0f:
            *p_data_length = MCP_DATA_LENGTH_64_BYTES;
            break;

        default:
            return 1;
    }
    return 0;
}
