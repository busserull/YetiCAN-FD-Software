#ifndef PACKET_BUILDER_H
#define PACKET_BUILDER_H

#include <stdint.h>

#define PACKET_BUILDER_BUFFER_SIZE 256

typedef struct {
    uint8_t type;
    uint16_t size;
    uint8_t * p_data;
    uint32_t crc;
} PacketBuilderPacket;

PacketBuilderPacket packet_builder_construct(uint8_t * buffer, uint16_t size);

#endif
