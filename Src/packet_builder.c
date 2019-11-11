#include "../Inc/packet_builder.h"

#define STX 0x02
#define ETX 0x03

typedef enum {
    PACKET_BUILDER_ACCEPT_STX,
    PACKET_BUILDER_ACCEPT_TYPE,
    PACKET_BUILDER_ACCEPT_SIZE,
    PACKET_BUILDER_ACCEPT_DATA,
    PACKET_BUILDER_ACCEPT_CRC,
    PACKET_BUILDER_ACCEPT_ETX
} PacketBuilderState;

static PacketBuilderState m_state = PACKET_BUILDER_ACCEPT_STX;
static uint8_t m_expected_bytes = 0;
static uint8_t m_packet_buffer[PACKET_BUILDER_BUFFER_SIZE];
static uint16_t m_packet_buffer_index = 0;

static PacketBuilderPacket packet_builder_new(){
    PacketBuilderPacket packet = {0};

    packet.type = m_packet_buffer[0];

    packet.size = (m_packet_buffer[1] << 8) | m_packet_buffer[2];

    packet.p_data = m_packet_buffer + 3;

    packet.crc = 0xb00b;

    return packet;
}

static int packet_builder_accept_byte(uint8_t byte){
    switch(m_state){
        case PACKET_BUILDER_ACCEPT_STX:
            if(byte == STX){
                m_packet_buffer_index = 0;
                m_state = PACKET_BUILDER_ACCEPT_TYPE;
            }
            break;

        case PACKET_BUILDER_ACCEPT_TYPE:
            m_packet_buffer[m_packet_buffer_index++] = byte;
            m_expected_bytes = 2;
            m_state = PACKET_BUILDER_ACCEPT_SIZE;
            break;

        case PACKET_BUILDER_ACCEPT_SIZE:
            m_packet_buffer[m_packet_buffer_index++] = byte;
            m_expected_bytes--;
            if(m_expected_bytes == 0){
                uint16_t size = (m_packet_buffer[1] << 8) | m_packet_buffer[2];
                m_expected_bytes = size;
                m_state = PACKET_BUILDER_ACCEPT_DATA;
            }
            break;

        case PACKET_BUILDER_ACCEPT_DATA:
            m_packet_buffer[m_packet_buffer_index++] = byte;
            m_expected_bytes--;
            if(m_expected_bytes == 0){
                m_expected_bytes = 4;
                m_state = PACKET_BUILDER_ACCEPT_CRC;
            }
            break;

        case PACKET_BUILDER_ACCEPT_CRC:
            m_packet_buffer[m_packet_buffer_index++] = byte;
            m_expected_bytes--;
            if(m_expected_bytes == 0){
                m_state = PACKET_BUILDER_ACCEPT_ETX;
            }
            break;

        case PACKET_BUILDER_ACCEPT_ETX:
            m_state = PACKET_BUILDER_ACCEPT_STX;
            if(byte == ETX){
                return 1;
            }
            break;
    }
    return 0;
}

PacketBuilderPacket packet_builder_construct(uint8_t * buffer, uint16_t size){
    PacketBuilderPacket packet = {0};

    for(int i = 0; i < size; i++){
        int packet_complete = packet_builder_accept_byte(buffer[i]);
        if(packet_complete){
            return packet_builder_new();
        }
    }

    return packet;
}
