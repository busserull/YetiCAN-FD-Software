#include "packet.h"
#include "mcp_can_controller.h"
#include "crc.h"

#define STX                                 0x02
#define ETX                                 0x03

#define TYPE_CONFIG_COMMIT                  0x00
#define TYPE_CONFIG_BIT_RATE                0x01
#define TYPE_CONFIG_TRANSMIT_EVENT_FIFO     0x02
#define TYPE_CONFIG_TRANSMIT_QUEUE          0x03
#define TYPE_CONFIG_FIFO                    0x04
#define TYPE_CONFIG_FILTER                  0x05

#define SIZE_TYPE                              1
#define SIZE_DATA                             64
#define SIZE_CRC                               4

#define SIZE_CONFIG_BIT_RATE                   4
#define SIZE_CONFIG_TRANSMIT_EVENT_FIFO        2
#define SIZE_CONFIG_TRANSMIT_QUEUE             2
#define SIZE_CONFIG_FIFO                       4
#define SIZE_CONFIG_FILTER                    12

#define TYPE_SEND                           0x1a
#define TYPE_BRIDGE                         0x96
#define TYPE_STATUS                         0x68
#define TYPE_FAULT                          0xff

typedef enum {
    FAULT_CRC,
    FAULT_CONFIG,
    FAULT_MALFORMED
} Packet_Fault;

extern MCP_MasterConfig g_mcp_master_config;


static void     packet_transition_ok();
static void     packet_transition_fault(Packet_Fault fault);

static uint8_t  packet_buffer(uint8_t byte);

static void     packet_accept_stx(uint8_t byte);
static void     packet_accept_type(uint8_t byte);

static void     packet_accept_data(uint8_t byte);

static void     packet_accept_crc(uint8_t byte);
static void     packet_accept_etx(uint8_t byte);

/* static void     packet_accept_length(uint8_t byte); */

static void     packet_config_commit();
static void     packet_config_bit_rate();
static void     packet_config_tef();
static void     packet_config_txq();
static void     packet_config_fifo();
static void     packet_config_filter();

static uint8_t  packet_parse_bit_rate(uint8_t offset, uint8_t * p_seg1, uint8_t * p_seg2);
static uint8_t  packet_parse_flag(uint8_t offset, uint8_t * p_use_timestamp);
static uint8_t  packet_parse_message_depth(uint8_t offset, uint8_t * p_message_depth);
static uint8_t  packet_parse_payload_size(uint8_t offset, MCP_PayloadSize * p_payload);
static uint8_t  packet_parse_fifo_number(uint8_t offset, uint8_t * p_fifo);
static uint8_t  packet_parse_filter_number(uint8_t offset, uint8_t * p_filter);
static uint8_t  packet_parse_frame_type(uint8_t offset, MCP_FilterFrameType * p_frame_type);
static uint8_t  packet_parse_mask_or_object(uint8_t offset, uint32_t * p_mask_or_object);


static void (* m_packet_builder_state)(uint8_t) = packet_accept_stx;
static void (* m_after_data_action)() = NULL;

static uint8_t m_expected_data = 0;
static uint8_t m_packet_buffer_index = 0;
static uint8_t m_packet_buffer[SIZE_TYPE + SIZE_DATA + SIZE_CRC] = {0};




void packet_build(uint8_t byte){
    m_packet_builder_state(byte);
}


static void packet_transition_ok(){
    uint8_t buffer[] = "OK\n\r";
    CDC_Transmit_FS(buffer, 4);

    m_packet_buffer_index = 0;
    m_packet_builder_state = packet_accept_stx;
}

static void packet_transition_fault(Packet_Fault fault){
    (void)(fault);
    uint8_t buffer[] = "ERR\n\r";
    CDC_Transmit_FS(buffer, 5);

    m_packet_buffer_index = 0;
    m_packet_builder_state = packet_accept_stx;
}

static uint8_t packet_buffer(uint8_t byte){
    m_packet_buffer[m_packet_buffer_index] = byte;
    m_packet_buffer_index++;
    m_expected_data--;

    return m_expected_data;
}


static void packet_accept_stx(uint8_t byte){
    switch(byte){
        case STX:
            m_packet_builder_state = packet_accept_type;
            break;

        default:
            break;
    }
}

static void packet_accept_type(uint8_t byte){
    packet_buffer(byte);

    switch(byte){
        case TYPE_CONFIG_COMMIT:
            m_after_data_action = packet_config_commit;
            m_expected_data = SIZE_CRC;
            m_packet_builder_state = packet_accept_crc;
            break;

        case TYPE_CONFIG_BIT_RATE:
            m_after_data_action = packet_config_bit_rate;
            m_expected_data = SIZE_CONFIG_BIT_RATE;
            m_packet_builder_state = packet_accept_data;
            break;

        case TYPE_CONFIG_TRANSMIT_EVENT_FIFO:
            m_after_data_action = packet_config_tef;
            m_expected_data = SIZE_CONFIG_TRANSMIT_EVENT_FIFO;
            m_packet_builder_state = packet_accept_data;
            break;

        case TYPE_CONFIG_TRANSMIT_QUEUE:
            m_after_data_action = packet_config_txq;
            m_expected_data = SIZE_CONFIG_TRANSMIT_QUEUE;
            m_packet_builder_state = packet_accept_data;
            break;

        case TYPE_CONFIG_FIFO:
            m_after_data_action = packet_config_fifo;
            m_expected_data = SIZE_CONFIG_FIFO;
            m_packet_builder_state = packet_accept_data;
            break;

        case TYPE_CONFIG_FILTER:
            m_after_data_action = packet_config_filter;
            m_expected_data = SIZE_CONFIG_FILTER;
            m_packet_builder_state = packet_accept_data;
            break;

        default:
            packet_transition_fault(FAULT_MALFORMED);
            break;
    }
}

static void packet_accept_data(uint8_t byte){
    switch(packet_buffer(byte)){
        case 0:
            m_expected_data = SIZE_CRC;
            m_packet_builder_state = packet_accept_crc;
            break;

        default:
            break;
    }
}

static void packet_accept_crc(uint8_t byte){
    if(packet_buffer(byte)){
        return;
    }

    m_packet_builder_state = packet_accept_etx;
}

static void packet_accept_etx(uint8_t byte){
    if(byte != ETX){
        packet_transition_fault(FAULT_MALFORMED);
        return;
    }

    uint8_t crc_index = m_packet_buffer_index - SIZE_CRC;

    uint32_t received_crc = 0
        | (m_packet_buffer[crc_index + 0] << 24)
        | (m_packet_buffer[crc_index + 1] << 16)
        | (m_packet_buffer[crc_index + 2] << 8)
        | (m_packet_buffer[crc_index + 3] << 0)
        ;

    uint32_t computed_crc = crc_calculate(m_packet_buffer, crc_index);

    if(received_crc != computed_crc){
        packet_transition_fault(FAULT_CRC);
        return;
    }

    if(m_after_data_action != NULL){
        m_after_data_action();
    }
}

static void packet_config_commit(){
    mcp_init(&g_mcp_master_config);

    packet_transition_ok();
}

static void packet_config_bit_rate(){
    uint8_t seg1, seg2;

    packet_parse_bit_rate(1, &seg1, &seg2);
    g_mcp_master_config.nominal_bit_rate_seg1 = seg1;
    g_mcp_master_config.nominal_bit_rate_seg2 = seg2;

    packet_parse_bit_rate(3, &seg1, &seg2);
    g_mcp_master_config.data_bit_rate_seg1 = seg1;
    g_mcp_master_config.data_bit_rate_seg2 = seg2;

    packet_transition_ok();
}

static void packet_config_tef(){
    uint8_t message_depth, use_timestamp;

    if(packet_parse_message_depth(1, &message_depth)){
        packet_transition_fault(FAULT_CONFIG);
        return;
    }
    if(packet_parse_flag(2, &use_timestamp)){
        packet_transition_fault(FAULT_CONFIG);
        return;
    }

    g_mcp_master_config.transmit_event_config.message_depth = message_depth;
    g_mcp_master_config.transmit_event_config.use_timestamp = use_timestamp;

    packet_transition_ok();
}

static void packet_config_txq(){
    MCP_PayloadSize payload_size;
    uint8_t message_depth;

    if(packet_parse_payload_size(1, &payload_size)){
        packet_transition_fault(FAULT_CONFIG);
        return;
    }
    if(packet_parse_message_depth(2, &message_depth)){
        packet_transition_fault(FAULT_CONFIG);
        return;
    }

    g_mcp_master_config.transmit_queue_config.payload_size = payload_size;
    g_mcp_master_config.transmit_queue_config.message_depth = message_depth;

    packet_transition_ok();
}

static void packet_config_fifo(){
    MCP_PayloadSize payload_size;
    uint8_t fifo, message_depth, use_timestamp;

    if(packet_parse_fifo_number(1, &fifo)){
        packet_transition_fault(FAULT_CONFIG);
        return;
    }
    if(packet_parse_payload_size(2, &payload_size)){
        packet_transition_fault(FAULT_CONFIG);
        return;
    }
    if(packet_parse_message_depth(3, &message_depth)){
        packet_transition_fault(FAULT_CONFIG);
        return;
    }
    if(packet_parse_flag(4, &use_timestamp)){
        packet_transition_fault(FAULT_CONFIG);
        return;
    }

    MCP_FifoConfig * p_fifo;
    p_fifo = &(g_mcp_master_config.receive_fifo_config[fifo - 1]);

    p_fifo->payload_size = payload_size;
    p_fifo->message_depth = message_depth;
    p_fifo->use_timestamp = use_timestamp;

    packet_transition_ok();
}

static void packet_config_filter(){
    uint8_t filter, use_filter, fifo_destination;
    MCP_FilterFrameType frame_type;
    uint32_t filter_mask, filter_object;

    if(packet_parse_filter_number(1, &filter)){
        packet_transition_fault(FAULT_CONFIG);
        return;
    }
    if(packet_parse_flag(2, &use_filter)){
        packet_transition_fault(FAULT_CONFIG);
        return;
    }
    if(packet_parse_fifo_number(3, &fifo_destination)){
        packet_transition_fault(FAULT_CONFIG);
        return;
    }
    if(packet_parse_frame_type(4, &frame_type)){
        packet_transition_fault(FAULT_CONFIG);
        return;
    }
    if(packet_parse_mask_or_object(5, &filter_mask)){
        packet_transition_fault(FAULT_CONFIG);
        return;
    }
    if(packet_parse_mask_or_object(9, &filter_object)){
        packet_transition_fault(FAULT_CONFIG);
        return;
    }

    MCP_FilterConfig * p_filter;
    p_filter = &(g_mcp_master_config.filter_config[filter]);

    p_filter->use_filter = use_filter;
    p_filter->fifo_destination = fifo_destination;
    p_filter->frame_type = frame_type;
    p_filter->filter_mask = filter_mask;
    p_filter->filter_object = filter_object;

    packet_transition_ok();
}




/* static void packet_accept_length(uint8_t byte){ */
/*     uint8_t buffer[] = "LEN\n\r"; */
/*     CDC_Transmit_FS(buffer, 5); */

/*     m_packet_builder_state = accept_data; */
/* } */

/* static void packet_accept_data(uint8_t byte){ */
/*     uint8_t buffer[] = "DAT\n\r"; */
/*     CDC_Transmit_FS(buffer, 5); */

/*     m_packet_builder_state = accept_crc; */
/* } */

/* static void packet_accept_crc(uint8_t byte){ */
/*     uint8_t buffer[] = "CRC\n\r"; */
/*     CDC_Transmit_FS(buffer, 5); */

/*     m_packet_builder_state = accept_etx; */
/* } */

/* static void packet_accept_etx(uint8_t byte){ */
/*     uint8_t buffer[] = "ETX\n\r"; */
/*     CDC_Transmit_FS(buffer, 5); */

/*     m_packet_builder_state = accept_stx; */
/* } */

static uint8_t  packet_parse_bit_rate(uint8_t offset, uint8_t * p_seg1, uint8_t * p_seg2){
    *p_seg1 = m_packet_buffer[offset];
    *p_seg2 = m_packet_buffer[offset + 1];
    return 0;
}

static uint8_t  packet_parse_flag(uint8_t offset, uint8_t * p_use_timestamp){
    *p_use_timestamp = m_packet_buffer[offset] ? 1 : 0;
    return 0;
}

static uint8_t  packet_parse_message_depth(uint8_t offset, uint8_t * p_message_depth){
    uint8_t messages = m_packet_buffer[offset];
    *p_message_depth = messages & 0x1f;

    if(messages < 1 || messages > 32){
        return 1;
    }
    return 0;
}

static uint8_t  packet_parse_payload_size(uint8_t offset, MCP_PayloadSize * p_payload){
    uint8_t requested_payload = m_packet_buffer[offset];
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

static uint8_t  packet_parse_fifo_number(uint8_t offset, uint8_t * p_fifo){
    uint8_t fifo = m_packet_buffer[offset];
    *p_fifo = fifo & 0x1f;

    if(fifo < 1 || fifo > 31){
        return 1;
    }
    return 0;
}

static uint8_t  packet_parse_filter_number(uint8_t offset, uint8_t * p_filter){
    uint8_t filter = m_packet_buffer[offset];
    *p_filter = filter & 0x1f;

    if(filter > 31){
        return 1;
    }
    return 0;
}

static uint8_t  packet_parse_frame_type(uint8_t offset, MCP_FilterFrameType * p_frame_type){
    uint8_t frame_type = m_packet_buffer[offset];
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

static uint8_t  packet_parse_mask_or_object(uint8_t offset, uint32_t * p_mask_or_object){
    uint32_t value = 0
        | (m_packet_buffer[offset + 0] << 24)
        | (m_packet_buffer[offset + 1] << 16)
        | (m_packet_buffer[offset + 2] << 8)
        | (m_packet_buffer[offset + 3])
        ;

    *p_mask_or_object = value;
    return 0;
}
