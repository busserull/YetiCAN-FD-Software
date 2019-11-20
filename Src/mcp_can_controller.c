#include "mcp_can_controller.h"
#include "stm32f0xx_hal.h"
#include <stdlib.h>

#define COMMAND_READ             0x03
#define COMMAND_WRITE            0x02
#define COMMAND_RESET            0x00
#define MCP_SPI_TIMEOUT           100

#define FIFO_INIT_TIMEOUT          10

#define SFR_OSC                0x0e00
#define SFR_IOCON              0x0e04
#define SFR_CRC                0x0e08
#define SFR_ECCCON             0x0e0c
#define SFR_ECCSTAT            0x0e10
#define SFR_DEVID              0x0e14

#include "mcp_can_controller_registers.h"

#define MODE_NORMAL_CAN_FD       0x00
#define MODE_SLEEP               0x01
#define MODE_INTERNAL_LOOPBACK   0x02
#define MODE_LISTEN_ONLY         0x03
#define MODE_CONFIGURATION       0x04
#define MODE_EXTERNAL_LOOPBACK   0x05
#define MODE_NORMAL_CAN_2_0      0x06
#define MODE_RESTRICTED          0x07

extern SPI_HandleTypeDef g_spi1_handle;

static uint8_t m_tef_timestamp_enabled = 0;
static uint8_t m_fifo_timestamp_enabled[MCP_NUMBER_OF_FIFOS] = {0};


static void     mcp_slave_select();
static void     mcp_slave_deselect();

static void     mcp_reset();
static void     mcp_mode_set(uint8_t mode, uint8_t kill_tx, uint8_t keep_sharing);
static void     mcp_clock_bypass_init();
static void     mcp_gpio_init();

static uint8_t  mcp_reg_get(uint16_t address, uint8_t byte_number);
static void     mcp_reg_set(uint16_t address, uint8_t byte_number, uint8_t value);

static uint32_t mcp_user_address_get(uint16_t ua_register);

static void     mcp_ram_read(uint16_t address, uint8_t * buffer, uint8_t size);
static void     mcp_ram_write(uint16_t address, uint8_t * buffer, uint8_t size);

static void     mcp_tef_init(MCP_FifoConfig * p_config);
static void     mcp_txq_init(MCP_FifoConfig * p_config);
static void     mcp_fifo_init(uint8_t fifo, MCP_FifoConfig * p_config);

static void     mcp_nominal_bit_time_init(uint8_t seg1, uint8_t seg2);
static void     mcp_data_bit_time_init(uint8_t seg1, uint8_t seg2);


static void mcp_slave_select(){
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
}

static void mcp_slave_deselect(){
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}

static void mcp_reset(){
    uint8_t command[2] = {0};
    command[0] = (uint8_t)(COMMAND_RESET << 4);
    command[1] = 0x00;

    mcp_slave_select();
    HAL_SPI_Transmit(&g_spi1_handle, command, 2, MCP_SPI_TIMEOUT);
    mcp_slave_deselect();
}

static void mcp_mode_set(uint8_t mode, uint8_t kill_tx, uint8_t keep_sharing){
    uint8_t transmit_bandwidth_sharing = 0x00;
    if(keep_sharing){
        transmit_bandwidth_sharing = mcp_reg_get(C1CON, 3) & 0xf0;
    }

    uint8_t request_mode_change = 0x00;
    request_mode_change = transmit_bandwidth_sharing;
    if(kill_tx){
        request_mode_change |= 0x08;
    }
    request_mode_change |= mode;

    mcp_reg_set(C1CON, 3, request_mode_change);

    uint8_t actual_mode = 0x00;
    do{
        actual_mode = (mcp_reg_get(C1CON, 2) & 0xe0) >> 5;
    } while(actual_mode != mode);
}

static void mcp_clock_bypass_init(){
    mcp_reg_set(SFR_OSC, 0, 0x00);
}

static void mcp_gpio_init(){
    /* Use GPIO pins as GPIO */
    mcp_reg_set(SFR_IOCON, 3, 0x03);

    /* Drive GPIO pins low */
    mcp_reg_set(SFR_IOCON, 1, 0x00);

    /* Set GPIO0 as output */
    mcp_reg_set(SFR_IOCON, 0, 0x02);
}

static uint8_t mcp_reg_get(uint16_t address, uint8_t byte_number){
    uint8_t command[2] = {0};
    command[0] = (uint8_t)(COMMAND_READ << 4) | (uint8_t)(address >> 8);
    command[1] = (uint8_t)(address) + byte_number;

    uint8_t value = 0;

    mcp_slave_select();
    HAL_SPI_Transmit(&g_spi1_handle, command, 2, MCP_SPI_TIMEOUT);
    HAL_SPI_Receive(&g_spi1_handle, &value, 1, MCP_SPI_TIMEOUT);
    mcp_slave_deselect();

    return value;
}

static void mcp_reg_set(uint16_t address, uint8_t byte_number, uint8_t value){
    uint8_t command[3] = {0};
    command[0] = (uint8_t)(COMMAND_WRITE << 4) | (uint8_t)(address >> 8);
    command[1] = (uint8_t)(address) + byte_number;
    command[2] = value;

    mcp_slave_select();
    HAL_SPI_Transmit(&g_spi1_handle, command, 3, MCP_SPI_TIMEOUT);
    mcp_slave_deselect();
}

static uint32_t mcp_user_address_get(uint16_t ua_register){
    uint32_t address = 0
        | (mcp_reg_get(ua_register, 0))
        | (mcp_reg_get(ua_register, 1) << 8)
        | (mcp_reg_get(ua_register, 2) << 16)
        | (mcp_reg_get(ua_register, 3) << 24)
        ;

    return address + 0x400;
}

static void mcp_ram_read(uint16_t address, uint8_t * buffer, uint8_t size){
    uint8_t header[2] = {0};
    header[0] = (uint8_t)(COMMAND_READ << 4) | (uint8_t)(address >> 8);
    header[1] = (uint8_t)(address);

    mcp_slave_select();
    HAL_SPI_Transmit(&g_spi1_handle, header, 2, MCP_SPI_TIMEOUT);
    HAL_SPI_Receive(&g_spi1_handle, buffer, size, MCP_SPI_TIMEOUT);
    mcp_slave_deselect();
}

static void mcp_ram_write(uint16_t address, uint8_t * buffer, uint8_t size){
    uint8_t header[2] = {0};
    header[0] = (uint8_t)(COMMAND_WRITE << 4) | (uint8_t)(address >> 8);
    header[1] = (uint8_t)(address);

    mcp_slave_select();
    HAL_SPI_Transmit(&g_spi1_handle, header, 2, MCP_SPI_TIMEOUT);
    HAL_SPI_Transmit(&g_spi1_handle, buffer, size, MCP_SPI_TIMEOUT);
    mcp_slave_deselect();
}

static void mcp_tef_init(MCP_FifoConfig * p_config){
    uint8_t enable_tef = mcp_reg_get(C1CON, 2) | 0x08;
    mcp_reg_set(C1CON, 2, enable_tef);

    mcp_reg_set(C1TEFCON, 3, p_config->message_depth - 1);

    uint8_t timestamp = p_config->use_timestamp ? 0x20 : 0x00;
    mcp_reg_set(C1TEFCON, 0, timestamp);
    m_tef_timestamp_enabled = p_config->use_timestamp;

    uint8_t reset_fifo = 0x04;
    mcp_reg_set(C1TEFCON, 1, reset_fifo);

    uint8_t timeout = FIFO_INIT_TIMEOUT;
    uint8_t tef_busy;
    do{
        tef_busy = mcp_reg_get(C1TEFCON, 1) & 0x04;
        timeout--;
    } while(tef_busy && timeout);
}

static void mcp_txq_init(MCP_FifoConfig * p_config){
    uint8_t enable_txq = mcp_reg_get(C1CON, 2) | 0x10;
    mcp_reg_set(C1CON, 2, enable_txq);

    uint8_t size = 0
        | ((uint8_t)(p_config->payload_size) << 5)
        | (p_config->message_depth - 1)
        ;

    mcp_reg_set(C1TXQCON, 3, size);

    uint8_t unlimited_retransmission_attempts = 0x60;
    mcp_reg_set(C1TXQCON, 2, unlimited_retransmission_attempts);

    uint8_t reset_txq = 0x04;
    mcp_reg_set(C1TXQCON, 1, reset_txq);

    uint8_t timeout = FIFO_INIT_TIMEOUT;
    uint8_t txq_busy;
    do{
        txq_busy = mcp_reg_get(C1TXQCON, 1) & 0x04;
        timeout--;
    } while(txq_busy && timeout);
}

static void mcp_fifo_init(uint8_t fifo, MCP_FifoConfig * p_config){
    uint32_t fifo_spacing = C1FIFOCON2 - C1FIFOCON1;
    uint32_t fifo_address = C1FIFOCON1 + (fifo - 1) * fifo_spacing;

    /* Track timestamp use */
    m_fifo_timestamp_enabled[fifo - 1] = p_config->use_timestamp;

    /* Set FIFO as Receive */
    uint8_t fifo_receive = p_config->use_timestamp ? 0x20 : 0x00;
    mcp_reg_set(fifo_address, 0, fifo_receive);

    /* Reserve space in RAM */
    uint8_t size = 0
        | ((uint8_t)(p_config->payload_size) << 5)
        | (p_config->message_depth - 1)
        ;

    mcp_reg_set(fifo_address, 3, size);

    /* Reset FIFO */
    mcp_reg_set(fifo_address, 1, 0x04);

    uint8_t timeout = FIFO_INIT_TIMEOUT;
    uint8_t fifo_busy;
    do{
        fifo_busy = mcp_reg_get(fifo_address, 1) & 0x04;
        timeout--;
    } while(fifo_busy && timeout);
}

void mcp_nominal_bit_time_init(uint8_t seg1, uint8_t seg2){
    /* Baud rate prescaler = 1 */
    mcp_reg_set(C1NBTCFG, 3, 0x00);

    mcp_reg_set(C1NBTCFG, 2, seg1);
    mcp_reg_set(C1NBTCFG, 1, seg2);

    uint8_t sync_jump_width = (seg1 > seg2) ? seg2 : seg1;
    mcp_reg_set(C1NBTCFG, 0, sync_jump_width);
}

void mcp_data_bit_time_init(uint8_t seg1, uint8_t seg2){
    /* Baud rate prescaler = 1 */
    mcp_reg_set(C1DBTCFG, 3, 0x00);

    mcp_reg_set(C1DBTCFG, 2, seg1);
    mcp_reg_set(C1DBTCFG, 1, seg2);

    uint8_t sync_jump_width = (seg1 > seg2) ? seg2 : seg1;
    mcp_reg_set(C1DBTCFG, 0, sync_jump_width);
}

void mcp_init(MCP_MasterConfig * p_config){
    mcp_reset();

    mcp_mode_set(MODE_CONFIGURATION, 1, 1);

    mcp_clock_bypass_init();

    mcp_gpio_init();

    mcp_nominal_bit_time_init(
        p_config->nominal_bit_rate_seg1,
        p_config->nominal_bit_rate_seg2
    );

    mcp_data_bit_time_init(
        p_config->data_bit_rate_seg1,
        p_config->data_bit_rate_seg2
    );

    mcp_tef_init(&(p_config->transmit_event_config));

    mcp_txq_init(&(p_config->transmit_queue_config));

    for(int i = 0; i < MCP_NUMBER_OF_FIFOS; i++){
        mcp_fifo_init(i + 1, &(p_config->receive_fifo_config[i]));
    }

    for(int i = 0; i < MCP_NUMBER_OF_FIFOS + 1; i++){
        /* Disable filter to configure */
        uint32_t filter_spacing = C1FLTCON1 - C1FLTCON0;
        uint32_t filter_register = C1FLTCON0 + (i / 4) * filter_spacing;

        uint8_t disable_filter = 0x00;
        mcp_reg_set(filter_register, i % 4, disable_filter);

        /* Set filter mask */
        uint32_t mask_spacing = C1MASK1 - C1MASK0;
        uint32_t mask_register = C1MASK0 + i * mask_spacing;
        uint32_t filter_mask = p_config->filter_config[i].filter_mask;

        MCP_FilterFrameType frame_type = p_config->filter_config[i].frame_type;
        uint8_t frame_mask = 0
            | ((frame_type != MCP_FILTER_ACCEPT_ANY) ? 0x40 : 0x00)
            | ((filter_mask >> 24) & 0x1f)
            ;

        mcp_reg_set(mask_register, 0, (uint8_t)(filter_mask));
        mcp_reg_set(mask_register, 1, (uint8_t)(filter_mask >> 8));
        mcp_reg_set(mask_register, 2, (uint8_t)(filter_mask >> 16));
        mcp_reg_set(mask_register, 3, frame_mask);

        /* Set filter object */
        uint32_t object_spacing = C1FLTOBJ1 - C1FLTOBJ0;
        uint32_t object_register = C1FLTOBJ0 + i * object_spacing;
        uint32_t filter_object = p_config->filter_config[i].filter_object;

        uint8_t frame_object = 0
            | ((frame_type == MCP_FILTER_ACCEPT_EXTENDED_ONLY) ? 0x40 : 0x00)
            | ((filter_object >> 24) & 0x1f)
            ;

        mcp_reg_set(object_register, 0, (uint8_t)(filter_object));
        mcp_reg_set(object_register, 1, (uint8_t)(filter_object >> 8));
        mcp_reg_set(object_register, 2, (uint8_t)(filter_object >> 16));
        mcp_reg_set(object_register, 3, frame_object);

        /* Set FIFO destination and enable filter */
        uint8_t filter_setting = 0
            | (p_config->filter_config[i].use_filter ? 0x80 : 0x00)
            | (p_config->filter_config[i].fifo_destination)
            ;

        mcp_reg_set(filter_register, i % 4, filter_setting);
    }

    mcp_mode_set(MODE_EXTERNAL_LOOPBACK, 1, 1);
}

void mcp_gpio_latch(){
    mcp_reg_set(SFR_IOCON, 1, 0x01);
}

void mcp_gpio_unlatch(){
    mcp_reg_set(SFR_IOCON, 1, 0x00);
}

uint8_t mcp_send(MCP_Message * p_msg){
    /* Check for space in TXQ */
    if(!(mcp_reg_get(C1TXQSTA, 0) & 0x01)){
        return 1;
    }

    /* Create Transmit Object header */
    uint8_t transmit_object_header[8] = {0};

    /* Packet ID */
    transmit_object_header[0] = (uint8_t)(p_msg->frame_id);
    transmit_object_header[1] = (uint8_t)(p_msg->frame_id >> 8);
    transmit_object_header[2] = (uint8_t)(p_msg->frame_id >> 16);
    transmit_object_header[3] = (uint8_t)(p_msg->frame_id >> 24) & 0x1f;

    /* Data length and flags */
    uint8_t packet_flags = 0
        | (p_msg->use_fd_format ? 0x80 : 0x00)
        | (p_msg->use_bit_rate_switch ? 0x40 : 0x00)
        | (p_msg->use_extended_id ? 0x10 : 0x00)
        ;

    uint8_t esi = p_msg->error_active ? 0x00 : 0x01;

    transmit_object_header[4] = packet_flags | (p_msg->data_length);
    transmit_object_header[5] = (uint8_t)(p_msg->sequence_number << 1) | esi;
    transmit_object_header[6] = (uint8_t)(p_msg->sequence_number >> 7);
    transmit_object_header[7] = (uint8_t)(p_msg->sequence_number >> 15);

    /* Insert transmit object */
    uint32_t head_address = mcp_user_address_get(C1TXQUA);
    uint32_t data_address = head_address + 8;

    mcp_ram_write(head_address, transmit_object_header, 8);
    uint8_t decoded_length = mcp_decode_data_length(p_msg->data_length);
    mcp_ram_write(data_address, p_msg->p_data, decoded_length);

    /* Increment HEAD and request transmission */
    mcp_reg_set(C1TXQCON, 1, 0x03);

    return 0;
}

uint8_t mcp_transmit_event_get(uint32_t * p_sequence, uint32_t * p_timestamp){
    /* Check for Transmit Event Objects */
    if(!(mcp_reg_get(C1TEFSTA, 0) & 0x01)){
        return 1;
    }

    /* Read Transmit Event Object, skip frame ID */
    uint32_t event_address = mcp_user_address_get(C1TEFUA) + 8;
    uint8_t event[8] = {0};
    mcp_ram_read(event_address, event, 8);

    /* Clear Transmit Event from RAM */
    mcp_reg_set(C1TEFCON, 1, 0x01);

    /* Construct sequence number and timestamp */
    *p_sequence = 0
        | (event[1] >> 1)
        | (event[2] << 7)
        | (event[3] << 15)
        ;

    *p_timestamp = 0
        | (event[4])
        | (event[5] << 8)
        | (event[6] << 16)
        | (event[7] << 24)
        ;

    return 0;
}

uint8_t mcp_receive(MCP_Message * p_msg, uint8_t fifo_number){
    /* Check for messages */
    uint32_t spacing = C1FIFOSTA2 - C1FIFOSTA1;
    uint32_t status_register = C1FIFOSTA1 + (fifo_number - 1) * spacing;

    if(!(mcp_reg_get(status_register, 0) & 0x01)){
        return 1;
    }

    /* Get Receive Object address */
    spacing = C1FIFOUA2 - C1FIFOUA1;
    uint32_t ua_register = C1FIFOUA1 + (fifo_number - 1) * spacing;
    uint32_t head_address = mcp_user_address_get(ua_register);

    /* Construct Receive Object header */
    uint8_t object_header[12] = {0};
    mcp_ram_read(head_address, object_header, 12);

    p_msg->frame_id = 0
        | (object_header[0])
        | (object_header[1] << 8)
        | (object_header[2] << 16)
        | ((object_header[3] << 24) & 0x1f)
        ;

    p_msg->use_fd_format = (object_header[4] & 0x80) ? 1 : 0;
    p_msg->use_bit_rate_switch = (object_header[4] & 0x40) ? 1 : 0;
    p_msg->use_extended_id = (object_header[4] & 0x10) ? 1 : 0;
    p_msg->data_length = object_header[4] & 0x0f;

    p_msg->error_active = object_header[5] & 0x01;
    p_msg->filter_hit = object_header[5] >> 3;

    p_msg->timestamp = 0
        | (object_header[8])
        | (object_header[9] << 8)
        | (object_header[10] << 16)
        | (object_header[11] << 24)
        ;

    /* Copy Object data */
    uint32_t data_address;
    if(m_fifo_timestamp_enabled[fifo_number - 1]){
        data_address = head_address + 12;
        p_msg->timestamp_valid = 1;
    }
    else{
        data_address = head_address + 8;
        p_msg->timestamp_valid = 0;
    }

    uint8_t data_length = object_header[4] & 0x0f;
    uint8_t decoded_length = mcp_decode_data_length(data_length);
    uint8_t * buffer = (uint8_t *)malloc(decoded_length * sizeof(uint8_t));
    mcp_ram_read(data_address, buffer, decoded_length);

    p_msg->p_data = buffer;

    /* Clear packet from RAM */
    spacing = C1FIFOCON2 - C1FIFOCON1;
    uint32_t control_reg = C1FIFOCON1 + (fifo_number - 1) * spacing;
    mcp_reg_set(control_reg, 1, 0x01);

    return 0;
}

uint8_t mcp_decode_data_length(MCP_DataLength data_length){
    switch(data_length){
        case MCP_DATA_LENGTH_00_BYTES:
            return 0;

        case MCP_DATA_LENGTH_01_BYTES:
            return 1;

        case MCP_DATA_LENGTH_02_BYTES:
            return 2;

        case MCP_DATA_LENGTH_03_BYTES:
            return 3;

        case MCP_DATA_LENGTH_04_BYTES:
            return 4;

        case MCP_DATA_LENGTH_05_BYTES:
            return 5;

        case MCP_DATA_LENGTH_06_BYTES:
            return 6;

        case MCP_DATA_LENGTH_07_BYTES:
            return 7;

        case MCP_DATA_LENGTH_08_BYTES:
            return 8;

        case MCP_DATA_LENGTH_12_BYTES:
            return 12;

        case MCP_DATA_LENGTH_16_BYTES:
            return 16;

        case MCP_DATA_LENGTH_20_BYTES:
            return 20;

        case MCP_DATA_LENGTH_24_BYTES:
            return 24;

        case MCP_DATA_LENGTH_32_BYTES:
            return 32;

        case MCP_DATA_LENGTH_48_BYTES:
            return 48;

        case MCP_DATA_LENGTH_64_BYTES:
            return 64;

        default:
            return -1;
    }
}
