#include "mcp_can_controller.h"
#include "stm32f0xx_hal.h"

#define COMMAND_READ             0x03
#define COMMAND_WRITE            0x02
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

typedef enum {
    PAYLOAD_08_BYTES = 0x00,
    PAYLOAD_12_BYTES = 0x01,
    PAYLOAD_16_BYTES = 0x02,
    PAYLOAD_20_BYTES = 0x03,
    PAYLOAD_24_BYTES = 0x04,
    PAYLOAD_32_BYTES = 0x05,
    PAYLOAD_48_BYTES = 0x06,
    PAYLOAD_64_BYTES = 0x07
} PayloadSize;

static void mcp_slave_select();
static void mcp_slave_deselect();
static void mcp_mode_set(uint8_t mode, uint8_t kill_tx, uint8_t keep_sharing);
static void mcp_clock_bypass_init();
static uint8_t mcp_reg_get(uint16_t address, uint8_t byte_number);
static void mcp_reg_set(uint16_t address, uint8_t byte_number, uint8_t value);
static void mcp_tef_init(uint8_t message_depth, uint8_t use_timestamp);
static void mcp_txq_init(PayloadSize payload_size, uint8_t message_depth);



static void mcp_slave_select(){
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
}

static void mcp_slave_deselect(){
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
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

static void mcp_tef_init(uint8_t message_depth, uint8_t use_timestamp){
    uint8_t enable_tef = mcp_reg_get(C1CON, 2) | 0x08;
    mcp_reg_set(C1CON, 2, enable_tef);

    mcp_reg_set(C1TEFCON, 3, message_depth - 1);

    uint8_t timestamp = use_timestamp ? 0x20 : 0x00;
    mcp_reg_set(C1TEFCON, 0, timestamp);
    m_tef_timestamp_enabled = use_timestamp;

    uint8_t reset_fifo = 0x04;
    mcp_reg_set(C1TEFCON, 1, reset_fifo);

    uint8_t timeout = FIFO_INIT_TIMEOUT;
    uint8_t tef_busy;
    do{
        tef_busy = mcp_reg_get(C1TEFCON, 1) & 0x04;
        timeout--;
    } while(tef_busy && timeout);
}

static void mcp_txq_init(PayloadSize payload_size, uint8_t message_depth){
    uint8_t enable_txq = mcp_reg_get(C1CON, 2) | 0x10;
    mcp_reg_set(C1CON, 2, enable_txq);

    uint8_t size = ((uint8_t)(payload_size) << 5) | (message_depth - 1);
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

static uint32_t mcp_user_address_get(uint16_t ua_register);

typedef struct {
    PayloadSize payload_size;
    uint8_t message_depth;
    uint8_t use_timestamp;
} MCP_FifoConfig;

static void mcp_fifo_init_recv(uint8_t fifo, MCP_FifoConfig * p_config){
    uint16_t fifo_spacing = C1FIFOCON2 - C1FIFOCON1;
    uint16_t fifo_address = C1FIFOCON1 + (fifo - 1) * fifo_spacing;

    /* Set FIFO as Receive */
    uint8_t fifo_receive = p_config->use_timestamp ? 0x20 : 0x00;
    mcp_reg_set(fifo_address, 0, fifo_receive);

    /* Reserve space in RAM */
    uint8_t size = 0
        | ((uint8_t)(p_config->payload_size) << 5)
        | (p_config->message_depth)
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

uint8_t any_fifos(){
    for(int i = 0; i < 31; i++){
        uint8_t not_empty = mcp_reg_get(C1FIFOSTA1 + i, 0) & 0x01;
        if(not_empty){
            return i + 1;
        }
    }

    return 0;
}

#include <stdlib.h>
uint8_t * read_fifo(uint8_t fifo_number){
    /* MCP_TransmitObject message = {0}; */

    uint16_t ua_register = C1FIFOUA1 + (fifo_number - 1) * (C1FIFOUA2 - C1FIFOUA1);
    uint32_t fifo_address = mcp_user_address_get(ua_register);

    uint8_t * buffer = (uint8_t *)malloc(76 * sizeof(uint8_t));
    mcp_read(fifo_address, buffer, 76);

    mcp_reg_set(C1FIFOCON1, 1, 0x01);

    return buffer;
}

void mcp_init(){
    mcp_mode_set(MODE_CONFIGURATION, 1, 1);

    mcp_clock_bypass_init();

    /* Configure IOCON for GPIO */

    mcp_tef_init(5, 1);

    mcp_txq_init(PAYLOAD_64_BYTES, 5);

    MCP_FifoConfig fifo_config = {
        .payload_size = PAYLOAD_64_BYTES,
        .message_depth = 6,
        .use_timestamp = 1,
    };
    mcp_fifo_init_recv(1, &fifo_config);
    /* mcp_fifo_init_recv(1, &fifo_config); */

    mcp_reg_set(C1FLTCON0, 0, 0x00);
    mcp_reg_set(C1FLTCON0, 0, 0x01);
    mcp_reg_set(C1FLTCON0, 0, 0x81);

    mcp_mode_set(MODE_EXTERNAL_LOOPBACK, 1, 1);
}

typedef struct {
    uint8_t use_fd_format;
    uint8_t use_bit_rate_switch;
    uint8_t use_extended_id;
    uint32_t frame_id;
    uint32_t sequence_number;
    MCP_FrameDataLength data_length;
    uint8_t * p_data;
} TransmitObject;

static void mcp_make_object_header(TransmitObject object, uint8_t * p_header){
    for(int i = 0; i < 8; i++){
        p_header[i] = 0x00;
    }

    /* Transmit Object word 0 */
    p_header[3] = (uint8_t)(object.frame_id);
    p_header[2] = (uint8_t)(object.frame_id >> 8);
    p_header[1] = (uint8_t)(object.frame_id >> 16);
    p_header[0] = (uint8_t)((object.frame_id >> 24) & 0x3f);

    if(object.use_extended_id){
        p_header[0] &= 0x1f;
    }
    else{
        p_header[2] &= 0x07;
        p_header[1] = 0x00;
        p_header[0] = (object.frame_id & (1 << 11)) ? 0x40 : 0x00;
    }

    /* Transmit Object word 1 */
    uint8_t fdf_flag = object.use_fd_format ? 0x80 : 0x00;
    uint8_t brs_flag = object.use_bit_rate_switch ? 0x40 : 0x00;
    uint8_t rtr_flag = 0x00;
    uint8_t ide_flag = object.use_extended_id ? 0x01 : 0x00;
    uint8_t control_field = fdf_flag | brs_flag | rtr_flag | ide_flag;

    p_header[7] = control_field | (uint8_t)(object.data_length);
    p_header[6] = (uint8_t)(object.sequence_number << 1);
    p_header[5] = (uint8_t)(object.sequence_number << 9);
    p_header[4] = (uint8_t)(object.sequence_number << 17);
}

static uint32_t mcp_user_address_get(uint16_t ua_register){
    uint32_t address = 0x00000000;

    address |= mcp_reg_get(ua_register, 0);
    address |= mcp_reg_get(ua_register, 1) << 8;
    address |= mcp_reg_get(ua_register, 2) << 16;
    address |= mcp_reg_get(ua_register, 3) << 24;

    return address + 0x400;
}

uint8_t mcp_receive();

uint8_t mcp_send(){
    /* Check for space in TXQ */
    if(!(mcp_reg_get(C1TXQSTA, 0) & 0x01)){
        return 1;
    }

    /* Make transmit object */
    uint8_t data[8] = "Yeti\n\r\0\0";

    TransmitObject transmit_object = {0};
        /* .use_fd_format = 1, */
        /* .use_bit_rate_switch = 0, */
        /* .use_extended_id = 0, */
        /* .frame_id = 12, */
        /* .sequence_number = 24, */
        /* .data_length = DATA_LENGTH_08_BYTES, */
        /* .p_data = data */
    /* }; */
    transmit_object.use_fd_format = 1;
    transmit_object.use_bit_rate_switch = 0;
    transmit_object.use_extended_id = 0;
    transmit_object.frame_id = 12;
    transmit_object.sequence_number = 24;
    transmit_object.data_length = DATA_LENGTH_08_BYTES;
    transmit_object.p_data = data;

    uint8_t transmit_object_header[8] = {0};
    mcp_make_object_header(transmit_object, transmit_object_header);

    /* Insert transmit object */
    uint32_t insert_address = mcp_user_address_get(C1TXQUA);

    mcp_write(insert_address, transmit_object_header, 8);
    mcp_write(insert_address + 8, data, 8);

    /* Increment HEAD and request transmission */
    mcp_reg_set(C1TXQCON, 1, 0x03);

    return 0;
}

uint8_t mcp_check_transmit_event(MCP_TransmitEvent * p_te){
    /* Check for Transmit Event Objects */
    uint8_t tef_empty = !(mcp_reg_get(C1TEFSTA, 0) & 0x01);
    if(tef_empty){
        return 1;
    }

    /* Read Transmit Event Object */
    uint32_t tef_object_address = mcp_user_address_get(C1TEFUA);
    uint8_t tef_object[12] = {0};
    mcp_read(tef_object_address, tef_object, 12);

    /* Clear Transmit Event Object from controller */
    mcp_reg_set(C1TEFCON, 1, 0x01);

    /* Translate data */
    p_te->fd_enabled = (tef_object[7] & 0x80) ? 1 : 0;
    p_te->bit_rate_switch_enabled = (tef_object[7] & 0x40) ? 1 : 0;
    p_te->extended_id_enabled = (tef_object[7] & 0x10) ? 1 : 0;

    p_te->frame_id = 0
        | (tef_object[3])
        | ((tef_object[2] & 0x07) << 8)
        ;

    p_te->sequence_number = 0
        | (tef_object[6] >> 1)
        | (tef_object[5] << 7)
        | (tef_object[4] << 15)
        ;

    p_te->data_length = (MCP_FrameDataLength)(tef_object[7] & 0x0f);

    p_te->timestamp_enabled = m_tef_timestamp_enabled;
    p_te->timestamp = 0
        | (tef_object[11])
        | (tef_object[10] << 8)
        | (tef_object[ 9] << 16)
        | (tef_object[ 8] << 24)
        ;

    return 0;
}



void mcp_read(uint16_t address, uint8_t * buffer, uint8_t size){
    uint8_t header[2] = {0};
    header[0] = (uint8_t)(COMMAND_READ << 4) | (uint8_t)(address >> 8);
    header[1] = (uint8_t)(address);

    mcp_slave_select();
    HAL_SPI_Transmit(&g_spi1_handle, header, 2, MCP_SPI_TIMEOUT);
    HAL_SPI_Receive(&g_spi1_handle, buffer, size, MCP_SPI_TIMEOUT);
    mcp_slave_deselect();
}

void mcp_write(uint16_t address, uint8_t * buffer, uint8_t size){
    uint8_t header[2] = {0};
    header[0] = (uint8_t)(COMMAND_WRITE << 4) | (uint8_t)(address >> 8);
    header[1] = (uint8_t)(address);

    mcp_slave_select();
    HAL_SPI_Transmit(&g_spi1_handle, header, 2, MCP_SPI_TIMEOUT);
    HAL_SPI_Transmit(&g_spi1_handle, buffer, size, MCP_SPI_TIMEOUT);
    mcp_slave_deselect();
}
