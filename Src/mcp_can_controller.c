#include "mcp_can_controller.h"
#include "stm32f0xx_hal.h"

#define MCP_READ_COMMAND 0x03
#define COMMAND_WRITE 0x02
#define MCP_SPI_TIMEOUT 1000

#define REG(BASE, BYTE) ((BASE) + (BYTE))
#define REG_OSC 0x0e00
#define REG_C1CON 0x000
#define REG_C1TXQCON 0x050
#define REG_C1TXQSTA 0x054
#define REG_C1TXQUA 0x058
#define REG_C1FIFOCON1 0x05c
#define REG_C1FIFOSTA1 0x060
#define REG_C1FIFOCA1 0x064
#define REG_C1FLTCON0 0x1d0

extern SPI_HandleTypeDef g_spi1_handle;

static void mcp_slave_select(){
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
}

static void mcp_slave_deselect(){
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}

 uint8_t mcp_register_read(uint16_t address){
    uint8_t tx_buffer[3] = {0};
    uint8_t rx_buffer[3] = {0};

    tx_buffer[0] = (uint8_t)(MCP_READ_COMMAND << 4) | (uint8_t)(address >> 8);
    tx_buffer[1] = (uint8_t)(address & 0xff);

    mcp_slave_select();
    HAL_SPI_TransmitReceive(
        &g_spi1_handle, tx_buffer, rx_buffer, 3, MCP_SPI_TIMEOUT
    );
    mcp_slave_deselect();

    return rx_buffer[2];
}

static void mcp_write(uint16_t address, uint8_t * p_buffer, uint8_t size){
    uint8_t header[2] = {0};
    header[0] = (uint8_t)(COMMAND_WRITE << 4) | (uint8_t)(address >> 8);
    header[1] = (uint8_t)(address);

    mcp_slave_select();
    HAL_SPI_Transmit(&g_spi1_handle, header, 2, MCP_SPI_TIMEOUT);
    HAL_SPI_Transmit(&g_spi1_handle, p_buffer, size, MCP_SPI_TIMEOUT);
    mcp_slave_deselect();
}

static void mcp_register_write(uint16_t address, uint8_t data){
    uint8_t tx_buffer[6] = {0};

    tx_buffer[0] = (uint8_t)(COMMAND_WRITE << 4) | (uint8_t)(address >> 8);
    tx_buffer[1] = (uint8_t)(address & 0xff);
    tx_buffer[2] = data;

    mcp_slave_select();
    HAL_SPI_Transmit(&g_spi1_handle, tx_buffer, 3, MCP_SPI_TIMEOUT);
    mcp_slave_deselect();
}

static void mcp_register_wait_for_bit(uint16_t address, uint8_t mask){
    uint8_t bit_set = 0;
    while(!bit_set){
        bit_set = mcp_register_read(address) & mask;
    }
}

/* Configuration functions */

uint8_t mcp_conf_mode_configuration(){
    /* Abort all transmissions, and request configuration mode */
    mcp_register_write(REG(REG_C1CON, 3), 0x0c);
    mcp_register_wait_for_bit(REG(REG_C1CON, 2), 0x80);
    return 0;
}

uint8_t mcp_conf_mode_internal_loopback(){
    mcp_register_write(REG(REG_C1TXQCON, 3), 0xa3);

    mcp_register_write(REG(REG_C1FIFOCON1, 3), 0xa3);

    mcp_register_write(REG(REG_C1FLTCON0, 0), 0x81);

    mcp_register_write(REG(REG_C1CON, 2), 0x18);
    mcp_register_write(REG(REG_C1CON, 3), 0x02);
    mcp_register_wait_for_bit(REG(REG_C1CON, 2), 0x40);
    return 0;
}

uint8_t mcp_conf_mode_normal_can_fd(){
    mcp_register_write(REG(REG_C1CON, 3), 0x00);
    return 0;
}

uint8_t mcp_conf_clock_bypass_20MHz(){
    mcp_register_write(REG(REG_OSC, 0), 0x00);
    return 0;
}

uint8_t mcp_send(uint8_t * payload, uint8_t size){
    uint8_t tx_full = mcp_register_read(REG(REG_C1TXQSTA, 0)) & 0x01;
    if(tx_full){
        return 1;
    }

    uint32_t object_address = 0;
    object_address |= mcp_register_read(REG(REG_C1TXQUA, 3)) << 24;
    object_address |= mcp_register_read(REG(REG_C1TXQUA, 2)) << 16;
    object_address |= mcp_register_read(REG(REG_C1TXQUA, 1)) << 8;
    object_address |= mcp_register_read(REG(REG_C1TXQUA, 0));
    object_address += 0x400;

    uint8_t transmit_object[] = {
        0x00, 0x00, 0x00, 0x21,
        0x00, 0x00, 0x02, 0x8b,
        ' ', 't', 'e', 'L',
        'i', 't', 'e', 'Y',
        't', 'n', 'e', ' ',
        'y', ' ', 'r', 'e',
        '\r', '\n', 'u', 'o'
    };
    mcp_write(object_address, transmit_object, 28);

    mcp_register_write(REG(REG_C1TXQCON, 2), 0x01);
}

uint8_t mcp_conf_simple_read(){
    return mcp_register_read(REG(REG_C1FIFOSTA1, 3)) & 0x01;
}
