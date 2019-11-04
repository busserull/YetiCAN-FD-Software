#include "mcp_can_controller.h"
#include "stm32f0xx_hal.h"

#define MCP_READ_COMMAND 0x03
#define MCP_WRITE_COMMAND 0x02
#define MCP_SPI_TIMEOUT 1000

extern SPI_HandleTypeDef g_spi1_handle;

static void mcp_slave_select(){
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
}

static void mcp_slave_deselect(){
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}

/* uint32_t mcp_read_register(uint16_t address){ */
/*     uint8_t tx_buffer[6] = {0}; */
/*     uint8_t rx_buffer[6] = {0}; */

/*     tx_buffer[0] = (uint8_t)(MCP_READ_COMMAND << 4) | (uint8_t)(address >> 8); */
/*     tx_buffer[1] = (uint8_t)(address & 0xff); */

/*     mcp_slave_select(); */

/*     HAL_SPI_TransmitReceive( */
/*         mp_spi_handle, tx_buffer, rx_buffer, 6, MCP_SPI_TIMEOUT */
/*     ); */

/*     mcp_slave_deselect(); */

/*     uint32_t mcp_register = 0; */
/*     mcp_register |= rx_buffer[2]; */
/*     mcp_register |= rx_buffer[3] << 8; */
/*     mcp_register |= rx_buffer[4] << 16; */
/*     mcp_register |= rx_buffer[5] << 24; */

/*     return mcp_register; */
/* } */

uint8_t mcp_read_byte(uint16_t address){
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

void mcp_write_byte(uint16_t address, uint8_t data){
    uint8_t tx_buffer[3] = {0};

    tx_buffer[0] = (uint8_t)(MCP_WRITE_COMMAND << 4) | (uint8_t)(address >> 8);
    tx_buffer[1] = (uint8_t)(address & 0xff);
    tx_buffer[2] = data;

    mcp_slave_select();

    HAL_SPI_Transmit(
        &g_spi1_handle, tx_buffer, 3, MCP_SPI_TIMEOUT
    );

    mcp_slave_deselect();
}
