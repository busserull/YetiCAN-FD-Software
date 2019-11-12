#include "mcp_can_controller.h"
#include "stm32f0xx_hal.h"

#define MCP_READ_COMMAND 0x03
#define COMMAND_WRITE 0x02
#define MCP_SPI_TIMEOUT 1000

#define REG(BASE, BYTE) ((BASE) + (BYTE))
#define REG_OSC 0xe00
#define REG_IOCON 0xe04
#define REG_C1CON 0x000
#define REG_C1TXQCON 0x050
#define REG_C1TXQSTA 0x054
#define REG_C1TXQUA 0x058
#define REG_C1FIFOCON1 0x05c
#define REG_C1FIFOSTA1 0x060
#define REG_C1FIFOCA1 0x064
#define REG_C1FLTCON0 0x1d0
#define REG_C1TEFCON 0x040
#define REG_C1TEFSTA 0x044
#define REG_C1TEFUA 0x048

extern SPI_HandleTypeDef g_spi1_handle;

static void mcp_slave_select(){
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
}

static void mcp_slave_deselect(){
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}

uint8_t mcp_get_reg(uint16_t address, uint8_t byte_number){
    uint8_t command[2] = {0};
    command[0] = (uint8_t)(MCP_READ_COMMAND << 4) | (uint8_t)(address >> 8);
    command[1] = (uint8_t)(address) + byte_number;

    uint8_t value = 0;

    mcp_slave_select();
    HAL_SPI_Transmit(&g_spi1_handle, command, 2, MCP_SPI_TIMEOUT);
    HAL_SPI_Receive(&g_spi1_handle, &value, 1, MCP_SPI_TIMEOUT);
    mcp_slave_deselect();

    return value;
}

void mcp_set_reg(uint16_t address, uint8_t byte_number, uint8_t value){
    uint8_t command[3] = {0};
    command[0] = (uint8_t)(COMMAND_WRITE << 4) | (uint8_t)(address >> 8);
    command[1] = (uint8_t)(address) + byte_number;
    command[2] = value;

    mcp_slave_select();
    HAL_SPI_Transmit(&g_spi1_handle, command, 3, MCP_SPI_TIMEOUT);
    mcp_slave_deselect();
}


/* Configuration functions */

/* uint8_t mcp_conf_mode_internal_loopback(){ */
/*     mcp_register_write(REG(REG_C1TXQCON, 3), 0xa3); */

/*     mcp_register_write(REG(REG_C1FIFOCON1, 3), 0xa3); */

/*     mcp_register_write(REG(REG_C1FLTCON0, 0), 0x81); */

/*     mcp_register_write(REG(REG_C1CON, 2), 0x18); */
/*     mcp_register_write(REG(REG_C1CON, 3), 0x02); */
/*     mcp_register_wait_for_bit(REG(REG_C1CON, 2), 0x40); */
/*     return 0; */
/* } */

/* uint8_t mcp_conf_mode_normal_can_fd(){ */
/*     mcp_register_write(REG(REG_C1CON, 3), 0x00); */
/*     return 0; */
/* } */


/* uint8_t mcp_send(){ */
    /* uint8_t tx_full = mcp_register_read(REG(REG_C1TXQSTA, 0)) & 0x01; */
    /* if(tx_full){ */
    /*     return 1; */
    /* } */

    /* uint32_t object_address = 0; */
    /* object_address |= mcp_register_read(REG(REG_C1TXQUA, 3)) << 24; */
    /* object_address |= mcp_register_read(REG(REG_C1TXQUA, 2)) << 16; */
    /* object_address |= mcp_register_read(REG(REG_C1TXQUA, 1)) << 8; */
    /* object_address |= mcp_register_read(REG(REG_C1TXQUA, 0)); */
    /* object_address += 0x400; */

    /* uint8_t transmit_object[] = { */
    /*     0x00, 0x00, 0x00, 0x21, */
    /*     0x00, 0x00, 0x02, 0x8b, */
    /*     ' ', 't', 'e', 'L', */
    /*     'i', 't', 'e', 'Y', */
    /*     't', 'n', 'e', ' ', */
    /*     'y', ' ', 'r', 'e', */
    /*     '\r', '\n', 'u', 'o' */
    /* }; */
    /* mcp_write(object_address, transmit_object, 28); */

    /* mcp_register_write(REG(REG_C1TXQCON, 2), 0x01); */

    /* return 0; */
/* } */


void mcp_init(){
    uint8_t buffer[4] = {0};

    /* Enter config mode */
    mcp_read(REG_C1CON, buffer, 4);
    buffer[3] &= 0xf8;
    buffer[3] |= 0x04;
    mcp_write(REG_C1CON, buffer, 4);
    /* do{ */
    /*     mcp_read(REG_C1CON, buffer, 4); */
    /* } while((buffer[2] & 0xe0) != 0x80); */

    return;

    /* Undivided clockout */
    mcp_read(REG_OSC, buffer, 4);
    buffer[0] &= 0x9f;
    mcp_write(REG_OSC, buffer, 4);

    /* Unasserted output GPIO */
    buffer[0] = 0x03;
    buffer[1] = 0x00;
    buffer[2] = 0x00;
    buffer[3] = 0x00;
    mcp_write(REG_IOCON, buffer, 4);

    /* Configure Transmit Event FIFO (TEF) */
    mcp_read(REG_C1CON, buffer, 4);
    buffer[2] |= 0x08;
    mcp_write(REG_C1CON, buffer, 4);
    //
    buffer[0] = 0x00;
    buffer[1] = 0x04; // Reset FIFO
    buffer[2] = 0x00;
    buffer[3] = 0x02; // 3 messages deep
    mcp_write(REG_C1TEFCON, buffer, 4);
    /* do{ */
    /*     mcp_read(REG_C1TEFCON, buffer, 4); */
    /* } while(buffer[1] & 0x04); */

    /* Configure Transmit Queue (TXQ) */
    mcp_read(REG_C1CON, buffer, 4);
    buffer[2] |= 0x10;
    mcp_write(REG_C1CON, buffer, 4);
    //
    buffer[0] = 0x80;
    buffer[1] = 0x04;
    buffer[2] = 0x60;
    buffer[3] = 0xe3; // 64 bit payload, 4 messages
    mcp_write(REG_C1TXQCON, buffer, 4);
    /* do{ */
    /*     mcp_write(REG_C1TXQCON, buffer, 4); */
    /* } while(buffer[1] & 0x04); */

    /* Enter external loopback */
    mcp_read(REG_C1CON, buffer, 4);
    buffer[3] &= 0xf8;
    buffer[3] |= 0x05;
    mcp_write(REG_C1CON, buffer, 4);
    /* do{ */
    /*     mcp_read(REG_C1CON, buffer, 4); */
    /* } while((buffer[2] & 0xe8) != 0xa0); */
}

void mcp_send(){
    uint8_t fifo_full = 0;
    mcp_read(REG_C1TXQSTA, &fifo_full, 1);
    fifo_full &= 0x01;

    uint8_t ram_address_bytes[4] = {0};
    mcp_read(REG_C1TXQUA, ram_address_bytes, 4);
    uint32_t ram_address = 0;
    for(int i = 3; i >= 0; i--){
        ram_address <<= 8;
        ram_address |= ram_address_bytes[i];
    }

    uint8_t message_object[72] = {
        0x00, 0x00, 0x00, 0x01,
        0x00, 0x00, 0x03, 0x8f,
        '\r', '\n', 'i', 'H'
    };

    mcp_write(ram_address, message_object, 72);

    uint8_t buffer[4] = {0};
    /* Increment UINC */
    mcp_read(REG_C1TXQCON, buffer, 4);
    buffer[1] |= 0x01;
    mcp_write(REG_C1TXQCON, buffer, 4);

    uint8_t tefneif = 0;
    mcp_read(REG_C1TEFSTA, &tefneif, 1);
    tefneif &= 0x01;

    /* Request sending */
    mcp_read(REG_C1TXQCON, buffer, 4);
    buffer[2] |= 0x02;
    mcp_write(REG_C1TXQCON, buffer, 4);

    /* do{ */
    /*     mcp_read(REG_C1TEFSTA, &tefneif, 1); */
    /*     tefneif &= 0x01; */
    /* } while(!tefneif); */
}


void mcp_read(uint16_t address, uint8_t * buffer, uint8_t size){
    uint8_t header[2] = {0};
    header[0] = (uint8_t)(MCP_READ_COMMAND << 4) | (uint8_t)(address >> 8);
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
