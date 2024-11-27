#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include <mcp2515_emulator.h>

#define MCP251x_SPI_CMD_RESET_BASE 0xC
#define MCP251x_SPI_CMD_RESET_CMD  0x0

#define MCP251x_SPI_CMD_READ_WRITE_BIT_MODIFY_BASE 0x0
#define MCP251x_SPI_CMD_READ_CMD 0x3
#define MCP251x_SPI_CMD_WRITE_CMD 0x2
#define MCP251x_SPI_CMD_BIT_MODIFY_CMD 0x5

#define MCP251x_SPI_CMD_READ_RX_BUFFER_BASE 0x9
#define MCP251x_SPI_CMD_READ_RX_BUFFER0_CMD 0x0
#define MCP251x_SPI_CMD_READ_RX_BUFFER1_CMD 0x2
#define MCP251x_SPI_CMD_READ_RX_BUFFER2_CMD 0x4
#define MCP251x_SPI_CMD_READ_RX_BUFFER3_CMD 0x6

#define MCP251x_SPI_CMD_LOAD_TX_BUFFER_BASE 0x4
#define MCP251x_SPI_CMD_LOAD_TX_BUFFER0_CMD 0x0
#define MCP251x_SPI_CMD_LOAD_TX_BUFFER1_CMD 0x1
#define MCP251x_SPI_CMD_LOAD_TX_BUFFER2_CMD 0x2
#define MCP251x_SPI_CMD_LOAD_TX_BUFFER3_CMD 0x3
#define MCP251x_SPI_CMD_LOAD_TX_BUFFER4_CMD 0x4
#define MCP251x_SPI_CMD_LOAD_TX_BUFFER5_CMD 0x5

#define MCP251x_SPI_CMD_RTS_BASE 0x8
#define MCP251x_SPI_CMD_RTS_TXB0_CMD 0x1
#define MCP251x_SPI_CMD_RTS_TXB1_CMD 0x2
#define MCP251x_SPI_CMD_RTS_TXB2_CMD 0x4

#define MCP251x_SPI_CMD_READ_STATUS_BASE 0xA
#define MCP251x_SPI_CMD_READ_STATUS_CMD 0x0

#define MCP251x_SPI_CMD_RX_STATUS_BASE 0xB
#define MCP251x_SPI_CMD_RX_STATUS_CMD 0x0

static uint8_t canstat_read_access(mcp251x_td *mcp251x)
{
    /* just return the value for now */
    return mcp251x->regs[MCP251x_REG_CANSTAT_H][MCP251x_REG_CANSTAT_L];
}

/* handles SPI address */
static uint8_t handle_spi_access(mcp251x_td *mcp251x, uint8_t spi_data)
{
    (void) spi_data;
    uint8_t outdata = 0;

    switch(mcp251x->spi_access_type)
    {
        case MCP2515_SPI_ACCESS_READ:
            outdata = mcp251x->special_reg_access_read_cb ? mcp251x->special_reg_access_read_cb(mcp251x) : mcp251x->regs[mcp251x->reg_addr_h][mcp251x->reg_addr_l];
            break;
        case MCP2515_SPI_ACCESS_WRITE:
        case MCP2515_SPI_ACCESS_MODIFY:
            if(mcp251x->special_reg_access_write_cb)
                mcp251x->special_reg_access_write_cb(mcp251x, spi_data, mcp251x->modify_mask);
            else 
                mcp251x->regs[mcp251x->reg_addr_h][mcp251x->reg_addr_l] = spi_data;
            break;
        default:
            break;
    }

    mcp251x->special_reg_access_read_cb = NULL;
    mcp251x->special_reg_access_write_cb = NULL;
    mcp251x->spi_state = MCP2515_SPI_STATE_SPI_CMD;

    return outdata;
}

/* handles SPI address */
static uint8_t handle_spi_addr(mcp251x_td *mcp251x, uint8_t spi_data)
{
    uint8_t higher_order_addr = (spi_data >> 4);
    uint8_t lower_order_addr = (spi_data & 0xF);

    mcp251x->reg_addr_h = higher_order_addr;
    mcp251x->reg_addr_l = lower_order_addr;

    /* check if this is a control register */
    switch(lower_order_addr)
    {
        case 0xC:
            /* BFPCTRL */
            switch(higher_order_addr)
            {
                case 0x0:
                    break;
            }
            break;
        case 0xD:
            /* TXRTSCTRL */
            /* REC */
            /* EFLG */
            break;
        case 0xE:
            /* CANSTAT */
            mcp251x->special_reg_access_read_cb = canstat_read_access;
            break;
        default:
            break;
    }

    mcp251x->spi_state = MCP2515_SPI_STATE_SPI_ACCESS;

    (void) mcp251x;

    return 0;
}

uint8_t mcp251x_spi_cmd_reset(mcp251x_td *mcp251x)
{
    MCP251x_SET_OPMODE(mcp251x->regs[MCP251x_REG_CANSTAT_H][MCP251x_REG_CANSTAT_L], MCP251x_OPMODE_CONFIG);
    MCP251x_SET_OPMODE(mcp251x->regs[MCP251x_REG_CANCTRL_H][MCP251x_REG_CANCTRL_L], MCP251x_OPMODE_CONFIG);
    MCP251x_CANCTRL_SET_CLKPRE(mcp251x->regs[MCP251x_REG_CANCTRL_H][MCP251x_REG_CANCTRL_L], MCP251x_CANCTRL_CLKPRE_DIV8);
    MCP251x_SET_BIT(mcp251x->regs[MCP251x_REG_CANCTRL_H][MCP251x_REG_CANCTRL_L], MCP251x_CANCTRL_CLKEN);

    /* Set back to 'idle' after reset */
    mcp251x->spi_state = MCP2515_SPI_STATE_SPI_CMD;

    return 0;
}

/* handles base SPI commands */
static uint8_t handle_spi_cmd(mcp251x_td *mcp251x, uint8_t spi_data)
{
    uint8_t base_cmd = (spi_data >> 4);
    uint8_t sub_cmd = (spi_data & 0xF);
    uint8_t spi_out = 0;

    switch(base_cmd)
    {
        case MCP251x_SPI_CMD_RESET_BASE:
            /* handle reset */
            spi_out = mcp251x_spi_cmd_reset(mcp251x);
            break;
        case MCP251x_SPI_CMD_READ_WRITE_BIT_MODIFY_BASE:
            switch(sub_cmd)
            {
                case MCP251x_SPI_CMD_READ_CMD:
                    mcp251x->spi_access_type = MCP2515_SPI_ACCESS_READ;
                    break;
                case MCP251x_SPI_CMD_WRITE_CMD:
                    mcp251x->spi_access_type = MCP2515_SPI_ACCESS_WRITE;
                    mcp251x->modify_mask = 0xFF;
                    break;
                case MCP251x_SPI_CMD_BIT_MODIFY_CMD:
                    mcp251x->spi_access_type = MCP2515_SPI_ACCESS_MODIFY;
                    break;
            }
            mcp251x->spi_state = MCP2515_SPI_STATE_SPI_ADDRESS;
            break;
    }

    return spi_out;
}

uint8_t mcp251x_spi_blabla(mcp251x_td *mcp251x, uint8_t spi_data)
{
    uint8_t outdata = 0;

    switch(mcp251x->spi_state)
    {
        case MCP2515_SPI_STATE_SPI_CMD:
            outdata = handle_spi_cmd(mcp251x, spi_data);
            break;
        case MCP2515_SPI_STATE_SPI_ADDRESS:
            outdata = handle_spi_addr(mcp251x, spi_data);
            break;
        case MCP2515_SPI_STATE_SPI_ACCESS:
            outdata = handle_spi_access(mcp251x, spi_data);
            break;
    }

    return outdata;
}

void mcp251x_spi_emu_init(mcp251x_td *mcp251x)
{
    memset(mcp251x->regs, 0, sizeof(mcp251x->regs));

    /* initialize mem pointers */
}

