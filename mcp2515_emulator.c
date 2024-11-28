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

static inline uint8_t canstat_read(mcp251x_td *mcp251x)
{
    /* just return the value for now */
    return mcp251x->regs[MCP251x_REG_CANSTAT_H][MCP251x_REG_CANSTAT_L];
}

static inline uint8_t canstat_write(mcp251x_td *mcp251x, uint8_t data, uint8_t mask)
{
    (void) mask;

    /* just return the value for now */
    mcp251x->regs[MCP251x_REG_CANSTAT_H][MCP251x_REG_CANSTAT_L] = data;
    return 0;
}

static inline uint8_t canctrl_read(mcp251x_td *mcp251x)
{
    /* just return the value for now */
    return mcp251x->regs[MCP251x_REG_CANCTRL_H][MCP251x_REG_CANCTRL_L];
}

static inline uint8_t canctrl_write(mcp251x_td *mcp251x, uint8_t data, uint8_t mask)
{
    (void) mask;

    /* just return the value for now */
    mcp251x->regs[MCP251x_REG_CANCTRL_H][MCP251x_REG_CANCTRL_L] = data;
    return 0;
}

static inline uint8_t caninte_read(mcp251x_td *mcp251x)
{
    /* just return the value for now */
    return mcp251x->regs[MCP251x_REG_CANINTE_H][MCP251x_REG_CANINTE_L];
}

static inline uint8_t caninte_write(mcp251x_td *mcp251x, uint8_t data, uint8_t mask)
{
    (void) mask;

    mcp251x->merre = MCP251x_BIT_SET(data, MCP251x_CANINTE_MERRE);
    mcp251x->wakie = MCP251x_BIT_SET(data, MCP251x_CANINTE_WAKIE);
    mcp251x->errie = MCP251x_BIT_SET(data, MCP251x_CANINTE_ERRIE);
    mcp251x->tx2ie = MCP251x_BIT_SET(data, MCP251x_CANINTE_TX2IE);
    mcp251x->tx1ie = MCP251x_BIT_SET(data, MCP251x_CANINTE_TX1IE);
    mcp251x->tx0ie = MCP251x_BIT_SET(data, MCP251x_CANINTE_TX0IE);
    mcp251x->rx1ie = MCP251x_BIT_SET(data, MCP251x_CANINTE_RX1IE);
    mcp251x->rx0ie = MCP251x_BIT_SET(data, MCP251x_CANINTE_RX0IE);

    mcp251x->regs[MCP251x_REG_CANINTE_H][MCP251x_REG_CANINTE_L] = data;
    return 0;
}

static inline uint8_t canintf_read(mcp251x_td *mcp251x)
{
    uint8_t outdata = 0;
    if(mcp251x->merrf) MCP251x_SET_BIT(outdata, MCP251x_CANINTF_MERRF);
    if(mcp251x->wakif) MCP251x_SET_BIT(outdata, MCP251x_CANINTF_WAKIF);
    if(mcp251x->errif) MCP251x_SET_BIT(outdata, MCP251x_CANINTF_ERRIF);
    if(mcp251x->tx2if) MCP251x_SET_BIT(outdata, MCP251x_CANINTF_TX2IF);
    if(mcp251x->tx1if) MCP251x_SET_BIT(outdata, MCP251x_CANINTF_TX1IF);
    if(mcp251x->tx0if) MCP251x_SET_BIT(outdata, MCP251x_CANINTF_TX0IF);
    if(mcp251x->rx1if) MCP251x_SET_BIT(outdata, MCP251x_CANINTF_RX1IF);
    if(mcp251x->rx0if) MCP251x_SET_BIT(outdata, MCP251x_CANINTF_RX0IF);

    /* just return the value for now */
    return outdata;
}

static inline uint8_t canintf_write(mcp251x_td *mcp251x, uint8_t data, uint8_t mask)
{
    (void) mask;

    mcp251x->merrf = MCP251x_BIT_SET(data, MCP251x_CANINTF_MERRF);
    mcp251x->wakif = MCP251x_BIT_SET(data, MCP251x_CANINTF_WAKIF);
    mcp251x->errif = MCP251x_BIT_SET(data, MCP251x_CANINTF_ERRIF);
    mcp251x->tx2if = MCP251x_BIT_SET(data, MCP251x_CANINTF_TX2IF);
    mcp251x->tx1if = MCP251x_BIT_SET(data, MCP251x_CANINTF_TX1IF);
    mcp251x->tx0if = MCP251x_BIT_SET(data, MCP251x_CANINTF_TX0IF);
    mcp251x->rx1if = MCP251x_BIT_SET(data, MCP251x_CANINTF_RX1IF);
    mcp251x->rx0if = MCP251x_BIT_SET(data, MCP251x_CANINTF_RX0IF);

    // mcp251x->regs[MCP251x_REG_CANINTF_H][MCP251x_REG_BFPCTRL_TEC_CANINTF_L] = data;
    return 0;
}

static inline void handle_spi_write(mcp251x_td *mcp251x, uint8_t spi_data) 
{
    switch(mcp251x->ctrl_reg)
    {
        case CANSTAT:
            canstat_write(mcp251x, spi_data, 0xFF);
            break;
        case CANCTRL:
            canctrl_write(mcp251x, spi_data, 0xFF);
            break;
        case CANINTF:
            canintf_write(mcp251x, spi_data, 0xFF);
            break;
        default:
            break;
    }
    // mcp251x->spi_state = MCP2515_SPI_STATE_SPI_CMD;
}

/* handles SPI address */
static inline uint8_t handle_spi_addr(mcp251x_td *mcp251x, uint8_t spi_data)
{
    uint8_t higher_order_addr = (spi_data >> 4);
    uint8_t lower_order_addr = (spi_data & 0xF);
    uint8_t outdata = 0;

    mcp251x->reg_addr_h = higher_order_addr;
    mcp251x->reg_addr_l = lower_order_addr;

    if(mcp251x->spi_access_type == MCP2515_SPI_ACCESS_WRITE || 
       mcp251x->spi_access_type == MCP2515_SPI_ACCESS_MODIFY)
    {
        mcp251x->spi_state = MCP2515_SPI_STATE_SPI_WRITE; 
        return 0;
    }

    /* check if this is a control register */
    switch(lower_order_addr)
    {
        case MCP251x_REG_BFPCTRL_TEC_CANINTF_L:
            /* BFPCTRL */
            switch(higher_order_addr)
            {
                case MCP251x_REG_CANINTF_H:
                    outdata = canintf_read(mcp251x);
                    mcp251x->ctrl_reg = CANINTF;
                    break;
            }
            break;
        case 0xD:
            /* TXRTSCTRL */
            /* REC */
            /* EFLG */
            break;
        case MCP251x_REG_CANSTAT_L:
            /* CANSTAT */
            outdata = canstat_read(mcp251x);
            mcp251x->ctrl_reg = CANSTAT;
            break;
        case MCP251x_REG_CANCTRL_L:
            /* CANCTRL */
            outdata= canctrl_read(mcp251x);
            mcp251x->ctrl_reg = CANCTRL;
            break;
        case MCP251x_REG_CANINTE_L:
            /* CANCTRL */
            outdata= caninte_read(mcp251x);
            mcp251x->ctrl_reg = CANCTRL;
            break;
        default:
            break;
    }

    mcp251x->spi_state = MCP2515_SPI_STATE_SPI_READ_DUMMY; 

    return outdata;
}

static inline uint8_t mcp251x_spi_cmd_reset(mcp251x_td *mcp251x)
{
    MCP251x_SET_OPMODE(mcp251x->regs[MCP251x_REG_CANSTAT_H][MCP251x_REG_CANSTAT_L], MCP251x_OPMODE_CONFIG);
    MCP251x_SET_OPMODE(mcp251x->regs[MCP251x_REG_CANCTRL_H][MCP251x_REG_CANCTRL_L], MCP251x_OPMODE_CONFIG);
    MCP251x_CANCTRL_SET_CLKPRE(mcp251x->regs[MCP251x_REG_CANCTRL_H][MCP251x_REG_CANCTRL_L], MCP251x_CANCTRL_CLKPRE_DIV8);
    MCP251x_SET_BIT(mcp251x->regs[MCP251x_REG_CANCTRL_H][MCP251x_REG_CANCTRL_L], MCP251x_CANCTRL_CLKEN);

    /* Set back to 'idle' after reset */
    mcp251x->spi_state = MCP2515_SPI_STATE_SPI_CMD;

    return 0;
}

#include <libopencm3/stm32/gpio.h>

/* handles base SPI commands */
static inline uint8_t handle_spi_cmd(mcp251x_td *mcp251x, uint8_t spi_data)
{
    uint8_t base_cmd = (spi_data >> 4);
    uint8_t sub_cmd = (spi_data & 0xF);
    uint8_t spi_out = 0;

    switch(base_cmd)
    {
        case MCP251x_SPI_CMD_RESET_BASE:
            /* handle reset */
            mcp251x_spi_cmd_reset(mcp251x);
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
        case MCP251x_SPI_CMD_RTS_BASE:
            /* handle sending of TX buffers */
            /* trigger interrupt */
            mcp251x->tx0if = 1;
            gpio_clear(GPIOB, GPIO10);
            break;
    }

    return spi_out;
}

uint8_t mcp251x_spi_isr_handler(mcp251x_td *mcp251x, uint8_t spi_data)
{
    uint8_t outdata = 0;

    switch(mcp251x->spi_state)
    {
        case MCP2515_SPI_STATE_SPI_CMD:
            gpio_set(GPIOB, GPIO10);

            gpio_set(GPIOB, GPIO11);
            outdata = handle_spi_cmd(mcp251x, spi_data);
            gpio_clear(GPIOB, GPIO11);
            break;
        case MCP2515_SPI_STATE_SPI_ADDRESS:
            gpio_set(GPIOB, GPIO11);
            outdata = handle_spi_addr(mcp251x, spi_data);
            gpio_clear(GPIOB, GPIO11);
            break;
        case MCP2515_SPI_STATE_SPI_WRITE:
            gpio_set(GPIOB, GPIO11);
            handle_spi_write(mcp251x, spi_data);
            gpio_clear(GPIOB, GPIO11);
            break;
        case MCP2515_SPI_STATE_SPI_READ_DUMMY:
            // mcp251x->spi_state = MCP2515_SPI_STATE_SPI_CMD;
            break;
    }

    return outdata;
}

void mcp251x_reset_state(mcp251x_td *mcp251x)
{
    mcp251x->spi_state = MCP2515_SPI_STATE_SPI_CMD;
}

void mcp251x_spi_emu_init(mcp251x_td *mcp251x)
{
    memset(mcp251x->regs, 0, sizeof(mcp251x->regs));
}
