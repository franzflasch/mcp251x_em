#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include <mcp251x_emulator.h>

#define MCP251x_BIT_SET(x, mask) (((x) & (mask)) != 0)
#define MCP251x_SET_BIT(reg, bitmask)   ((reg) |= (bitmask))
#define MCP251x_CLEAR_BIT(reg, bitmask) ((reg) &= ~(bitmask))
#define MCP251x_MODIFY_BIT(reg, bitmask, value) \
    ((reg) = ((reg) & ~(bitmask)) | ((value) ? (bitmask) : 0))

/* Common Operation Mode Bits (bits 7-5) for CANSTAT and CANCTRL */
#define MCP251x_OPMODE_MASK           0xE0
#define MCP251x_OPMODE_SHIFT          5

#define MCP251x_SET_OPMODE(reg, mode) \
    ((reg) = ((reg) & ~MCP251x_OPMODE_MASK) | \
    ((mode) << MCP251x_OPMODE_SHIFT))

#define MCP251x_GET_OPMODE(reg) \
    (((reg) & MCP251x_OPMODE_MASK) >> MCP251x_OPMODE_SHIFT)

/* CANSTAT Register Bits */
#define MCP251x_CANSTAT_ICOD_MASK         0x0E
#define MCP251x_CANSTAT_ICOD_SHIFT        1

#define MCP251x_CANSTAT_ICOD_NOINTERRUPT  0
#define MCP251x_CANSTAT_ICOD_ERROR        1
#define MCP251x_CANSTAT_ICOD_WAKEUP       2
#define MCP251x_CANSTAT_ICOD_TXB0         3
#define MCP251x_CANSTAT_ICOD_TXB1         4
#define MCP251x_CANSTAT_ICOD_TXB2         5
#define MCP251x_CANSTAT_ICOD_RXB0         6
#define MCP251x_CANSTAT_ICOD_RXB1         7

/* CANCTRL Register Bits */
#define MCP251x_CANCTRL_ABAT              0x10  // 1 << 4
#define MCP251x_CANCTRL_OSM               0x08  // 1 << 3
#define MCP251x_CANCTRL_CLKEN             0x04  // 1 << 2

#define MCP251x_CANCTRL_CLKPRE_MASK       0x03
#define MCP251x_CANCTRL_CLKPRE_SHIFT      0

#define MCP251x_CANCTRL_CLKPRE_DIV1       0x00  // 00: System Clock / 1
#define MCP251x_CANCTRL_CLKPRE_DIV2       0x01  // 01: System Clock / 2
#define MCP251x_CANCTRL_CLKPRE_DIV4       0x02  // 10: System Clock / 4
#define MCP251x_CANCTRL_CLKPRE_DIV8       0x03  // 11: System Clock / 8

#define MCP251x_CANCTRL_SET_CLKPRE(reg, prescaler) \
    ((reg) = ((reg) & ~MCP251x_CANCTRL_CLKPRE_MASK) | \
    ((prescaler) << MCP251x_CANCTRL_CLKPRE_SHIFT))

#define MCP251x_CANCTRL_GET_CLKPRE(reg) \
    (((reg) & MCP251x_CANCTRL_CLKPRE_MASK) >> MCP251x_CANCTRL_CLKPRE_SHIFT)

/* CANINTE Register Bits */
/* Bit Masks */
#define MCP251x_CANINTE_MERRE   (1 << 7) // Bit 7: Message Error Interrupt Enable
#define MCP251x_CANINTE_WAKIE   (1 << 6) // Bit 6: Wake-up Interrupt Enable
#define MCP251x_CANINTE_ERRIE   (1 << 5) // Bit 5: Error Interrupt Enable
#define MCP251x_CANINTE_TX2IE   (1 << 4) // Bit 4: Transmit Buffer 2 Empty Interrupt Enable
#define MCP251x_CANINTE_TX1IE   (1 << 3) // Bit 3: Transmit Buffer 1 Empty Interrupt Enable
#define MCP251x_CANINTE_TX0IE   (1 << 2) // Bit 2: Transmit Buffer 0 Empty Interrupt Enable
#define MCP251x_CANINTE_RX1IE   (1 << 1) // Bit 1: Receive Buffer 1 Full Interrupt Enable
#define MCP251x_CANINTE_RX0IE   (1 << 0) // Bit 0: Receive Buffer 0 Full Interrupt Enable

/* CANINTF Register Bits */
/* Bit Masks */
#define MCP251x_CANINTF_MERRF   (1 << 7) // Bit 7: Message Error Interrupt Flag
#define MCP251x_CANINTF_WAKIF   (1 << 6) // Bit 6: Wake-up Interrupt Flag
#define MCP251x_CANINTF_ERRIF   (1 << 5) // Bit 5: Error Interrupt Flag
#define MCP251x_CANINTF_TX2IF   (1 << 4) // Bit 4: Transmit Buffer 2 Empty Interrupt Flag
#define MCP251x_CANINTF_TX1IF   (1 << 3) // Bit 3: Transmit Buffer 1 Empty Interrupt Flag
#define MCP251x_CANINTF_TX0IF   (1 << 2) // Bit 2: Transmit Buffer 0 Empty Interrupt Flag
#define MCP251x_CANINTF_RX1IF   (1 << 1) // Bit 1: Receive Buffer 1 Full Interrupt Flag
#define MCP251x_CANINTF_RX0IF   (1 << 0) // Bit 0: Receive Buffer 0 Full Interrupt Flag

// CNF1 Register Bits (Address: 0x2A)
#define MCP251x_CNF1_SJW1   (1 << 7) // Bit 7: Synchronization Jump Width bit 1
#define MCP251x_CNF1_SJW0   (1 << 6) // Bit 6: Synchronization Jump Width bit 0
#define MCP251x_CNF1_BRP5   (1 << 5) // Bit 5: Baud Rate Prescaler bit 5
#define MCP251x_CNF1_BRP4   (1 << 4) // Bit 4: Baud Rate Prescaler bit 4
#define MCP251x_CNF1_BRP3   (1 << 3) // Bit 3: Baud Rate Prescaler bit 3
#define MCP251x_CNF1_BRP2   (1 << 2) // Bit 2: Baud Rate Prescaler bit 2
#define MCP251x_CNF1_BRP1   (1 << 1) // Bit 1: Baud Rate Prescaler bit 1
#define MCP251x_CNF1_BRP0   (1 << 0) // Bit 0: Baud Rate Prescaler bit 0

// CNF2 Register Bits (Address: 0x29)
#define MCP251x_CNF2_BTLMODE   (1 << 7) // Bit 7: PS2 Bit Time Lengthening Mode
#define MCP251x_CNF2_SAM       (1 << 6) // Bit 6: Sample Point Configuration
#define MCP251x_CNF2_PHSEG12   (1 << 5) // Bit 5: PHSEG1 bit 2
#define MCP251x_CNF2_PHSEG11   (1 << 4) // Bit 4: PHSEG1 bit 1
#define MCP251x_CNF2_PHSEG10   (1 << 3) // Bit 3: PHSEG1 bit 0
#define MCP251x_CNF2_PRSEG2    (1 << 2) // Bit 2: Propagation Segment bit 2
#define MCP251x_CNF2_PRSEG1    (1 << 1) // Bit 1: Propagation Segment bit 1
#define MCP251x_CNF2_PRSEG0    (1 << 0) // Bit 0: Propagation Segment bit 0

// CNF3 Register Bits (Address: 0x28)
#define MCP251x_CNF3_SOF       (1 << 7) // Bit 7: Start-of-Frame Signal Bit
#define MCP251x_CNF3_WAKFIL    (1 << 6) // Bit 6: Wake-up Filter Enable
// Bits 5-3 are unused/reserved and should be set to '0'
#define MCP251x_CNF3_PHSEG22   (1 << 2) // Bit 2: PHSEG2 bit 2
#define MCP251x_CNF3_PHSEG21   (1 << 1) // Bit 1: PHSEG2 bit 1
#define MCP251x_CNF3_PHSEG20   (1 << 0) // Bit 0: PHSEG2 bit 0

// RXB0CTRL/RXB1CTRL Register Bits (Address: 0x60)
#define MCP251x_RXBxCTRL_RXM1      (1 << 6) // Bit 6: Receive Buffer Mode bit 1
#define MCP251x_RXBxCTRL_RXM0      (1 << 5) // Bit 5: Receive Buffer Mode bit 0
#define MCP251x_RXBxCTRL_RXRTR     (1 << 3) // Bit 3: Receive Remote Transmission Request
#define MCP251x_RXBxCTRL_BUKT      (1 << 2) // Bit 2: Bus Utilization Control (Overflow Flag)
#define MCP251x_RXBxCTRL_BUKT1     (1 << 1) // Bit 1: Bus Utilization Control bit1
#define MCP251x_RXBxCTRL_FILHIT0   (1 << 0) // Bit 0: Filter Hit Index bit0


/* MCP251 SPI Register addresses */
/* CANSTAT */
#define MCP251x_REG_CANSTAT 0x0E
#define MCP251x_REG_CANSTAT_H 0x0 /* don't care in this case */
#define MCP251x_REG_CANSTAT_L 0xE

/* CANCTRL */
#define MCP251x_REG_CANCTRL 0x0F
#define MCP251x_REG_CANCTRL_H 0x0 /* don't care in this case */
#define MCP251x_REG_CANCTRL_L 0xF

/* CANINTE */
#define MCP251x_REG_CANINTE 0x2B
#define MCP251x_REG_CANINTE_H 0x2
#define MCP251x_REG_CANINTE_L 0xB

/* CANINTF */
#define MCP251x_REG_CANINTF 0x2C
#define MCP251x_REG_CANINTF_H 0x2

/* common BFPCTRL_TEC_CANINTF_L */
#define MCP251x_REG_BFPCTRL_TEC_CANINTF_L 0xC

/* CNFx */
#define MCP251x_REG_CNFx_H 0x2
#define MCP251x_REG_CNF1 0x2A
#define MCP251x_REG_CNF1_L 0xA
#define MCP251x_REG_CNF2 0x29
#define MCP251x_REG_CNF2_L 0x9
#define MCP251x_REG_CNF3 0x28
#define MCP251x_REG_CNF3_L 0x8

/* RXB0 */
#define MCP251x_REG_RXB0CTRL 0x60
#define MCP251x_REG_RXB0CTRL_H 0x6
#define MCP251x_REG_RXB0CTRL_L 0x0

/* RXB1 */
#define MCP251x_REG_RXB1CTRL 0x70
#define MCP251x_REG_RXB1CTRL_H 0x7
#define MCP251x_REG_RXB1CTRL_L 0x0


/* MCP251x SPI CMDs */
#define MCP251x_SPI_CMD_RESET 0xC0
#define MCP251x_SPI_CMD_RESET_BASE 0xC
#define MCP251x_SPI_CMD_RESET_CMD  0x0

#define MCP251x_SPI_CMD_READ 0x03
#define MCP251x_SPI_CMD_WRITE 0x02
#define MCP251x_SPI_CMD_BIT_MODIFY 0x05
#define MCP251x_SPI_CMD_READ_WRITE_BIT_MODIFY_BASE 0x0
#define MCP251x_SPI_CMD_READ_CMD_SUB 0x3
#define MCP251x_SPI_CMD_WRITE_CMD_SUB 0x2
#define MCP251x_SPI_CMD_BIT_MODIFY_CMD_SUB 0x5

#define MCP251x_SPI_CMD_READ_RX_BUFFER 0x90
#define MCP251x_SPI_CMD_READ_RX_BUFFER_MASK 0xF9
#define MCP251x_SPI_CMD_READ_RX_BUFFER_BASE 0x9
#define MCP251x_SPI_CMD_READ_RX_BUFFER0_CMD 0x0
#define MCP251x_SPI_CMD_READ_RX_BUFFER1_CMD 0x2
#define MCP251x_SPI_CMD_READ_RX_BUFFER2_CMD 0x4
#define MCP251x_SPI_CMD_READ_RX_BUFFER3_CMD 0x6

#define MCP251x_SPI_CMD_LOAD_TX_BUFFER 0x40
#define MCP251x_SPI_CMD_LOAD_TX_BUFFER_MASK 0xF8
#define MCP251x_SPI_CMD_LOAD_TX_BUFFER_BASE 0x4
#define MCP251x_SPI_CMD_LOAD_TX_BUFFER0_CMD 0x0
#define MCP251x_SPI_CMD_LOAD_TX_BUFFER1_CMD 0x1
#define MCP251x_SPI_CMD_LOAD_TX_BUFFER2_CMD 0x2
#define MCP251x_SPI_CMD_LOAD_TX_BUFFER3_CMD 0x3
#define MCP251x_SPI_CMD_LOAD_TX_BUFFER4_CMD 0x4
#define MCP251x_SPI_CMD_LOAD_TX_BUFFER5_CMD 0x5

#define MCP251x_SPI_CMD_RTS 0x80
#define MCP251x_SPI_CMD_RTS_MASK 0xF8
#define MCP251x_SPI_CMD_RTS_BASE 0x8
#define MCP251x_SPI_CMD_RTS_TXB0_CMD 0x1
#define MCP251x_SPI_CMD_RTS_TXB1_CMD 0x2
#define MCP251x_SPI_CMD_RTS_TXB2_CMD 0x4

#define MCP251x_SPI_CMD_READ_STATUS 0xA0
#define MCP251x_SPI_CMD_READ_STATUS_BASE 0xA
#define MCP251x_SPI_CMD_READ_STATUS_CMD 0x0

#define MCP251x_SPI_CMD_RX_STATUS 0xB0
#define MCP251x_SPI_CMD_RX_STATUS_BASE 0xB
#define MCP251x_SPI_CMD_RX_STATUS_CMD 0x0

#define MCP251x_SPI_CMD_TXBx_RXBx_CTRL_BASE 0x0

typedef enum
{
    MCP251x_OPMODE_NORMAL = 0x00,
    MCP251x_OPMODE_SLEEP = 0x01,
    MCP251x_OPMODE_LOOPBACK = 0x02,
    MCP251x_OPMODE_LISTEN_ONLY = 0x03,
    MCP251x_OPMODE_CONFIG = 0x04

} MCP251x_OPMODE;

static inline uint8_t canstat_read(mcp251x_td *mcp251x)
{
    /* just return the value for now */
    return mcp251x->canstat;
}

static inline uint8_t canstat_write(mcp251x_td *mcp251x, uint8_t data, uint8_t mask)
{
    (void) mask;
    /* just return the value for now */
    mcp251x->canstat = data;
    return 0;
}

static inline uint8_t canctrl_read(mcp251x_td *mcp251x)
{
    /* just return the value for now */
    return mcp251x->canctrl;
}

static inline uint8_t canctrl_write(mcp251x_td *mcp251x, uint8_t data, uint8_t mask)
{
    (void) mask;

    int opmode = MCP251x_GET_OPMODE(data);

    /* just return the value for now */
    mcp251x->canctrl = data;

    /* also set the canstat for now */
    MCP251x_SET_OPMODE(mcp251x->canstat, opmode);
    return 0;
}

static inline uint8_t caninte_read(mcp251x_td *mcp251x)
{
    uint8_t outdata = 0;

    if(mcp251x->merrf) MCP251x_SET_BIT(outdata, MCP251x_CANINTE_MERRE);
    if(mcp251x->wakif) MCP251x_SET_BIT(outdata, MCP251x_CANINTE_WAKIE);
    if(mcp251x->errif) MCP251x_SET_BIT(outdata, MCP251x_CANINTE_ERRIE);
    if(mcp251x->tx2if) MCP251x_SET_BIT(outdata, MCP251x_CANINTE_TX2IE);
    if(mcp251x->tx1if) MCP251x_SET_BIT(outdata, MCP251x_CANINTE_TX1IE);
    if(mcp251x->tx0if) MCP251x_SET_BIT(outdata, MCP251x_CANINTE_TX0IE);
    if(mcp251x->rx1if) MCP251x_SET_BIT(outdata, MCP251x_CANINTE_RX1IE);
    if(mcp251x->rx0if) MCP251x_SET_BIT(outdata, MCP251x_CANINTE_RX0IE);

    /* just return the value for now */
    return outdata;
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

    return 0;
}

static inline void handle_spi_write(mcp251x_td *mcp251x, uint8_t spi_data)
{
    switch(mcp251x->ctrl_reg)
    {
        case CANSTAT:
            canstat_write(mcp251x, spi_data, mcp251x->modify_mask);
            break;
        case CANCTRL:
            canctrl_write(mcp251x, spi_data, mcp251x->modify_mask);
            break;
        case CANINTF:
            canintf_write(mcp251x, spi_data, mcp251x->modify_mask);
            break;
        default:
            break;
    }
}

static inline void handle_load_tx(mcp251x_td *mcp251x, uint8_t spi_data)
{
    uint8_t *tx_buf = NULL;

    switch(mcp251x->load_tx_addr_h)
    {
        case 3:
            tx_buf = mcp251x->txb0;
            break;
        case 4:
            tx_buf = mcp251x->txb1;
            break;
        case 5:
            tx_buf = mcp251x->txb2;
            break;
    }

    if(!tx_buf)
        return;

    tx_buf[mcp251x->load_tx_addr_l] = spi_data;
    mcp251x->load_tx_addr_l++;

    if(mcp251x->load_tx_addr_l > MCP251x_TXB_RXB_REG_SIZE)
    {
        mcp251x->load_tx_addr_l = 0;
        mcp251x_reset_state(mcp251x);
    }
}

/* handles SPI address */
static inline uint8_t handle_spi_addr(mcp251x_td *mcp251x, uint8_t spi_data)
{
    uint8_t higher_order_addr = (spi_data >> 4);
    uint8_t lower_order_addr = (spi_data & 0xF);
    uint8_t outdata = 0;
    int is_read = 0;

    mcp251x->reg_addr_h = higher_order_addr;
    mcp251x->reg_addr_l = lower_order_addr;

    if(mcp251x->spi_access_type == mcp251x_SPI_ACCESS_WRITE)
    {
        mcp251x->spi_state = mcp251x_SPI_STATE_SPI_WRITE;
        mcp251x->modify_mask = 0xFF;
    }
    else if(mcp251x->spi_access_type == mcp251x_SPI_ACCESS_MODIFY)
    {
        mcp251x->spi_state = mcp251x_SPI_STATE_SPI_BIT_MODIFY;
    }
    else
    {
        is_read = 1;
        mcp251x->spi_state = mcp251x_SPI_STATE_SPI_READ_DUMMY;
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
            outdata=canstat_read(mcp251x);
            mcp251x->ctrl_reg = CANSTAT;
            break;
        case MCP251x_REG_CANCTRL_L:
            /* CANCTRL */
            outdata=canctrl_read(mcp251x);
            mcp251x->ctrl_reg = CANCTRL;
            break;
        /* TEC */
        /* REC */
        /* CNF3 */
        case MCP251x_REG_CNF3_L:
            break;
        /* CNF2 */
        case MCP251x_REG_CNF2_L:
            break;
        /* CNF1 */
        case MCP251x_REG_CNF1_L:
            break;
        case MCP251x_REG_CANINTE_L:
            /* CANCTRL */
            outdata=caninte_read(mcp251x);
            mcp251x->ctrl_reg = CANCTRL;
            break;
        /* CANINTF */
        /* EFLG */
        case MCP251x_SPI_CMD_TXBx_RXBx_CTRL_BASE:
            /* TXB0 */
            /* TXB1 */
            /* TXB2 */
            /* RXB0 */
            /* RXB1 */
            break;
        default:
            printf("unknown address! %x\r\n", spi_data);
            break;
    }

    /* only return a value if it is a read */
    if(!is_read)
        outdata = 0;

    return outdata;
}

static inline uint8_t mcp251x_spi_cmd_reset(mcp251x_td *mcp251x)
{
    MCP251x_SET_OPMODE(mcp251x->canstat, MCP251x_OPMODE_CONFIG);
    MCP251x_SET_OPMODE(mcp251x->canctrl, MCP251x_OPMODE_CONFIG);
    MCP251x_CANCTRL_SET_CLKPRE(mcp251x->canctrl, MCP251x_CANCTRL_CLKPRE_DIV8);
    MCP251x_SET_BIT(mcp251x->canctrl, MCP251x_CANCTRL_CLKEN);

    /* Set back to 'idle' after reset */
    mcp251x->spi_state = mcp251x_SPI_STATE_SPI_CMD;

    return 0;
}

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
                case MCP251x_SPI_CMD_READ_CMD_SUB:
                    mcp251x->spi_access_type = mcp251x_SPI_ACCESS_READ;
                    break;
                case MCP251x_SPI_CMD_WRITE_CMD_SUB:
                    mcp251x->spi_access_type = mcp251x_SPI_ACCESS_WRITE;
                    break;
                case MCP251x_SPI_CMD_BIT_MODIFY_CMD_SUB:
                    mcp251x->spi_access_type = mcp251x_SPI_ACCESS_MODIFY;
                    break;
            }
            mcp251x->spi_state = mcp251x_SPI_STATE_SPI_ADDRESS;
            break;
        case MCP251x_SPI_CMD_LOAD_TX_BUFFER_BASE:
            switch(sub_cmd)
            {
                case 0:
                    mcp251x->load_tx_addr_h = 3;
                    mcp251x->load_tx_addr_l = 0;
                    break;
                case 1:
                    mcp251x->load_tx_addr_h = 3;
                    mcp251x->load_tx_addr_l = 5;
                    break;
                case 2:
                    mcp251x->load_tx_addr_h = 4;
                    mcp251x->load_tx_addr_l = 0;
                    break;
                case 3:
                    mcp251x->load_tx_addr_h = 4;
                    mcp251x->load_tx_addr_l = 5;
                    break;
                case 4:
                    mcp251x->load_tx_addr_h = 5;
                    mcp251x->load_tx_addr_l = 0;
                    break;
                case 5:
                    mcp251x->load_tx_addr_h = 5;
                    mcp251x->load_tx_addr_l = 5;
                    break;
            }
            mcp251x->spi_state = mcp251x_SPI_STATE_SPI_LOAD_TX;
            break;
        case MCP251x_SPI_CMD_RX_STATUS_BASE:
            mcp251x->spi_state = mcp251x_SPI_STATE_SPI_READ_STATUS;
            break;
        case MCP251x_SPI_CMD_RTS_BASE:
            task_queue_push(&mcp251x->can_tx_irq_queue, mcp251x->can_tx_irq_queue_cb, NULL);
            break;
    }

    return spi_out;
}

#include <libopencm3/stm32/gpio.h>

uint8_t mcp251x_spi_isr_handler(mcp251x_td *mcp251x, uint8_t spi_data)
{
    uint8_t outdata = 0;

    switch(mcp251x->spi_state)
    {
        case mcp251x_SPI_STATE_SPI_CMD:
            outdata = handle_spi_cmd(mcp251x, spi_data);
            break;
        case mcp251x_SPI_STATE_SPI_ADDRESS:
            outdata = handle_spi_addr(mcp251x, spi_data);
            break;
        case mcp251x_SPI_STATE_SPI_WRITE:
            handle_spi_write(mcp251x, spi_data);
            break;
        case mcp251x_SPI_STATE_SPI_BIT_MODIFY:
            mcp251x->modify_mask = spi_data;
            mcp251x->spi_state = mcp251x_SPI_STATE_SPI_WRITE;
        case mcp251x_SPI_STATE_SPI_READ_DUMMY:
            break;
        case mcp251x_SPI_STATE_SPI_LOAD_TX:
            handle_load_tx(mcp251x, spi_data);
            break;
        case mcp251x_SPI_STATE_SPI_READ_STATUS:
            outdata = 0;
            break;
    }

    return outdata;
}

void mcp251x_reset_state(mcp251x_td *mcp251x)
{
    mcp251x->set_irq_cb(1);
    mcp251x->spi_state = mcp251x_SPI_STATE_SPI_CMD;
}

void mcp251x_spi_emu_init(mcp251x_td *mcp251x, void (*can_tx_irq_cb)(void *priv), void (*set_irq_cb)(int high))
{
    memset(mcp251x, 0, sizeof(mcp251x_td));

    mcp251x->can_tx_irq_queue_cb = can_tx_irq_cb;
    task_queue_init(&mcp251x->can_tx_irq_queue, 5);

    mcp251x->set_irq_cb = set_irq_cb;

    mcp251x_reset_state(mcp251x);
}

void mcp251x_emu_can_tx_irq_process(mcp251x_td *mcp251x)
{
    task_queue_pull(&mcp251x->can_tx_irq_queue);
}



/************ DEBUG BUFFER TRACE *****************/
#define DECODE_BIT(val, bitmask, name) (((val) & (bitmask)) ? (name) : "x")

/* Callback for Processing Received Data */
void rx_buf_trace_queue_cb(void *priv)
{
    trace_buffer_t *rx_trace_buffer = priv;

    int msg_len = rx_trace_buffer->trace_buf[rx_trace_buffer->buf_read_location].msg_len;
    int curr_reg = 0;
    int curr_reg_val = 0;

    /* Validate msg_len to prevent buffer overread */
    if (msg_len > RX_TRACE_BUF_SIZE)
    {
        printf("Error: Message length %d exceeds buffer size.\r\n", msg_len);
        msg_len = RX_TRACE_BUF_SIZE; /* Adjust as needed */
    }

    printf("WR: %u RD: %u ", rx_trace_buffer->buf_write_location, rx_trace_buffer->buf_read_location);
    for(int i = 0; i < msg_len; i++)
    {
        printf("%02x ", rx_trace_buffer->trace_buf[rx_trace_buffer->buf_read_location].buf[i]);
        /* mark this buffer as read by setting the msg_len to 0 */
        rx_trace_buffer->trace_buf[rx_trace_buffer->buf_read_location].msg_len = 0;
    }

    for(int i = 0; i < msg_len; i++)
    {
        /* CMD */
        if(i==0)
        {
            switch(rx_trace_buffer->trace_buf[rx_trace_buffer->buf_read_location].buf[0])
            {
                case MCP251x_SPI_CMD_RESET:
                    printf("RST ");
                break;
                case MCP251x_SPI_CMD_READ:
                    printf("RD ");
                break;
                case MCP251x_SPI_CMD_WRITE:
                    printf("WR ");
                break;
                case MCP251x_SPI_CMD_BIT_MODIFY:
                    printf("BIT MODIFY ");
                break;
                default:
                break;
            }

            if( (rx_trace_buffer->trace_buf[rx_trace_buffer->buf_read_location].buf[0] & MCP251x_SPI_CMD_LOAD_TX_BUFFER_MASK) == MCP251x_SPI_CMD_LOAD_TX_BUFFER)
                printf("LOAD_TX ");
        }
        /* ADDRESS */
        else if(i==1)
        {
            curr_reg = rx_trace_buffer->trace_buf[rx_trace_buffer->buf_read_location].buf[1];
            switch(curr_reg)
            {
                case MCP251x_REG_CANCTRL:
                    printf("CANCTRL ");
                break;
                case MCP251x_REG_CANSTAT:
                    printf("CANSTAT ");
                break;
                case MCP251x_REG_CANINTE:
                    printf("CANINTE ");
                break;
                case MCP251x_REG_CNF1:
                    printf("CNF1 ");
                break;
                case MCP251x_REG_CNF2:
                    printf("CNF2 ");
                break;
                case MCP251x_REG_CNF3:
                    printf("CNF3 ");
                break;
                case MCP251x_REG_RXB1CTRL:
                    printf("RXB1CTRL ");
                break;
                case MCP251x_REG_RXB0CTRL:
                    printf("RXB0CTRL ");
                break;
                default:
                break;
            }
        }
        else if(i==2)
        {
            curr_reg_val = rx_trace_buffer->trace_buf[rx_trace_buffer->buf_read_location].buf[2];
            switch(curr_reg)
            {
                case MCP251x_REG_RXB1CTRL:
                case MCP251x_REG_RXB0CTRL:
                {
                    printf("%s:%s:%s:%s:%s:%s:%s",
                            DECODE_BIT(curr_reg_val, MCP251x_RXBxCTRL_RXM1, "RXM1"),
                            DECODE_BIT(curr_reg_val, MCP251x_RXBxCTRL_RXM0, "RXM0"),
                            DECODE_BIT(curr_reg_val, 0x10, "-"),
                            DECODE_BIT(curr_reg_val, MCP251x_RXBxCTRL_RXRTR, "RXRTR"),
                            DECODE_BIT(curr_reg_val, MCP251x_RXBxCTRL_BUKT, "BUKT"),
                            DECODE_BIT(curr_reg_val, MCP251x_RXBxCTRL_BUKT1, "BUKT1"),
                            DECODE_BIT(curr_reg_val, MCP251x_RXBxCTRL_FILHIT0, "FILHIT0")
                            );
                    break;
                }
                case MCP251x_REG_CNF1:
                {
                    printf("%s:%s:%s:%s:%s:%s:%s:%s",
                            DECODE_BIT(curr_reg_val, MCP251x_CNF1_SJW1, "SJW1"),
                            DECODE_BIT(curr_reg_val, MCP251x_CNF1_SJW0, "SJW0"),
                            DECODE_BIT(curr_reg_val, MCP251x_CNF1_BRP5, "BRP5"),
                            DECODE_BIT(curr_reg_val, MCP251x_CNF1_BRP4, "BRP4"),
                            DECODE_BIT(curr_reg_val, MCP251x_CNF1_BRP3, "BRP3"),
                            DECODE_BIT(curr_reg_val, MCP251x_CNF1_BRP2, "BRP2"),
                            DECODE_BIT(curr_reg_val, MCP251x_CNF1_BRP1, "BRP1"),
                            DECODE_BIT(curr_reg_val, MCP251x_CNF1_BRP0, "BRP0")
                            );
                    break;
                }
                case MCP251x_REG_CNF2:
                {
                    printf("%s:%s:%s:%s:%s:%s:%s:%s",
                            DECODE_BIT(curr_reg_val, MCP251x_CNF2_BTLMODE, "BTLMODE"),
                            DECODE_BIT(curr_reg_val, MCP251x_CNF2_SAM, "SAM"),
                            DECODE_BIT(curr_reg_val, MCP251x_CNF2_PHSEG12, "PHSEG12"),
                            DECODE_BIT(curr_reg_val, MCP251x_CNF2_PHSEG11, "PHSEG11"),
                            DECODE_BIT(curr_reg_val, MCP251x_CNF2_PHSEG10, "PHSEG10"),
                            DECODE_BIT(curr_reg_val, MCP251x_CNF2_PRSEG2, "PRSEG2"),
                            DECODE_BIT(curr_reg_val, MCP251x_CNF2_PRSEG1, "PRSEG1"),
                            DECODE_BIT(curr_reg_val, MCP251x_CNF2_PRSEG0, "PRSEG0")
                            );
                    break;
                }
                case MCP251x_REG_CNF3:
                {
                    printf("%s:%s:%s:%s:%s:%s:%s:%s",
                            DECODE_BIT(curr_reg_val, MCP251x_CNF3_SOF, "SOF"),
                            DECODE_BIT(curr_reg_val, MCP251x_CNF3_WAKFIL, "WAKFIL"),
                            DECODE_BIT(curr_reg_val, 0x20, "-"),
                            DECODE_BIT(curr_reg_val, 0x10, "-"),
                            DECODE_BIT(curr_reg_val, 0x08, "-"),
                            DECODE_BIT(curr_reg_val, MCP251x_CNF3_PHSEG22, "PRSEG22"),
                            DECODE_BIT(curr_reg_val, MCP251x_CNF3_PHSEG21, "PRSEG21"),
                            DECODE_BIT(curr_reg_val, MCP251x_CNF3_PHSEG20, "PRSEG20")
                            );
                    break;
                }
                case MCP251x_REG_CANCTRL:
                {
                    int opmode = curr_reg_val >> MCP251x_OPMODE_SHIFT;
                    printf("%s:%s:%s:%s:%s:%s:%s:%s",
                            opmode == MCP251x_OPMODE_NORMAL ? "NORMAL" :
                            opmode == MCP251x_OPMODE_SLEEP ? "SLEEP" :
                            opmode == MCP251x_OPMODE_LOOPBACK ? "LOOPBACK" :
                            opmode == MCP251x_OPMODE_LISTEN_ONLY ? "LISTEN" : "CONFIG",
                            DECODE_BIT(curr_reg_val, MCP251x_CANCTRL_ABAT, "ABAT"),
                            DECODE_BIT(curr_reg_val, MCP251x_CANCTRL_OSM, "OSM"),
                            DECODE_BIT(curr_reg_val, MCP251x_CANCTRL_CLKEN, "CLKEN"),
                            DECODE_BIT(curr_reg_val, MCP251x_CANCTRL_CLKPRE_DIV8, "CLKPRE_DIV8"),
                            DECODE_BIT(curr_reg_val, MCP251x_CANCTRL_CLKPRE_DIV4, "CLKPRE_DIV4"),
                            DECODE_BIT(curr_reg_val, MCP251x_CANCTRL_CLKPRE_DIV2, "CLKPRE_DIV2"),
                            DECODE_BIT(curr_reg_val, MCP251x_CANCTRL_CLKPRE_DIV1, "CLKPRE_DIV1")
                            );
                    break;
                }
                case MCP251x_REG_CANSTAT:
                {
                    int opmode = curr_reg_val >> MCP251x_OPMODE_SHIFT;
                    int icod = ((curr_reg_val >> MCP251x_CANSTAT_ICOD_SHIFT) & MCP251x_CANSTAT_ICOD_MASK);
                    printf("%s %s",
                            opmode == MCP251x_OPMODE_NORMAL ? "NORMAL" :
                            opmode == MCP251x_OPMODE_SLEEP ? "SLEEP" :
                            opmode == MCP251x_OPMODE_LOOPBACK ? "LOOPBACK" :
                            opmode == MCP251x_OPMODE_LISTEN_ONLY ? "LISTEN" : "CONFIG",
                            icod == MCP251x_CANSTAT_ICOD_NOINTERRUPT ? "NOINT" :
                            icod == MCP251x_CANSTAT_ICOD_ERROR ? "ERROR" :
                            icod == MCP251x_CANSTAT_ICOD_WAKEUP ? "WAKEUP" :
                            icod == MCP251x_CANSTAT_ICOD_TXB0 ? "TXB0" :
                            icod == MCP251x_CANSTAT_ICOD_TXB1 ? "TXB1" :
                            icod == MCP251x_CANSTAT_ICOD_TXB2 ? "TXB2" :
                            icod == MCP251x_CANSTAT_ICOD_RXB0 ? "RXB0" : "RXB1"
                            );
                    break;
                }
                case MCP251x_REG_CANINTE:
                {
                    int caninte = curr_reg_val;
                    printf("%s %s %s %s %s %s %s %s",
                            caninte & MCP251x_CANINTE_MERRE ? "MERRE" : "x",
                            caninte & MCP251x_CANINTE_WAKIE ? "WAKIE" : "x",
                            caninte & MCP251x_CANINTE_ERRIE ? "ERRIE" : "x",
                            caninte & MCP251x_CANINTE_TX2IE ? "TX2IE" : "x",
                            caninte & MCP251x_CANINTE_TX1IE ? "TX1IE" : "x",
                            caninte & MCP251x_CANINTE_TX0IE ? "TX0IE" : "x",
                            caninte & MCP251x_CANINTE_RX1IE ? "RX1IE" : "x",
                            caninte & MCP251x_CANINTE_RX0IE ? "RX0IE" : "x"
                            );
                    break;
                }
                default:
                break;
            }
        }
    }

    printf("\r\n");

    rx_trace_buffer->buf_read_location = (rx_trace_buffer->buf_read_location + 1) % RX_TRACE_BUF_ELEMENTS;
}

void mcp251x_emu_rx_trace_buf_init(trace_buffer_t *rx_trace_buffer)
{
    task_queue_init(&rx_trace_buffer->buf_task_queue, RX_TRACE_BUF_ELEMENTS);
}

void mcp251x_emu_rx_trace_buf_add(trace_buffer_t *rx_trace_buffer)
{
    rx_trace_buffer->buf_write_location = (rx_trace_buffer->buf_write_location + 1) % RX_TRACE_BUF_ELEMENTS;
    rx_trace_buffer->buf_count = 0;

    /* check if we are producing faster than the data can be read */
    if(rx_trace_buffer->trace_buf[rx_trace_buffer->buf_write_location].msg_len)
        printf("Warning: Too much data to process!\r\n");

    task_queue_push(&rx_trace_buffer->buf_task_queue, rx_buf_trace_queue_cb, rx_trace_buffer);
}

void mcp251x_emu_rx_trace_buf_add_data(trace_buffer_t *rx_trace_buffer, uint8_t indata)
{
    if (rx_trace_buffer->buf_count < RX_TRACE_BUF_SIZE) 
    {
        rx_trace_buffer->trace_buf[rx_trace_buffer->buf_write_location].buf[rx_trace_buffer->buf_count++] = indata;
        rx_trace_buffer->trace_buf[rx_trace_buffer->buf_write_location].msg_len = rx_trace_buffer->buf_count;
    }
    else 
    {
        /* Handle buffer overflow if necessary */
        printf("Warning: RX buffer overflow. Data byte 0x%02x discarded.\r\n", indata);
        /* Optionally, reset buf_count or implement other overflow handling */
    }
}

void mcp251x_emu_rx_trace_buf_process(trace_buffer_t *rx_trace_buffer)
{
    task_queue_pull(&rx_trace_buffer->buf_task_queue);
}
