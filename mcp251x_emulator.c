#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include <mcp251x_emulator.h>

#include <usart3_printf.h>

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

// RXB0CTRL Register Bits (Address: 0x60)
#define MCP251x_RXB0CTRL_RXM1      (1 << 6) // Bit 6: Receive Buffer Mode bit 1
#define MCP251x_RXB0CTRL_RXM0      (1 << 5) // Bit 5: Receive Buffer Mode bit 0
#define MCP251x_RXB0CTRL_RXRTR     (1 << 3) // Bit 3: Receive Remote Transmission Request
#define MCP251x_RXB0CTRL_BUKT      (1 << 2) // Bit 2: Bus Utilization Control (Overflow Flag)
#define MCP251x_RXB0CTRL_BUKT1     (1 << 1) // Bit 1: Bus Utilization Control bit1
#define MCP251x_RXB0CTRL_FILHIT0   (1 << 0) // Bit 0: Filter Hit Index bit0

// RXB1CTRL Register Bits (Address: 0x60)
#define MCP251x_RXB1CTRL_RXM1      (1 << 6) // Bit 6: Receive Buffer Mode bit 1
#define MCP251x_RXB1CTRL_RXM0      (1 << 5) // Bit 5: Receive Buffer Mode bit 0
#define MCP251x_RXB1CTRL_RXRTR     (1 << 3) // Bit 3: Receive Remote Transmission Request
#define MCP251x_RXB1CTRL_FILHIT2   (1 << 2) // Bit 2: Bus Utilization Control (Overflow Flag)
#define MCP251x_RXB1CTRL_FILHIT1   (1 << 1) // Bit 1: Bus Utilization Control bit1
#define MCP251x_RXB1CTRL_FILHIT0   (1 << 0) // Bit 0: Filter Hit Index bit0

// TXB0CTRL/TXB1CTRL/TXB1CTRL Register Bits (Address: 0x60)
#define MCP251x_TXBxCTRL_ABTF      (1 << 6)
#define MCP251x_TXBxCTRL_MLOA      (1 << 5)
#define MCP251x_TXBxCTRL_TXERR     (1 << 4)
#define MCP251x_TXBxCTRL_TXREQ     (1 << 3)
#define MCP251x_TXBxCTRL_TXP1      (1 << 1)
#define MCP251x_TXBxCTRL_TXP0      (1 << 0)

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

#define MCP251x_REG_TXBx_RXBx_CTRL_L 0x0

/* RXB0 */
#define MCP251x_REG_RXB0CTRL 0x60
#define MCP251x_REG_RXB0CTRL_H 0x6
#define MCP251x_REG_RXB0CTRL_L 0x0

/* RXB1 */
#define MCP251x_REG_RXB1CTRL 0x70
#define MCP251x_REG_RXB1CTRL_H 0x7
#define MCP251x_REG_RXB1CTRL_L 0x0

/* TXB0CTRL */
#define MCP251x_REG_TXB0CTRL 0x30
#define MCP251x_REG_TXB0CTRL_H 0x3
#define MCP251x_REG_TXB0CTRL_L 0x0

/* TXB1CTRL */
#define MCP251x_REG_TXB1CTRL 0x40
#define MCP251x_REG_TXB1CTRL_H 0x4
#define MCP251x_REG_TXB1CTRL_L 0x0

/* TXB2CTRL */
#define MCP251x_REG_TXB2CTRL 0x50
#define MCP251x_REG_TXB2CTRL_H 0x5
#define MCP251x_REG_TXB2CTRL_L 0x0

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

typedef enum
{
    MCP251x_OPMODE_NORMAL = 0x00,
    MCP251x_OPMODE_SLEEP = 0x01,
    MCP251x_OPMODE_LOOPBACK = 0x02,
    MCP251x_OPMODE_LISTEN_ONLY = 0x03,
    MCP251x_OPMODE_CONFIG = 0x04

} MCP251x_OPMODE;

static inline void check_txreq(mcp251x_td *mcp251x, uint8_t data, uint8_t *txbnctrl)
{
    if(MCP251x_BIT_SET(data, MCP251x_TXBxCTRL_TXREQ))
    {
        MCP251x_CLEAR_BIT(*txbnctrl, MCP251x_TXBxCTRL_ABTF);
        MCP251x_CLEAR_BIT(*txbnctrl, MCP251x_TXBxCTRL_MLOA);
        MCP251x_CLEAR_BIT(*txbnctrl, MCP251x_TXBxCTRL_TXERR);
        MCP251x_SET_BIT(*txbnctrl, MCP251x_TXBxCTRL_TXREQ);
        task_queue_push(&mcp251x->can_tx_irq_queue, mcp251x->can_txb0_cb, NULL);
    }
}

static inline uint8_t txb0ctrl_read(mcp251x_td *mcp251x)
{
    // printf("%s unimplemented\r\n", __func__);
    // while(1);
    return mcp251x->txb0ctrl;
}

static inline uint8_t txb0ctrl_write(mcp251x_td *mcp251x, uint8_t data, uint8_t mask)
{
    uint8_t tmp = data & mask;
    check_txreq(mcp251x, tmp, &mcp251x->txb0ctrl);

    // printf("%s unimplemented\r\n", __func__);
    // while(1);
    return 0;
}

static inline uint8_t txb1ctrl_read(mcp251x_td *mcp251x)
{
    printf("%s unimplemented\r\n", __func__);
    while(1);
    return mcp251x->txb1ctrl;
}

static inline uint8_t txb1ctrl_write(mcp251x_td *mcp251x, uint8_t data, uint8_t mask)
{
    mcp251x->txb1ctrl = data & mask;

    printf("%s unimplemented\r\n", __func__);
    while(1);
    return 0;
}

static inline uint8_t txb2ctrl_read(mcp251x_td *mcp251x)
{
    printf("%s unimplemented\r\n", __func__);
    while(1);
    return mcp251x->txb2ctrl;
}

static inline uint8_t txb2ctrl_write(mcp251x_td *mcp251x, uint8_t data, uint8_t mask)
{
    mcp251x->txb2ctrl = data & mask;

    printf("%s unimplemented\r\n", __func__);
    while(1);
    return 0;
}

static inline uint8_t canstat_read(mcp251x_td *mcp251x)
{
    /* just return the value for now */
    return mcp251x->canstat;
}

static inline uint8_t canstat_write(mcp251x_td *mcp251x, uint8_t data, uint8_t mask)
{
    /* just return the value for now */
    mcp251x->canstat = data & mask;
    return 0;
}

static inline uint8_t canctrl_read(mcp251x_td *mcp251x)
{
    /* just return the value for now */
    return mcp251x->canctrl;
}

static inline uint8_t canctrl_write(mcp251x_td *mcp251x, uint8_t data, uint8_t mask)
{
    int opmode = 0;

    /* just return the value for now */
    mcp251x->canctrl = data & mask;

    opmode = MCP251x_GET_OPMODE(mcp251x->canctrl);

    /* also set the canstat for now */
    MCP251x_SET_OPMODE(mcp251x->canstat, opmode);
    return 0;
}

static inline uint8_t caninte_read(mcp251x_td *mcp251x)
{
    uint8_t outdata = 0;

    if(mcp251x->merre) MCP251x_SET_BIT(outdata, MCP251x_CANINTE_MERRE);
    if(mcp251x->wakie) MCP251x_SET_BIT(outdata, MCP251x_CANINTE_WAKIE);
    if(mcp251x->errie) MCP251x_SET_BIT(outdata, MCP251x_CANINTE_ERRIE);
    if(mcp251x->tx2ie) MCP251x_SET_BIT(outdata, MCP251x_CANINTE_TX2IE);
    if(mcp251x->tx1ie) MCP251x_SET_BIT(outdata, MCP251x_CANINTE_TX1IE);
    if(mcp251x->tx0ie) MCP251x_SET_BIT(outdata, MCP251x_CANINTE_TX0IE);
    if(mcp251x->rx1ie) MCP251x_SET_BIT(outdata, MCP251x_CANINTE_RX1IE);
    if(mcp251x->rx0ie) MCP251x_SET_BIT(outdata, MCP251x_CANINTE_RX0IE);

    /* just return the value for now */
    return outdata;
}

static inline uint8_t caninte_write(mcp251x_td *mcp251x, uint8_t data, uint8_t mask)
{
    if (mask & MCP251x_CANINTE_MERRE)
        mcp251x->merre = MCP251x_BIT_SET(data, MCP251x_CANINTE_MERRE);

    if (mask & MCP251x_CANINTE_WAKIE)
        mcp251x->wakie = MCP251x_BIT_SET(data, MCP251x_CANINTE_WAKIE);

    if (mask & MCP251x_CANINTE_ERRIE)
        mcp251x->errie = MCP251x_BIT_SET(data, MCP251x_CANINTE_ERRIE);

    if (mask & MCP251x_CANINTE_TX2IE)
        mcp251x->tx2ie = MCP251x_BIT_SET(data, MCP251x_CANINTE_TX2IE);

    if (mask & MCP251x_CANINTE_TX1IE)
        mcp251x->tx1ie = MCP251x_BIT_SET(data, MCP251x_CANINTE_TX1IE);

    if (mask & MCP251x_CANINTE_TX0IE)
        mcp251x->tx0ie = MCP251x_BIT_SET(data, MCP251x_CANINTE_TX0IE);

    if (mask & MCP251x_CANINTE_RX1IE)
        mcp251x->rx1ie = MCP251x_BIT_SET(data, MCP251x_CANINTE_RX1IE);

    if (mask & MCP251x_CANINTE_RX0IE)
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
    if (mask & MCP251x_CANINTF_MERRF)
        mcp251x->merrf = MCP251x_BIT_SET(data, MCP251x_CANINTF_MERRF);

    if (mask & MCP251x_CANINTF_WAKIF)
        mcp251x->wakif = MCP251x_BIT_SET(data, MCP251x_CANINTF_WAKIF);

    if (mask & MCP251x_CANINTF_ERRIF)
        mcp251x->errif = MCP251x_BIT_SET(data, MCP251x_CANINTF_ERRIF);

    if (mask & MCP251x_CANINTF_TX2IF)
        mcp251x->tx2if = MCP251x_BIT_SET(data, MCP251x_CANINTF_TX2IF);

    if (mask & MCP251x_CANINTF_TX1IF)
        mcp251x->tx1if = MCP251x_BIT_SET(data, MCP251x_CANINTF_TX1IF);

    if (mask & MCP251x_CANINTF_TX0IF)
        mcp251x->tx0if = MCP251x_BIT_SET(data, MCP251x_CANINTF_TX0IF);

    if (mask & MCP251x_CANINTF_RX1IF)
        mcp251x->rx1if = MCP251x_BIT_SET(data, MCP251x_CANINTF_RX1IF);

    if (mask & MCP251x_CANINTF_RX0IF)
        mcp251x->rx0if = MCP251x_BIT_SET(data, MCP251x_CANINTF_RX0IF);

    return 0;
}

static inline void handle_spi_write(mcp251x_td *mcp251x, uint8_t spi_data)
{
    switch(mcp251x->ctrl_reg)
    {
        case TXB0CTRL:
            txb0ctrl_write(mcp251x, spi_data, mcp251x->modify_mask);
            break;
        case TXB1CTRL:
            txb1ctrl_write(mcp251x, spi_data, mcp251x->modify_mask);
            break;
        case TXB2CTRL:
            txb2ctrl_write(mcp251x, spi_data, mcp251x->modify_mask);
            break;
        case CANSTAT:
            canstat_write(mcp251x, spi_data, mcp251x->modify_mask);
            break;
        case CANCTRL:
            canctrl_write(mcp251x, spi_data, mcp251x->modify_mask);
            break;
        case CANINTE:
            caninte_write(mcp251x, spi_data, mcp251x->modify_mask);
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

    switch(mcp251x->load_addr_h)
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

    tx_buf[mcp251x->load_addr_l] = spi_data;
    mcp251x->load_addr_l++;

    if(mcp251x->load_addr_l > MCP251x_TXB_RXB_REG_SIZE)
    {
        mcp251x->load_addr_l = 0;
        mcp251x_reset_state(mcp251x);
    }
}

static inline uint8_t handle_read_rx_buffer(mcp251x_td *mcp251x)
{
    uint8_t *rx_buf = NULL;
    uint8_t ret_val = 0;

    switch(mcp251x->load_addr_h)
    {
        case 6:
            rx_buf = mcp251x->rxb0;
            break;
        case 7:
            rx_buf = mcp251x->rxb1;
            break;
    }

    if(!rx_buf)
        return 0;

    ret_val = rx_buf[mcp251x->load_addr_l];
    mcp251x->load_addr_l++;

    if(mcp251x->load_addr_l >= MCP251x_TXB_RXB_REG_SIZE)
    {
        mcp251x->load_addr_l = 0;

        /* clear rx0if */
        mcp251x->rx0if = 0;
        mcp251x_reset_state(mcp251x);
    }

    return ret_val;
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
            mcp251x->ctrl_reg = CANINTE;
            break;
        /* CANINTF */
        /* EFLG */
        case MCP251x_REG_TXBx_RXBx_CTRL_L:
            switch(higher_order_addr)
            {
                /* TXB0 */
                case MCP251x_REG_TXB0CTRL_H:
                    outdata = txb0ctrl_read(mcp251x);
                    mcp251x->ctrl_reg = TXB0CTRL;
                    break;
                /* TXB1 */
                case MCP251x_REG_TXB1CTRL_H:
                    outdata = txb1ctrl_read(mcp251x);
                    mcp251x->ctrl_reg = TXB1CTRL;
                    break;
                /* TXB2 */
                case MCP251x_REG_TXB2CTRL_H:
                    outdata = txb2ctrl_read(mcp251x);
                    mcp251x->ctrl_reg = TXB2CTRL;
                    break;
                /* RXB0 */
                /* RXB1 */
            }
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
    int txb = 0;

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
                    mcp251x->load_addr_h = 3;
                    mcp251x->load_addr_l = 0;
                    break;
                case 1:
                    mcp251x->load_addr_h = 3;
                    mcp251x->load_addr_l = 5;
                    break;
                case 2:
                    mcp251x->load_addr_h = 4;
                    mcp251x->load_addr_l = 0;
                    break;
                case 3:
                    mcp251x->load_addr_h = 4;
                    mcp251x->load_addr_l = 5;
                    break;
                case 4:
                    mcp251x->load_addr_h = 5;
                    mcp251x->load_addr_l = 0;
                    break;
                case 5:
                    mcp251x->load_addr_h = 5;
                    mcp251x->load_addr_l = 5;
                    break;
            }
            mcp251x->spi_state = mcp251x_SPI_STATE_SPI_LOAD_TX;
            break;
        case MCP251x_SPI_CMD_RX_STATUS_BASE:
            mcp251x->spi_state = mcp251x_SPI_STATE_SPI_READ_STATUS;
            break;
        case MCP251x_SPI_CMD_READ_RX_BUFFER_BASE:
            mcp251x->spi_state = mcp251x_SPI_STATE_SPI_READ_RX_BUFFER;
            int nm = (sub_cmd >> 1);
            switch(nm)
            {
                case 0:
                    mcp251x->load_addr_h = 6;
                    mcp251x->load_addr_l = 0;
                    break;
                case 1:
                    mcp251x->load_addr_h = 6;
                    mcp251x->load_addr_l = 5;
                    break;
                case 2:
                    mcp251x->load_addr_h = 7;
                    mcp251x->load_addr_l = 0;
                    break;
                case 3:
                    mcp251x->load_addr_h = 7;
                    mcp251x->load_addr_l = 5;
                    break;
            }
            spi_out = handle_read_rx_buffer(mcp251x);
            break;
        case MCP251x_SPI_CMD_RTS_BASE:
            txb = (sub_cmd & ~(MCP251x_SPI_CMD_RTS_MASK));
            if(txb & MCP251x_SPI_CMD_RTS_TXB0_CMD)
                task_queue_push(&mcp251x->can_tx_irq_queue, mcp251x->can_txb0_cb, NULL);
            if(txb & MCP251x_SPI_CMD_RTS_TXB1_CMD)
                task_queue_push(&mcp251x->can_tx_irq_queue, mcp251x->can_txb1_cb, NULL);
            if(txb & MCP251x_SPI_CMD_RTS_TXB2_CMD)
                task_queue_push(&mcp251x->can_tx_irq_queue, mcp251x->can_txb2_cb, NULL);
            break;
    }

    return spi_out;
}

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
        case mcp251x_SPI_STATE_SPI_READ_RX_BUFFER:
            outdata = handle_read_rx_buffer(mcp251x);
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

void mcp251x_spi_emu_init(mcp251x_td *mcp251x, 
                          void (*can_txb0_cb)(void *priv), 
                          void (*can_txb1_cb)(void *priv),
                          void (*can_txb2_cb)(void *priv),
                          void (*set_irq_cb)(int high))
{
    memset(mcp251x, 0, sizeof(mcp251x_td));

    mcp251x->can_txb0_cb = can_txb0_cb;
    mcp251x->can_txb1_cb = can_txb1_cb;
    mcp251x->can_txb2_cb = can_txb2_cb;
    task_queue_init(&mcp251x->can_tx_irq_queue, 5);

    mcp251x->set_irq_cb = set_irq_cb;

    mcp251x_reset_state(mcp251x);
}

void mcp251x_emu_can_tx_irq_process(mcp251x_td *mcp251x)
{
    task_queue_pull(&mcp251x->can_tx_irq_queue);
}

void mcp251x_emu_set_irq_flag(mcp251x_td *mcp251x, MCP251x_IRQ_FLAGS irq_flag)
{    
    switch (irq_flag) 
    {
        case INTERRUPT_MERRF: mcp251x->merrf = 1; break;
        case INTERRUPT_WAKIF: mcp251x->wakif = 1; break;
        case INTERRUPT_ERRIF: mcp251x->errif = 1; break;
        case INTERRUPT_TX2IF: mcp251x->tx2if = 1; break;
        case INTERRUPT_TX1IF: mcp251x->tx1if = 1; break;
        case INTERRUPT_TX0IF: mcp251x->tx0if = 1; break;
        case INTERRUPT_RX1IF: mcp251x->rx1if = 1; break;
        case INTERRUPT_RX0IF: mcp251x->rx0if = 1; break;
        default:
            break;
    }
}

void mcp251x_emu_handle_txb_done(mcp251x_td *mcp251x, MCP251x_IRQ_FLAGS txb)
{
    mcp251x_emu_set_irq_flag(mcp251x, txb);

    if( (txb == INTERRUPT_TX0IF) && (mcp251x->tx0if) )
        /* clear TXREQ */
        MCP251x_CLEAR_BIT(mcp251x->txb0ctrl, MCP251x_TXBxCTRL_TXREQ);
    else if( (txb == INTERRUPT_TX1IF) && (mcp251x->tx1if) )
        /* clear TXREQ */
        MCP251x_CLEAR_BIT(mcp251x->txb1ctrl, MCP251x_TXBxCTRL_TXREQ);
    else if( (txb == INTERRUPT_TX2IF) && (mcp251x->tx2if) )
        /* clear TXREQ */
        MCP251x_CLEAR_BIT(mcp251x->txb2ctrl, MCP251x_TXBxCTRL_TXREQ);

    if(mcp251x->tx0ie || mcp251x->tx1ie || mcp251x->tx2ie)
        mcp251x->set_irq_cb(0);
}

static inline uint8_t MCP251x_TXB_flag_to_mask(MCP251x_TXB_ERROR_FLAGS flag)
{
    switch (flag)
    {
        case ERR_ABTF:
            return MCP251x_TXBxCTRL_ABTF;
        case ERR_MLOA:
            return MCP251x_TXBxCTRL_MLOA;
        case ERR_TXERR:
            return MCP251x_TXBxCTRL_TXERR;
        case ERR_NONE:
        default:
            return 0; // No bits set
    }
}

void mcp251x_emu_handle_txb_error(mcp251x_td *mcp251x, MCP251x_CTRL_REGS txbnctrl, MCP251x_TXB_ERROR_FLAGS flag)
{
    uint8_t *reg_ptr = NULL;

    switch (txbnctrl) 
    {
        case TXB0CTRL:
            reg_ptr = &mcp251x->txb0ctrl;
            break;
        case TXB1CTRL:
            reg_ptr = &mcp251x->txb1ctrl;
            break;
        case TXB2CTRL:
            reg_ptr = &mcp251x->txb2ctrl;
            break;
        default:
            return; // No valid register selected
    }

    *reg_ptr |= MCP251x_TXB_flag_to_mask(flag);

    if((*reg_ptr & MCP251x_TXBxCTRL_TXERR))
    {
        canintf_write(mcp251x, MCP251x_CANINTF_MERRF, 0xFF);

        if(mcp251x->merre)
            mcp251x->set_irq_cb(0);
    }
}

void mcp251x_emu_rx_data(mcp251x_td *mcp251x)
{
    mcp251x->rxb0[0] = 0x321 >> 3;
    mcp251x->rxb0[1] = (0x321 & 7) << 5;
    mcp251x->rxb0[2] = 0x3;
    mcp251x->rxb0[3] = 0x21;
    mcp251x->rxb0[4] = 0x8;
    mcp251x->rxb0[5] = 0x42;
    mcp251x->rxb0[6] = 0x43;
    mcp251x->rxb0[7] = 0x44;
    mcp251x->rxb0[8] = 0x45;
    mcp251x->rxb0[9] = 0x46;
    mcp251x->rxb0[10] = 0x47;
    mcp251x->rxb0[11] = 0x48;
    mcp251x->rxb0[12] = 0x49;

    mcp251x_emu_set_irq_flag(mcp251x, INTERRUPT_RX0IF);
    mcp251x->set_irq_cb(0);

}


/************************************************* DEBUG BUFFER TRACE *************************************************/
#define DECODE_BIT(val, bitmask, name) (((val) & (bitmask)) ? (name) : "x")

/* Callback for Processing Received Data */
void rx_buf_trace_queue_cb(void *priv)
{
    trace_buffer_t *rx_trace_buffer = priv;

    int msg_len = rx_trace_buffer->trace_buf[rx_trace_buffer->buf_read_location].msg_len;

    /* Validate msg_len to prevent buffer overread */
    if (msg_len > RX_TRACE_BUF_SIZE)
    {
        usart3_printf("Error: Message length %d exceeds buffer size.\r\n", msg_len);
        msg_len = RX_TRACE_BUF_SIZE; /* Adjust as needed */
    }

    // printf("WR: %u RD: %u ", rx_trace_buffer->buf_write_location, rx_trace_buffer->buf_read_location);
    for(int i = 0; i < msg_len; i++)
    {
        usart3_printf("%02x ", rx_trace_buffer->trace_buf[rx_trace_buffer->buf_read_location].buf[i]);
        /* mark this buffer as read by setting the msg_len to 0 */
        rx_trace_buffer->trace_buf[rx_trace_buffer->buf_read_location].msg_len = 0;
    }

    usart3_printf("\r\n");

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
        usart3_printf("Warning: Too much data to process!\r\n");

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
        usart3_printf("Warning: RX buffer overflow. Data byte 0x%02x discarded.\r\n", indata);
        /* Optionally, reset buf_count or implement other overflow handling */
    }
}

void mcp251x_emu_rx_trace_buf_process(trace_buffer_t *rx_trace_buffer)
{
    task_queue_pull(&rx_trace_buffer->buf_task_queue);
}
