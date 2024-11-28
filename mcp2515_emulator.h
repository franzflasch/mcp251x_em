#ifndef MCP2515_EMULATOR_H
#define MCP2515_EMULATOR_H

#include <stdint.h>

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
#define MCP251x_CANINTE_MERRE   0x80  // Bit 7: Message Error Interrupt Enable
#define MCP251x_CANINTE_WAKIE   0x40  // Bit 6: Wake-up Interrupt Enable
#define MCP251x_CANINTE_ERRIE   0x20  // Bit 5: Error Interrupt Enable
#define MCP251x_CANINTE_TX2IE   0x10  // Bit 4: Transmit Buffer 2 Empty Interrupt Enable
#define MCP251x_CANINTE_TX1IE   0x08  // Bit 3: Transmit Buffer 1 Empty Interrupt Enable
#define MCP251x_CANINTE_TX0IE   0x04  // Bit 2: Transmit Buffer 0 Empty Interrupt Enable
#define MCP251x_CANINTE_RX1IE   0x02  // Bit 1: Receive Buffer 1 Full Interrupt Enable
#define MCP251x_CANINTE_RX0IE   0x01  // Bit 0: Receive Buffer 0 Full Interrupt Enable

/* CANINTF Register Bits */
/* Bit Masks */
#define MCP251x_CANINTF_MERRF   0x80  // Bit 7: Message Error Interrupt Enable
#define MCP251x_CANINTF_WAKIF   0x40  // Bit 6: Wake-up Interrupt Enable
#define MCP251x_CANINTF_ERRIF   0x20  // Bit 5: Error Interrupt Enable
#define MCP251x_CANINTF_TX2IF   0x10  // Bit 4: Transmit Buffer 2 Empty Interrupt Enable
#define MCP251x_CANINTF_TX1IF   0x08  // Bit 3: Transmit Buffer 1 Empty Interrupt Enable
#define MCP251x_CANINTF_TX0IF   0x04  // Bit 2: Transmit Buffer 0 Empty Interrupt Enable
#define MCP251x_CANINTF_RX1IF   0x02  // Bit 1: Receive Buffer 1 Full Interrupt Enable
#define MCP251x_CANINTF_RX0IF   0x01  // Bit 0: Receive Buffer 0 Full Interrupt Enable

// CNF1 Register Bits (Address: 0x2A)
#define MCP251x_CNF1_SJW1   0x80  // Bit 7: Synchronization Jump Width bit 1
#define MCP251x_CNF1_SJW0   0x40  // Bit 6: Synchronization Jump Width bit 0
#define MCP251x_CNF1_BRP5   0x20  // Bit 5: Baud Rate Prescaler bit 5
#define MCP251x_CNF1_BRP4   0x10  // Bit 4: Baud Rate Prescaler bit 4
#define MCP251x_CNF1_BRP3   0x08  // Bit 3: Baud Rate Prescaler bit 3
#define MCP251x_CNF1_BRP2   0x04  // Bit 2: Baud Rate Prescaler bit 2
#define MCP251x_CNF1_BRP1   0x02  // Bit 1: Baud Rate Prescaler bit 1
#define MCP251x_CNF1_BRP0   0x01  // Bit 0: Baud Rate Prescaler bit 0

// CNF2 Register Bits (Address: 0x29)
#define MCP251x_CNF2_BTLMODE   0x80  // Bit 7: PS2 Bit Time Lengthening Mode
#define MCP251x_CNF2_SAM       0x40  // Bit 6: Sample Point Configuration
#define MCP251x_CNF2_PHSEG12   0x20  // Bit 5: PHSEG1 bit 2
#define MCP251x_CNF2_PHSEG11   0x10  // Bit 4: PHSEG1 bit 1
#define MCP251x_CNF2_PHSEG10   0x08  // Bit 3: PHSEG1 bit 0
#define MCP251x_CNF2_PRSEG2    0x04  // Bit 2: Propagation Segment bit 2
#define MCP251x_CNF2_PRSEG1    0x02  // Bit 1: Propagation Segment bit 1
#define MCP251x_CNF2_PRSEG0    0x01  // Bit 0: Propagation Segment bit 0

// CNF3 Register Bits (Address: 0x28)
#define MCP251x_CNF3_SOF       0x80  // Bit 7: Start-of-Frame Signal Bit
#define MCP251x_CNF3_WAKFIL    0x40  // Bit 6: Wake-up Filter Enable
// Bits 5-3 are unused/reserved and should be set to '0'
#define MCP251x_CNF3_PHSEG22   0x04  // Bit 2: PHSEG2 bit 2
#define MCP251x_CNF3_PHSEG21   0x02  // Bit 1: PHSEG2 bit 1
#define MCP251x_CNF3_PHSEG20   0x01  // Bit 0: PHSEG2 bit 0

// RXB0CTRL/RXB1CTRL Register Bits (Address: 0x60)
#define MCP251x_RXBxCTRL_RXM1      0x40  // Bit 6: Receive Buffer Mode bit 1
#define MCP251x_RXBxCTRL_RXM0      0x20  // Bit 5: Receive Buffer Mode bit 0
#define MCP251x_RXBxCTRL_RXRTR     0x08  // Bit 3: Receive Remote Transmission Request
#define MCP251x_RXBxCTRL_BUKT      0x04  // Bit 2: Bus Utilization Control (Overflow Flag)
#define MCP251x_RXBxCTRL_BUKT1     0x02  // Bit 1: Bus Utilization Control bit1
#define MCP251x_RXBxCTRL_FILHIT0   0x01  // Bit 0: Filter Hit Index bit0

/* MCP251x SPI Commands */
#define MCP251x_RESET_CMD 0xC0
#define MCP251x_WRITE_CMD 0x02
#define MCP251x_READ_CMD 0x03
#define MCP251x_LOAD_TX_BUFFER 0x40
#define MCP251x_LOAD_TX_BUFFER_MASK 0xF8

/* MCP251 SPI Register addresses */
#define MCP251x_H_ORDER_ADDRS 8
#define MCP251x_L_ORDER_ADDRS 16

#define MCP251x_REG_CANSTAT 0x0E
#define MCP251x_REG_CANSTAT_H 0x0 /* don't care in this case */
#define MCP251x_REG_CANSTAT_L 0xE

#define MCP251x_REG_CANCTRL 0x0F
#define MCP251x_REG_CANCTRL_H 0x0 /* don't care in this case */
#define MCP251x_REG_CANCTRL_L 0xF

#define MCP251x_REG_CANINTE 0x2B
#define MCP251x_REG_CANINTE_H 0x2
#define MCP251x_REG_CANINTE_L 0xB

#define MCP251x_REG_CANINTF 0x2C
#define MCP251x_REG_CANINTF_H 0x2

#define MCP251x_REG_BFPCTRL_TEC_CANINTF_L 0xC


#define MCP251x_REG_CNFx_H 0x2
#define MCP251x_REG_CNF1 0x2A
#define MCP251x_REG_CNF1_L 0xA
#define MCP251x_REG_CNF2 0x29
#define MCP251x_REG_CNF2_L 0x9
#define MCP251x_REG_CNF3 0x28
#define MCP251x_REG_CNF3_L 0x8

#define MCP251x_REG_RXB0CTRL 0x60
#define MCP251x_REG_RXB0CTRL_H 0x6
#define MCP251x_REG_RXB0CTRL_L 0x0

#define MCP251x_REG_RXB1CTRL 0x70
#define MCP251x_REG_RXB1CTRL_H 0x7
#define MCP251x_REG_RXB1CTRL_L 0x0

typedef enum
{
    MCP251x_OPMODE_NORMAL = 0x00,
    MCP251x_OPMODE_SLEEP = 0x01,
    MCP251x_OPMODE_LOOPBACK = 0x02,
    MCP251x_OPMODE_LISTEN_ONLY = 0x03,
    MCP251x_OPMODE_CONFIG = 0x04

} MCP251x_OPMODE;

typedef enum
{
    MCP2515_SPI_STATE_SPI_CMD = 0x00,
    MCP2515_SPI_STATE_SPI_ADDRESS,
    MCP2515_SPI_STATE_SPI_WRITE,
    MCP2515_SPI_STATE_SPI_READ_DUMMY,

} MCP251x_SPI_STATE;

typedef enum
{
    MCP2515_SPI_ACCESS_READ = 0x00,
    MCP2515_SPI_ACCESS_WRITE,
    MCP2515_SPI_ACCESS_MODIFY,

} MCP251x_SPI_ACCESS_TYPE;

typedef enum
{
    NONE = 0,
    BFPCTRL,
    TXRTSCTRL,
    CANSTAT,
    CANCTRL,
    TEC,
    REC,
    CNF3,
    CNF2,
    CNF1,
    CANINTE,
    CANINTF,
    EFLG,
    TXB0CTRL,
    TXB1CTRL,
    TXB2CTRL,
    RXB0CTRL,
    RXB1CTRL

} MCP251x_CTRL_REGS;

typedef struct mcp251x_struct mcp251x_td;
typedef struct mcp251x_struct
{
    MCP251x_SPI_STATE spi_state;
    MCP251x_SPI_ACCESS_TYPE spi_access_type;
    uint8_t reg_addr_h;
    uint8_t reg_addr_l;
    uint8_t modify_mask;
    uint8_t regs[MCP251x_H_ORDER_ADDRS][MCP251x_L_ORDER_ADDRS];
    MCP251x_CTRL_REGS ctrl_reg;

    /* interrupt enable */
    int merre;
    int wakie;
    int errie;
    int tx2ie;
    int tx1ie;
    int tx0ie;
    int rx1ie;
    int rx0ie;

    /* interrupt flags */
    int merrf;
    int wakif;
    int errif;
    int tx2if;
    int tx1if;
    int tx0if;
    int rx1if;
    int rx0if;

} mcp251x_td;

void mcp2515_emu_init(mcp251x_td *mcp251x);
void mcp2515_emu_rx_buf_process(void);

uint8_t mcp251x_spi_isr_handler(mcp251x_td *mcp251x, uint8_t spi_data);
void mcp251x_reset_state(mcp251x_td *mcp251x);
void mcp251x_spi_emu_init(mcp251x_td *mcp251x);

#endif /* MCP2515_EMULATOR_H */
