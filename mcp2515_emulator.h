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

/* MCP251x SPI Commands */
#define MCP251x_RESET_CMD 0xC0
#define MCP251x_WRITE_CMD 0x02
#define MCP251x_READ_CMD 0x03

/* MCP251 SPI Register addresses */
#define MCP251x_H_ORDER_ADDRS 8
#define MCP251x_L_ORDER_ADDRS 16
#define MCP251x_REG_CANSTAT_H 0x0 /* don't care in this case */
#define MCP251x_REG_CANSTAT_L 0xE
#define MCP251x_REG_CANCTRL_H 0x0 /* don't care in this case */
#define MCP251x_REG_CANCTRL_L 0xF

//#define MCP251x_REG_CANCTRL 0x0F
//#define MCP251x_REG_CANINTE 0x2B

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
    MCP2515_SPI_STATE_SPI_ACCESS,

} MCP251x_SPI_STATE;

typedef enum
{
    MCP2515_SPI_ACCESS_READ = 0x00,
    MCP2515_SPI_ACCESS_WRITE,
    MCP2515_SPI_ACCESS_MODIFY,

} MCP251x_SPI_ACCESS_TYPE;

typedef struct mcp251x_struct mcp251x_td;
typedef struct mcp251x_struct
{
    MCP251x_SPI_STATE spi_state;
    MCP251x_SPI_ACCESS_TYPE spi_access_type;
    uint8_t reg_addr_h;
    uint8_t reg_addr_l;
    uint8_t modify_mask;
    uint8_t regs[MCP251x_H_ORDER_ADDRS][MCP251x_L_ORDER_ADDRS];
    uint8_t (*special_reg_access_read_cb)(mcp251x_td *mcp251x);
    uint8_t (*special_reg_access_write_cb)(mcp251x_td *mcp251x, uint8_t data, uint8_t mask);

} mcp251x_td;

void mcp2515_emu_init(mcp251x_td *mcp251x);
void mcp2515_emu_rx_buf_process(void);

#endif /* MCP2515_EMULATOR_H */
