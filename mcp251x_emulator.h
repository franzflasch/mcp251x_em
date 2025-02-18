#ifndef mcp251x_EMULATOR_H
#define mcp251x_EMULATOR_H

#include <stdint.h>
#include <task_queue.h>

#define ENABLE_RX_TRACE_BUF

#define MCP251x_TXB_RXB_REG_SIZE 13

typedef enum
{
    mcp251x_SPI_STATE_SPI_CMD = 0x00,
    mcp251x_SPI_STATE_SPI_ADDRESS,
    mcp251x_SPI_STATE_SPI_WRITE,
    mcp251x_SPI_STATE_SPI_BIT_MODIFY,
    mcp251x_SPI_STATE_SPI_READ_DUMMY,
    mcp251x_SPI_STATE_SPI_LOAD_TX,
    mcp251x_SPI_STATE_SPI_READ_STATUS,
    mcp251x_SPI_STATE_SPI_READ_RX_BUFFER,

} MCP251x_SPI_STATE;

typedef enum
{
    mcp251x_SPI_ACCESS_READ = 0x00,
    mcp251x_SPI_ACCESS_WRITE,
    mcp251x_SPI_ACCESS_MODIFY,

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

typedef enum 
{
    INTERRUPT_MERRF,
    INTERRUPT_WAKIF,
    INTERRUPT_ERRIF,
    INTERRUPT_TX2IF,
    INTERRUPT_TX1IF,
    INTERRUPT_TX0IF,
    INTERRUPT_RX1IF,
    INTERRUPT_RX0IF

} MCP251x_IRQ_FLAGS;

typedef enum 
{
    ERR_NONE = 0,
    ERR_ABTF,
    ERR_MLOA,
    ERR_TXERR,

} MCP251x_TXB_ERROR_FLAGS;

typedef struct mcp251x_struct mcp251x_td;
typedef struct mcp251x_struct
{
    task_queue_td can_tx_irq_queue;
    void (*can_txb0_cb)(void *priv);
    void (*can_txb1_cb)(void *priv);
    void (*can_txb2_cb)(void *priv);

    void (*set_irq_cb)(int high);

    MCP251x_SPI_STATE spi_state;
    MCP251x_SPI_ACCESS_TYPE spi_access_type;
    uint8_t reg_addr_h;
    uint8_t reg_addr_l;
    uint8_t modify_mask;
    uint8_t load_addr_h;
    uint8_t load_addr_l;
    MCP251x_CTRL_REGS ctrl_reg;

    uint8_t txb0[MCP251x_TXB_RXB_REG_SIZE];
    uint8_t txb1[MCP251x_TXB_RXB_REG_SIZE];
    uint8_t txb2[MCP251x_TXB_RXB_REG_SIZE];
    uint8_t rxb0[MCP251x_TXB_RXB_REG_SIZE];
    uint8_t rxb1[MCP251x_TXB_RXB_REG_SIZE];

    /* ctrl regs */
    uint8_t bfpctrl;
    uint8_t txrtsctrl;
    uint8_t canstat;
    uint8_t canctrl;
    uint8_t tec;
    uint8_t rec;
    uint8_t cnf3;
    uint8_t cnf2;
    uint8_t cnf1;
    uint8_t eflg;
    uint8_t txb0ctrl;
    uint8_t txb1ctrl;
    uint8_t txb2ctrl;
    uint8_t rxb0ctrl;
    uint8_t rxb1ctrl;

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

void mcp251x_emu_rx_buf_process(void);
uint8_t mcp251x_spi_isr_handler(void *dev, uint8_t spi_data);
void mcp251x_reset_state(void *dev);
void mcp251x_spi_emu_init(mcp251x_td *mcp251x, 
                          void (*can_txb0_cb)(void *priv), 
                          void (*can_txb1_cb)(void *priv),
                          void (*can_txb2_cb)(void *priv),
                          void (*set_irq_cb)(int high));
void mcp251x_emu_can_tx_irq_process(mcp251x_td *mcp251x);
void mcp251x_emu_set_irq_flag(mcp251x_td *mcp251x, MCP251x_IRQ_FLAGS irq_flag);
void mcp251x_emu_handle_txb_done(mcp251x_td *mcp251x, MCP251x_IRQ_FLAGS txb);
void mcp251x_emu_set_transmit_err_flag(mcp251x_td *mcp251x, MCP251x_CTRL_REGS txbnctrl, MCP251x_TXB_ERROR_FLAGS flag);
void mcp251x_emu_rx_data_rxb0(mcp251x_td *mcp251x);


/* Buffer Configuration */
#define RX_TRACE_BUF_ELEMENTS 256
#define RX_TRACE_BUF_SIZE    64

/* Receive Buffer Structure */
typedef struct buf_struct 
{
    int msg_len;
    uint8_t buf[RX_TRACE_BUF_SIZE];

} buf_td;

typedef struct trace_buffer_struct 
{
    unsigned int buf_write_location;
    unsigned int buf_read_location;
    buf_td trace_buf[RX_TRACE_BUF_ELEMENTS];
    unsigned int buf_count;
    task_queue_td buf_task_queue;

} trace_buffer_t;

void mcp251x_emu_rx_trace_buf_init(trace_buffer_t *rx_trace_buffer);
void mcp251x_emu_rx_trace_buf_add(trace_buffer_t *rx_trace_buffer);
void mcp251x_emu_rx_trace_buf_add_data(trace_buffer_t *rx_trace_buffer, uint8_t indata);
void mcp251x_emu_rx_trace_buf_process(trace_buffer_t *rx_trace_buffer);

#endif /* mcp251x_EMULATOR_H */
