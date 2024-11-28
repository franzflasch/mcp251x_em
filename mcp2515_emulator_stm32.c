/* Include Headers */
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/sync.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/spi.h>

#include <usart_dma.h>
#include <antirtos_c_wrapper.h>

#include <mcp2515_emulator_stm32.h>

/* Buffer Configuration */
#define RX_TRACE_BUF_ELEMENTS 256
#define RX_TRACE_BUF_SIZE    64

mcp251x_td *mcp251x_ref = NULL;

/* Receive Buffer Structure */
typedef struct rx_buf_struct 
{
    int msg_len;
    uint8_t trace_buf[RX_TRACE_BUF_SIZE];
} rx_buf_td;

typedef struct rx_buffer_struct 
{
    unsigned int rx_buf_write_location;
    unsigned int rx_buf_read_location;
    rx_buf_td rx_trace_buf[RX_TRACE_BUF_ELEMENTS];
    unsigned int rx_buf_count;
    fQ_t *rx_buf_queue;
} rx_buffer_t;

/* Global Instance */
static volatile rx_buffer_t rx_buffer = 
{
    .rx_buf_write_location = 0,
    .rx_buf_read_location  = 0,
    .rx_trace_buf         = {{0}},
    .rx_buf_count         = 0,
    .rx_buf_queue         = NULL
};

#define DECODE_BIT(val, bitmask, name) (((val) & (bitmask)) ? (name) : "x")

/* Callback for Processing Received Data */
void rx_buf_queue_cb(void) 
{
    int msg_len = rx_buffer.rx_trace_buf[rx_buffer.rx_buf_read_location].msg_len;
    int curr_reg = 0;
    int curr_reg_val = 0;

    /* Validate msg_len to prevent buffer overread */
    if (msg_len > RX_TRACE_BUF_SIZE) 
    {
        printf("Error: Message length %d exceeds buffer size.\r\n", msg_len);
        msg_len = RX_TRACE_BUF_SIZE; /* Adjust as needed */
    }

    printf("WR: %u RD: %u ", rx_buffer.rx_buf_write_location, rx_buffer.rx_buf_read_location);
    for(int i = 0; i < msg_len; i++) 
    {
        printf("%02x ", rx_buffer.rx_trace_buf[rx_buffer.rx_buf_read_location].trace_buf[i]);
        /* mark this buffer as read by setting the msg_len to 0 */
        rx_buffer.rx_trace_buf[rx_buffer.rx_buf_read_location].msg_len = 0;
    }

    for(int i = 0; i < msg_len; i++) 
    {
        /* CMD */
        if(i==0)
        {
            switch(rx_buffer.rx_trace_buf[rx_buffer.rx_buf_read_location].trace_buf[0])
            {
                case MCP251x_RESET_CMD:
                    printf("RST ");
                break;
                case MCP251x_READ_CMD:
                    printf("RD ");
                break;
                case MCP251x_WRITE_CMD:
                    printf("WR ");
                break;
                default:
                break;
            }

            if( (rx_buffer.rx_trace_buf[rx_buffer.rx_buf_read_location].trace_buf[0] & MCP251x_LOAD_TX_BUFFER_MASK) == MCP251x_LOAD_TX_BUFFER)
                printf("LOAD_TX ");
        }
        /* ADDRESS */
        else if(i==1)
        {
            curr_reg = rx_buffer.rx_trace_buf[rx_buffer.rx_buf_read_location].trace_buf[1];
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
            curr_reg_val = rx_buffer.rx_trace_buf[rx_buffer.rx_buf_read_location].trace_buf[2];
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

    rx_buffer.rx_buf_read_location = (rx_buffer.rx_buf_read_location + 1) % RX_TRACE_BUF_ELEMENTS;
}

/* Initialize Receive Buffer Queue */
void mcp2515_emu_rx_buf_init(void)
{
    rx_buffer.rx_buf_queue = fQ_create(RX_TRACE_BUF_ELEMENTS);
    if (rx_buffer.rx_buf_queue == NULL) 
    {
        /* Handle queue creation failure */
        printf("Error: Failed to create receive buffer queue.\r\n");
        /* Implement fallback or reset mechanisms as needed */
    }
}

/* Deinitialize Receive Buffer Queue */
void mcp2515_emu_rx_buf_deinit(void)
{
    if (rx_buffer.rx_buf_queue != NULL) 
    {
        fQ_destroy(rx_buffer.rx_buf_queue);
        rx_buffer.rx_buf_queue = NULL;
    }
}

/* Process Receive Buffer Queue */
void mcp2515_emu_rx_buf_process(void)
{
    if (rx_buffer.rx_buf_queue != NULL) 
    {
        fQ_pull(rx_buffer.rx_buf_queue);
    }
}

/* SPI1 Interrupt Service Routine */
void spi1_isr(void)
{
    uint8_t indata = spi_read8(SPI1);
    uint8_t outdata = 0;

    outdata = mcp251x_spi_isr_handler(mcp251x_ref, indata);

    spi_send8(SPI1, outdata);

    if (rx_buffer.rx_buf_count < RX_TRACE_BUF_SIZE) 
    {
        rx_buffer.rx_trace_buf[rx_buffer.rx_buf_write_location].trace_buf[rx_buffer.rx_buf_count++] = indata;
        rx_buffer.rx_trace_buf[rx_buffer.rx_buf_write_location].msg_len = rx_buffer.rx_buf_count;
    }
    else 
    {
        /* Handle buffer overflow if necessary */
        printf("Warning: RX buffer overflow. Data byte 0x%02x discarded.\r\n", indata);
        /* Optionally, reset rx_buf_count or implement other overflow handling */
    }

    /* Optionally, handle SPI transmission here */
}

/* EXTI4 Interrupt Service Routine */
void exti4_isr(void)
{
    if (exti_get_flag_status(EXTI4)) 
    {
        /* reset mcp state machine */
        mcp251x_reset_state(mcp251x_ref);

        rx_buffer.rx_buf_write_location = (rx_buffer.rx_buf_write_location + 1) % RX_TRACE_BUF_ELEMENTS;
        rx_buffer.rx_buf_count = 0;

        /* check if we are producing faster than the data can be read */
        if(rx_buffer.rx_trace_buf[rx_buffer.rx_buf_write_location].msg_len)
            printf("Warning: Too much data to process!\r\n");

        if (rx_buffer.rx_buf_queue != NULL) 
        {
            fQ_push(rx_buffer.rx_buf_queue, rx_buf_queue_cb);
        } 
        else 
        {
            printf("Error: Receive buffer queue is not initialized.\r\n");
        }

        exti_reset_request(EXTI4);
    }
}


/* Setup for EXTI4 (GPIOA pin 4) */
static void extipa4_setup(void)
{
    /* Enable GPIOA and SYSCFG clocks */
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_SYSCFG);

    exti_reset_request(EXTI4);

    /* Enable EXTI4 interrupt */
    nvic_enable_irq(NVIC_EXTI4_IRQ);

    /* Configure GPIOA pin 4 as input with pull-down */
    /* commented out, because this pin is used as NSS (slave select) pin primarily
     * additionally used as EXTI
     */
    // gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_PULLDOWN, GPIO4);

    /* Configure EXTI4 */
    exti_select_source(EXTI4, GPIOA);
    exti_set_trigger(EXTI4, EXTI_TRIGGER_RISING);
    exti_enable_request(EXTI4);
}

/* Initialize SPI1 for MCP2515 Emulator */
void mcp2515_stm32_init(mcp251x_td *mcp251x)
{
    /* Initialize Receive Buffer */
    mcp2515_emu_rx_buf_init();

    /* Enable SPI1, GPIOA, and GPIOB clocks */
    rcc_periph_clock_enable(RCC_SPI1);
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);

    /* Setup GPIO pins for SPI1 (AF5) */
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO5 | GPIO6);
    gpio_set_af(GPIOA, GPIO_AF5, GPIO5 | GPIO6);
    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO5);
    gpio_set_af(GPIOB, GPIO_AF5, GPIO5);

    /* Configure NSS pin with pull-up */
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO4);
    gpio_set_af(GPIOA, GPIO_AF5, GPIO4);

    /* Configure SPI1 in slave mode */
    spi_set_slave_mode(SPI1);
    spi_disable_software_slave_management(SPI1);
    spi_disable_ss_output(SPI1);
    spi_set_full_duplex_mode(SPI1);
    spi_set_data_size(SPI1, SPI_CR2_DS_8BIT);
    spi_send_msb_first(SPI1);
    spi_fifo_reception_threshold_8bit(SPI1);

    /* Enable SPI1 RX buffer not empty interrupt */
    spi_enable_rx_buffer_not_empty_interrupt(SPI1);

    /* Enable SPI1 interrupt in NVIC */
    nvic_enable_irq(NVIC_SPI1_IRQ);

    /* Enable SPI1 peripheral */
    spi_enable(SPI1);

    /* Setup EXTI for MCP2515 Emulator */
    extipa4_setup();

    mcp251x_ref = mcp251x;

    mcp251x_spi_emu_init(mcp251x);
}
