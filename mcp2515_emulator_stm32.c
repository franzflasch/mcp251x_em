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

// /* Buffer Configuration */
#define RX_TRACE_BUF_ELEMENTS 256
#define RX_TRACE_BUF_SIZE    64

extern rx_buffer_t rx_buffer;
static fQ_t *rx_buf_queue;
static mcp251x_td *mcp251x_ref = NULL;

/* Initialize Receive Buffer Queue */
void mcp2515_emu_rx_buf_init(void)
{
    rx_buf_queue = fQ_create(RX_TRACE_BUF_ELEMENTS);
    if (rx_buf_queue == NULL) 
    {
        /* Handle queue creation failure */
        printf("Error: Failed to create receive buffer queue.\r\n");
        /* Implement fallback or reset mechanisms as needed */
    }
}

/* Deinitialize Receive Buffer Queue */
void mcp2515_emu_rx_buf_deinit(void)
{
    if (rx_buf_queue != NULL) 
    {
        fQ_destroy(rx_buf_queue);
        rx_buf_queue = NULL;
    }
}

/* Process Receive Buffer Queue */
void mcp2515_emu_rx_buf_process(void)
{
    if (rx_buf_queue != NULL) 
    {
        fQ_pull(rx_buf_queue);
    }
}

void can_tx_send_done(void *priv)
{
    (void) priv;
    int i = 0;
    printf("CAN TX done!\r\n");

    for(i=0;i<MCP251x_TXB_RXB_REG_SIZE;i++)
    {
        printf("%x\r\n", mcp251x_ref->txb0[i]);
    }

    /* Fake txf interrupt here */
    /* trigger interrupt */
    mcp251x_ref->tx0if = 1;
    gpio_clear(GPIOB, GPIO10);
}

void mcp2515_emu_can_tx_irq_process(mcp251x_td *mcp251x)
{
    task_queue_pull(&mcp251x->can_tx_irq_queue);
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

        if (rx_buf_queue != NULL) 
        {
            fQ_push(rx_buf_queue, rx_buf_trace_queue_cb);
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

    mcp251x_spi_emu_init(mcp251x, can_tx_send_done);
}
