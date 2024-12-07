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

#include <mcp251x_emulator_stm32.h>

static trace_buffer_t rx_trace_buffer = {0};
static mcp251x_td *mcp251x_ref = NULL;

void can_tx_cb(void *priv)
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
    mcp251x_ref->set_irq_cb(0);
}

void mcp251x_emu_set_irq_cb(int high)
{
    high ? gpio_set(GPIOB, GPIO10) : gpio_clear(GPIOB, GPIO10);
}

/* SPI1 Interrupt Service Routine */
void spi1_isr(void)
{
    uint8_t indata = spi_read8(SPI1);
    uint8_t outdata = 0;

    gpio_set(GPIOB, GPIO11);
    outdata = mcp251x_spi_isr_handler(mcp251x_ref, indata);
    spi_send8(SPI1, outdata);
    gpio_clear(GPIOB, GPIO11);

    mcp251x_emu_rx_trace_buf_add_data(&rx_trace_buffer, indata);
}

/* EXTI4 Interrupt Service Routine */
void exti4_isr(void)
{
    if (exti_get_flag_status(EXTI4)) 
    {
        /* reset mcp state machine */
        mcp251x_reset_state(mcp251x_ref);

        mcp251x_emu_rx_trace_buf_add(&rx_trace_buffer);

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

/* Initialize SPI1 for mcp251x Emulator */
void mcp251x_stm32_init(mcp251x_td *mcp251x)
{
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

    /* Setup EXTI for mcp251x Emulator */
    extipa4_setup();

    mcp251x_ref = mcp251x;

    mcp251x_spi_emu_init(mcp251x, can_tx_cb, mcp251x_emu_set_irq_cb);
}

trace_buffer_t *mcp251x_emu_stm32_trace_buf_init(void)
{
    /* Initialize Receive Buffer */
    mcp251x_emu_rx_trace_buf_init(&rx_trace_buffer);

    return &rx_trace_buffer;
}
