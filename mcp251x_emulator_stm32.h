#ifndef mcp251x_EMULATOR_STM32_H
#define mcp251x_EMULATOR_STM32_H

#include <mcp251x_emulator.h>

void mcp251x_stm32_init(mcp251x_td *mcp251x);
void mcp251x_emu_can_tx_irq_process(mcp251x_td *mcp251x);
trace_buffer_t *mcp251x_emu_stm32_trace_buf_init(void);

#endif /* mcp251x_EMULATOR_STM32_H */