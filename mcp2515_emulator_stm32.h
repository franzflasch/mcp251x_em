#ifndef MCP2515_EMULATOR_STM32_H
#define MCP2515_EMULATOR_STM32_H

#include <mcp2515_emulator.h>

void mcp2515_stm32_init(mcp251x_td *mcp251x);
void mcp2515_emu_can_tx_irq_process(mcp251x_td *mcp251x);

#endif /* MCP2515_EMULATOR_STM32_H */