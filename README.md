# MCP251x Emulator
This repository contains a software-based emulator for the MCP251x family of CAN controllers (e.g., MCP2515). It provides a simulated SPI interface, register map, interrupt handling, and basic transmit/receive buffer functionality, allowing you to test or prototype CAN-related logic without requiring actual MCP251x hardware.

## Features
- **SPI Command Emulation:** Handles common SPI operations (READ, WRITE, BIT MODIFY, RESET, READ RX BUFFER, LOAD TX BUFFER, etc.).
- **Register Simulation:** Maintains internal register states (e.g., CANSTAT, CANCTRL, TXB0CTRL, CANINTE, CANINTF, etc.).
- **Interrupt Simulation:** Supports setting and clearing interrupt flags, mimicking real hardware behavior.
- **TX and RX Buffers:** Emulates transmit and receive buffers with callbacks to simulate data flow.
- **RX Trace Buffer:** Provides a simple ring buffer mechanism to trace and process received data.

## Usage
Include the Emulator in Your Project

Add the header and source files to your project:

```
mcp251x_emulator.h
mcp251x_emulator.c
```

### Reference Implementation for STM32F3

For guidance on integrating the emulator with an STM32F3 microcontroller, refer to:

```
mcp251x_emulator_stm32.c
```

Dependencies: https://github.com/franzflasch/rem_packages/tree/master/OS/task_queue

Currently events are queued in the ISR routine. So you need to pull and process those events in your main context.  
Here is an example of how to initialize and handle the events in your main function:

```
mcp251x_td mcp251x = {0};

int main(void)
{
    clock_setup();
    led_setup();
    gpio_setup();

    mcp251x_stm32_init(&mcp251x);

    while (1)
    {
        led_toggle();

        /* process mcp251x events */
        mcp251x_emu_can_tx_irq_process(&mcp251x);
    }

    return 0;
}
```


## Work in Progress (WIP)
This project is currently a Work in Progress. Not all features are fully implemented yet. However, it has been successfully used in the following setup:

- Raspberry Pi: Running the mainline MCP251x driver.
- STM32F3: Running the MCP251x emulator.

```
[   77.987297] mcp251x spi3.0 can0: MCP2515 successfully initialized.
```

```
4: can0: <NOARP,UP,LOWER_UP,ECHO> mtu 16 qdisc pfifo_fast state UP group default qlen 10
    link/can
```

## Current Limitations
Configuration Registers: To use the emulator with real hardware, you need to implement functions for the configuration registers (e.g., MCP251x_REG_CNFx).
Simulated Environment: Currently, the emulator operates in a simulated environment by ignoring configuration registers. Receiving and transmitting are simulated internally. Extending support for real CAN hardware should be straightforward from this stage.

Note: The STM32F3 can handle a maximum SPI frequency of approximately 500kHz.

## Why?
Imagine you have a Linux-based system connected to an MCU (with CAN capabilities) via SPI. This emulator allows you to use the mainline MCP251x driver from the Linux kernel (source) to provide the CAN interface from the MCU to Linux without needing actual MCP251x hardware.

## Contributing
Contributions are welcome! If you encounter issues or have suggestions for improvements, please open an issue or submit a pull request.
