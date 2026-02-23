# STM32F4 Bare-Metal Peripheral Driver Development

This repository contains a **bare-metal GPIO driver implementation**
for the **STM32F407** microcontroller, written in **C from scratch**
without using STM32 HAL GPIO drivers.

The project is developed using **STM32CubeIDE** and focuses on
**register-level programming**, GPIO, SPI, I2C and USART peripheral control
and interrupt handling using EXTI and NVIC.
A complete bare-metal driver development framework for STM32F407 microcontrollers (Cortex-M4), written from scratch in C without using ST’s HAL or LL libraries. This project demonstrates how to implement low-level peripheral drivers, learn register-level programming, and build real embedded applications.

---

## Table of Contents

Project Overview

Repository Structure

Driver Modules

Example Applications

How It Works

Getting Started

Building & Flashing

License

---

## Project Overview

This repository implements bare-metal peripheral drivers for the STM32F407 microcontroller family. It contains both:

Driver source code (.c) and headers (.h)

Example applications demonstrating usage (blinking LEDs, SPI, I2C, USART, etc.)

All drivers interact directly with MCU registers and hardware — no HAL or middleware dependencies. This project is ideal for:

Learning low-level embedded systems programming

Studying ARM Cortex-M architecture peripherals

Building minimal firmware with full control over registers

Customizing drivers for specific hardware requirements

---

## Repository Structure
Core
<br>
├──  Src/
<br>
│   ├── 001_LED_Blinking_Application_…       — LED blinking example
<br>
│   ├── 002LED_Blinking_application_t…       — Variant LED test
<br>
│   ├── 003_SPI_tx_testing_wo_slave.c         — SPI transmit (no slave)
<br>
│   ├── 004_SPI_tx_testing_w_slave_AR…        — SPI transmit w/ slave
<br>
│   ├── 005_SPI_TxRx_Arduino.c                — SPI master ↔ Arduino example
<br>
│   ├── 007_I2C_Tx_w_slave_Arduino.c          — I2C master transmit to Arduino
<br>
│   ├── 008_I2C_RxTx_w_slave_arduino.c        — I2C bidirectional master/slave
<br>
│   ├── 009_USART_Tx_w_Arduino.c              — UART transmit to Arduino
<br>
│   └── main.c                                — Master application entry
<br>

Custom Drivers
<br>
├── Inc/
<br>
│   ├── stm32f407xx.h                         — MCU core & register definitions
<br>
│   ├── stm32f407xx_gpio_driver.h             — GPIO driver API
<br>
│   ├── stm32f407xx_rcc_driver.h              — RCC clock control API
<br>
│   ├── stm32f407xx_spi_driver.h              — SPI driver API
<br>
│   ├── stm32f407xx_i2c_driver.h              — I2C driver API
<br>
│   └── stm32f407xx_usart_driver.h            — USART/UART API
<br>
│
<br>
├── Src/
<br>
│   ├── stm32f407xx_gpio_driver.c             — GPIO driver implementation
<br>
│   ├── stm32f407xx_rcc_driver.c              — RCC clock configuration logic
<br>
│   ├── stm32f407xx_spi_driver.c              — SPI master/slave registers
<br>
│   ├── stm32f407xx_i2c_driver.c              — I2C master/slave support
<br>
│   └── stm32f407xx_usart_driver.c            — UART/USART driver code
<br>
└── …
<br>

---

## Driver Modules

**RCC Driver**

Manages system and peripheral clocks.
Allows enabling clocks for GPIO, SPI, USART, and I2C modules.
This ensures peripheral registers are powered before use.

**GPIO Driver**

Configures GPIO pins as input, output, alternate function, or analog.
Supports setting pin speed, type, pull-up/pull-down, and alternate functions — crucial for SPI, I2C, and UART use.

**SPI Driver**

Supports SPI controller setup, master/slave mode, clock speed selection, and transmit/receive operations.
Used to communicate with SPI peripherals or another SPI device.

**I2C Driver**

Bare-metal I2C master and slave communication support.
Supports blocking mode send/receive and example communication with Arduino.

**USART Driver**

Configures UART peripheral for asynchronous serial communication at configurable baud rates.
Used for sending debug messages or data exchange with serial terminals.

---

## Example Applications

Each example in Src/ shows how to use drivers:

**1. GPIO Testing**

 001 / 002 – LED Blinking without and with interrupts.

Toggle GPIO pins connected to LEDs in a loop — classic bare-metal starting point.

**2. SPI Communication Tests**

Transmit only

Master ↔ Arduino

With/without slave

These demonstrate configuring SPI registers and sending data without HAL.

**3. I2C Communication Examples**

Master send and receive transactions with Arduino or another I2C device.

**4. USART UART Transmit Example**

Demonstrates serial data transmission to external hosts (PC terminal or microcontroller).

---

## How It Works

Bare-metal drivers access hardware through register definitions in stm32f407xx.h. Example:

GPIOA->MODER |= (1 << (2 * pin_number));

This sets the GPIO direction without any HAL abstraction.

All peripheral setup functions follow the steps:

Enable peripheral clock via RCC

Configure GPIO pins

Set peripheral configuration registers

Transmit/Receive or run application logic

By avoiding HAL, you get full control, minimal overhead, and deep hardware understanding.

---

## Getting Started
Prerequisites

ARM GCC toolchain (e.g., arm-none-eabi-gcc)

ST-Link programmer/debugger

STM32F4 Discovery / NUCLEO board

Build & Flash

Clone the repository

git clone https://github.com/tanishqsagartsg/stm32f4-baremetal-drivers.git

Open in your IDE (STM32CubeIDE, VSCode + Makefile, or other)

Build the project

Flash to MCU using ST-Link or similar

You should see LEDs blink or terminal UART output depending on the example.

---

## Why Bare-Metal?

Writing drivers from scratch builds:

Deep MCU architectural understanding

Register-level debugging skills

Customizable, lightweight firmware

Independence from vendor HAL abstractions

---

## License

This project is open-source and free to use for learning and development.

---

## Author

**Tanishq Sagar**

This repository is part of my embedded systems learning journey,
focused on **low-level firmware development**, **driver design**,
and understanding STM32 microcontroller internals without
using abstraction layers.
