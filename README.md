# STM32F407 GPIO Driver (Bare-Metal)

This repository contains a **bare-metal GPIO driver implementation**
for the **STM32F407** microcontroller, written in **C from scratch**
without using STM32 HAL GPIO drivers.

The project is developed using **STM32CubeIDE** and focuses on
**register-level programming**, GPIO peripheral control,
and interrupt handling using EXTI and NVIC.

---

## Project Objectives

- Understand GPIO operation at the **register level**
- Design a **reusable GPIO driver**
- Implement **interrupt-based GPIO handling**
- Learn EXTI and NVIC configuration
- Practice modular embedded driver architecture
- Use **Git & GitHub** for version-controlled firmware development

---

### Custom GPIO Driver (CORE WORK)

**All custom GPIO driver implementation is located here:**
<br>
Custom_Driver/
<br>
├── Inc/
<br>
│ ├── stm32f407xx.h
<br>
│ └── stm32f407xx_gpio_driver.h
<br>
│
<br>
└── Src/
<br>
└── stm32f407xx_gpio_driver.c
<br>

This driver includes:
- MCU register definitions and base address mapping
- GPIO configuration structures
- Peripheral clock enable/disable APIs
- GPIO initialization APIs
- GPIO read/write/toggle APIs
- GPIO interrupt configuration and handling support

This folder represents my **primary driver development work**.

---

## GPIO Driver Features Implemented

- GPIO register mapping using memory-mapped structures
- Peripheral clock control
- GPIO pin configuration:
  - Input / Output / Interrupt modes
  - Output speed
  - Pull-up / pull-down configuration
  - Output type
- GPIO input read (pin & port)
- GPIO output write and toggle
- EXTI interrupt configuration (falling edge)
- NVIC interrupt enable and priority configuration
- Interrupt callback handling

---

## Application Implemented: LED Control Using External Interrupt

An application has been implemented using the **custom GPIO driver**
to demonstrate **interrupt-driven GPIO control**.

---

### Application Description

- **LED:** Connected to **GPIOD Pin 12**
- **Button:** Connected to **GPIOD Pin 5**
- Button is configured in **falling-edge interrupt mode**
- LED toggles **inside the EXTI interrupt handler**
- EXTI line is mapped to **EXTI9_5 IRQ**
- A software delay is added for basic debouncing

---

### Application Flow

1. Configure LED pin as output
2. Configure button pin as GPIO interrupt (falling edge)
3. Enable EXTI line and NVIC interrupt
4. MCU waits in infinite loop
5. On button press:
   - EXTI interrupt triggers
   - ISR clears pending bit
   - LED toggles

---

### Application Code Location

The application logic is implemented in:

Core/Src/001LED_Blinking_application_testing.c
Interrupt handling is done using:
EXTI9_5_IRQHandler()
The application exclusively uses **custom GPIO driver APIs**.

---

## Key Learning Outcomes

- Configuring GPIO interrupts without HAL
- Understanding EXTI line mapping
- NVIC interrupt enable and priority handling
- Writing ISR-safe GPIO logic
- Using handle-based driver APIs in real applications
- Clear separation of driver and application layers

---

## Tools & Environment

- **MCU:** STM32F407xx
- **IDE:** STM32CubeIDE
- **Language:** C
- **Programming Style:** Bare-metal / Register-level
- **Interrupt Handling:** EXTI + NVIC
- **Version Control:** Git & GitHub

---

## Author

**Tanishq Sagar**

This repository is part of my embedded systems learning journey,
focused on **low-level firmware development**, **driver design**,
and understanding STM32 microcontroller internals without
using abstraction layers.
