# STM32F407 GPIO Driver (Bare-Metal)

This repository contains a **bare-metal GPIO driver implementation**
for the **STM32F407** microcontroller, written in **C from scratch**
without using STM32 HAL GPIO drivers.

The project is developed using **STM32CubeIDE** and follows a
**register-level programming approach** to understand MCU internals,
peripheral registers, and driver design.

---

## Project Objectives

- Understand GPIO at the **register level**
- Design a **reusable GPIO driver**
- Practice **modular embedded driver architecture**
- Learn how STM32CubeIDE projects are structured
- Use **Git & GitHub** for version-controlled firmware development
- Build and test applications using custom drivers

---

### Custom GPIO Driver (CORE WORK)

**All custom GPIO driver implementation is located here:**
<br>
driver/
<br>
├── Inc/
<br>
│ ├── stm32f407xx.h
<br>
│ └── stm32f407xx_gpio_driver.h
<br>
│
└── Src/
<br>
└── stm32f407xx_gpio_driver.c
<br>

This driver includes:
- MCU-specific register definitions
- Peripheral base address mapping
- GPIO configuration structures
- Clock enable/disable macros
- GPIO initialization APIs
- GPIO read/write/toggle APIs

If you want to review my **actual driver implementation**, start here.

---

## GPIO Driver Features Implemented

- GPIO register definitions using memory-mapped structures
- Peripheral clock control APIs
- GPIO pin configuration:
  - Mode (input, output, alternate, analog)
  - Output speed
  - Pull-up / pull-down
  - Output type (push-pull / open-drain)
- GPIO input read (pin & port)
- GPIO output write and toggle
- Handle-based driver design for scalability

---

## Application Implemented: LED Blinking with Button (Polling)

An application has been implemented using the **custom GPIO driver**
to demonstrate real hardware usage.

### Application Description

- **LED:** Connected to **GPIOD Pin 12**
- **Button:** Connected to **GPIOA Pin 0**
- LED toggles **only when the button is pressed**
- Button state is read using **polling**
- A simple software delay is used for debouncing

### Key Learning Outcomes

- Using custom GPIO driver APIs in `001LED_Blinking_application_testing.c`
- Configuring GPIO input and output pins
- Reading input pin state
- Controlling hardware without HAL
- Understanding polling-based button handling

### Application Code Location

The application logic is implemented in:

Core/Src/001LED_Blinking_application_testing.c

It directly uses the GPIO driver APIs defined in the `driver/` folder.

---

## Tools & Environment

- **MCU:** STM32F407xx
- **IDE:** STM32CubeIDE
- **Language:** C
- **Programming Style:** Bare-metal / Register-level
- **Version Control:** Git & GitHub

---

## Work in Progress

The following features will be added incrementally:

- GPIO interrupt configuration (EXTI)
- NVIC interrupt handling
- Button interrupt–based LED control
- Separation of application logic into `Applications/` folder
- Additional peripheral drivers

Each feature will be added through **separate commits** to clearly
show the development and learning progression.

---

## Author

**Tanishq Sagar**

This repository is part of my embedded systems learning journey,
focused on **low-level firmware development**, **driver design**,
and understanding microcontroller internals without abstraction layers.
