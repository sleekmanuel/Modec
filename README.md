Modec

Overview

Modec is a project that involves the configuration and management of a Zigbee module for communication, low-power mode management, and handling various peripherals such as GPIO, USART, and Timers. It also includes robust error handling and LED indications for device states.

Features

Initialization: Configures system clock, peripherals (GPIO, USART, Timer), and Zigbee module.
Low Power Mode: Manages low-power mode by enabling/disabling STOP mode and controlling the Zigbee module sleep state.
Zigbee Communication: Configures Zigbee parameters, sends and receives data through UART, processes commands, and responds accordingly.
Interrupt Handlers: Handles UART reception and GPIO interrupts for detecting events such as motion (PIR sensor input).
Error Handling: Implements error handling with flashing LEDs and error logging into flash memory, followed by a system reset.
LED Indications: Uses LEDs to signal errors, low voltage, and operational states.
Timers: Utilizes a timer to track activity periods (e.g., 15 seconds of device activation).
Installation

Clone the repository:
git clone https://github.com/sleekmanuel/Modec.git
Open the project in your preferred IDE.
Usage

Initialize the system and peripherals by calling HAL_Init().
Configure the system clock using SystemClock_Config().
Initialize all configured peripherals:
C
MX_GPIO_Init();
MX_USART1_UART_Init();
MX_TIM2_Init();
Configure Zigbee module parameters and handle communication as per your requirements.
Contributing

Fork the repository.
Create a new branch:
git checkout -b feature-branch
Make your changes and commit them:
git commit -m "Add some feature"
Push to the branch:
git push origin feature-branch
Open a pull request.
License

This project is licensed under the terms that can be found in the LICENSE file in the root directory of this software component.

For more details, visit the Modec repository.
