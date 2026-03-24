# 🛡️ Master's Thesis: Fault-Tolerant Architecture in CAN Bus Networks with STM32 and FreeRTOS

![C](https://img.shields.io/badge/C-00599C?style=for-the-badge&logo=c&logoColor=white)
![FreeRTOS](https://img.shields.io/badge/FreeRTOS-20232A?style=for-the-badge&logo=freertos&logoColor=25A162)
![STM32](https://img.shields.io/badge/STM32-03234B?style=for-the-badge&logo=stmicroelectronics&logoColor=white)
![CAN Bus](https://img.shields.io/badge/CAN_Bus-Industrial-orange?style=for-the-badge)

This repository contains the source code for my Master's Thesis. The project implements a **robust and fault-tolerant distributed system** designed for critical environments (industrial/aerospace) using STM32 microcontrollers communicating via the industrial CAN Bus standard.

## 📋 Project Overview

The system consists of a network topology with **1 Master Node (HMI + Central Processor)** and **3 Slave Nodes (Sensors)**. It uses redundancy and data validation algorithms to detect, mitigate, and isolate anomalies in real-time, ensuring data integrity against electromagnetic interference or hardware failures.

The entire software architecture is built on **FreeRTOS**, ensuring deterministic behavior through the use of concurrent tasks, queues, and mailboxes.

## 🛠️ Hardware Used

* **Master Node:** 1x **STM32F429I-DISC1** development board (ARM Cortex-M4) with a QVGA resistive touch LCD.
* **Slave Nodes:** 3x **STM32F407G-DISC1** development boards.
* **Physical Layer:** CAN transceiver modules (e.g., TJA1050 / MCP2551) connecting all nodes to a common bus with 120Ω terminating resistors.

## ⚙️ Features & Fault Tolerance

The Fault Tolerance Engine implements the following safety policies:

1.  **Integrity Protection (CRC16):** Detection of corrupted frames. If a node sends 3 consecutive messages with an invalid CRC, the Master isolates the node from the network (*Muting*).
2.  **Sequence Control:** Detection of lost packets. If a sequence number skips, the system logs the failure and prepares an automatic network resynchronization.
3.  **Data Credibility:** Filtering of physically impossible sensor values (out of logical range), discarding the anomalous data without penalizing the slave's network status.
4.  **Triple Modular Redundancy (TMR) / Consensus:** Simultaneous reading from 3 independent nodes. The Master acts as a judge (voter) to isolate discrepant data and calculate the safe value.
5.  **Interactive Fault Injection:** The slaves feature a malicious routine. Pressing the **User Button (Blue Button)** on a slave intentionally inverts the CRC bits before sending the frame, simulating severe interference to evaluate the Master's response.

## 🖥️ Graphical User Interface (HMI)

The Master Node features an interactive control panel designed from scratch using the STMicroelectronics BSP library and analog touch reading (STMPE811).
* **Multi-screen Dashboard:** Smooth navigation with no flickering.
* **Real-time Monitoring:** Telemetry display, CRC errors, lost packets, and network status updated at 5 Hz.
* **Active Control Panel:** Allows the operator to manually connect/disconnect nodes from the network via touch buttons.

## 📂 Repository Structure

```text
├── common/                  # Shared libraries across all nodes
│   ├── inc/                 # Headers (can_protocol.h, fault_tolerance.h, crc16.h)
│   └── src/                 # Fault tolerance engine and CRC implementation
├── tfm_master_stm32f429i/   # Full project for the Master Node (HMI + Core)
│   ├── Core/                # Main code, FreeRTOS Tasks, and GUI
│   └── CMakeLists.txt       # Build configuration (CMake/GCC)
└── tfm_slave_stm32f407g/    # Project for the Slave Node (Sensors + Fault Injector)
    ├── Core/                # Sensor reading and CAN transmission tasks
    └── CMakeLists.txt
```

## 🚀 Installation & Build

This project uses **CMake** and the **Arm GNU Toolchain** for the build process. It is fully compatible with **Visual Studio Code**.

### Prerequisites
1. Install [Visual Studio Code](https://code.visualstudio.com/).
2. Install the following VS Code extensions:
   * C/C++ (Microsoft)
   * CMake Tools (Microsoft)
   * STM32Cube Core
3. Ensure you have the **Arm GNU Toolchain** (`arm-none-eabi-gcc`) and **CMake** installed and added to your system's PATH.

### Build Instructions
1. Clone this repository to your local machine:
   ```bash
   git clone https://github.com/ToniHG/TFM_STM_TF.git
   ```
2. Open the specific project folder you want to compile (e.g., `tfm_master_stm32f429i`) in VS Code.
3. Open the Command Palette (`Ctrl + Shift + P`) and run:
   * `CMake: Select a Kit` -> Choose your `arm-none-eabi-gcc` toolchain.
   * `CMake: Delete Cache and Reconfigure` (to ensure paths are correct).
4. Click the **Build** button in the VS Code status bar (bottom).
5. Once compiled successfully, flash the generated `.elf` file to your STM32 board using ST-Link Utility, STM32CubeProgrammer, or directly via OpenOCD/GDB in VS Code.

## 👨‍🎓 Author

* **Antonio Hermoso Garcia** - *Embedded Systems & Firmware Developer*
* **LinkedIn:** [[Antonio Hermoso](https://www.linkedin.com/in/antonio-hermoso-garc%C3%ADa-06122b177/)]
* **Advisor(s):** Javier Garcia Martin (Master's Thesis Tutor)
