# STM32 Zonal Architecture - HIL Digital Twin (Phase 1)

**Focus:** Sensor Fusion, Battery Modeling, and Braking Arbitration for Zonal EV Architectures.

## Project Overview
This repository contains the implementation of a **Hardware-in-the-Loop (HIL) Digital Twin** representing an Electric Vehicle's Zonal Controller (Node B). The system runs a real-time physics engine, battery plant model, and control logic on an **STM32F446RE**, communicating with a Stimulus Node (Node A) via **CAN Bus**.

## Key Features
* **Physics-Based Digital Twin:**
    * Simulates Vehicle Dynamics (Drag, Rolling Resistance, Inertia).
    * **Thevenin Battery Model:**  V_term = V_OCV − I · R_dynamic
    * **Bathtub Curve Resistance:**  R_int = f(SoC)
    * **Thermal Efficiency:** Models 95% powertrain efficiency for realistic thermal losses.  

* **Sensor Fusion (EKF):**
    * Extended Kalman Filter (EKF) for State of Charge (SoC) estimation.
    * **Drift Rejection:** Fuses Current Integration (high-frequency) with Voltage Feedback (low-drift).
    * **Dynamic Gating:** 1000A Correction Gate enabled by dynamic resistance modeling, robust against high-current braking transients.
* **Control Logic:**
    * **Blended Braking Arbitration:** Seamlessly transitions between Regenerative Braking (Motor) and Friction Braking (Hydraulic) to maintain 1.0g deceleration.
    * **One-Pedal Driving:** Automatic regen engagement during coasting.

## Repository Structure
* **`Firmware/`**: STM32CubeIDE projects for the Zonal Controller (Node B) and Stimulus Generator (Node A).
* **`Docs/`**: Detailed Technical Report (Phase 1).
* **`Results/`**: Validated CSV logs and plot visuals from HIL testing.

## Build & Run Instructions

### 1. Hardware Requirements
* **Microcontrollers:** 2x STM32F446RE (Nucleo-64 Boards)
* **Transceivers:** 2x SN65HVD230  (CAN Transceivers)
* **Wiring:** Jumper wires (Female-Female / Male-Female)

### 2. Wiring Diagram (HIL Setup)

**CAN Bus Connection (Between Node A & Node B):**
| Node A (Stimulus) | CAN Transceiver A | <--> | CAN Transceiver B | Node B (Controller) |
| :--- | :--- | :--- | :--- | :--- |
| PA12 (CAN_TX) | TXD | | TXD | PA12 (CAN_TX) |
| PA11 (CAN_RX) | RXD | | RXD | PA11 (CAN_RX) |
| GND | GND | | GND | GND |
| | CAN H | <--> | CAN H | |
| | CAN L | <--> | CAN L | |

**Telemetry Connection (Node B to PC):**
| Node B (Controller) | PC Connection |
| :--- | :--- |
| PA2 (USART2_TX) | USB Cable (Virtual COM Port) |
| GND | USB GND |

### 3. Software Setup
1.  **IDE:** Install **STM32CubeIDE**.
2.  **Clone Repo:** `git clone https://github.com/LakshyaTaneja/STM32-Zonal-Architecture-Digital-Twin.git`
3.  **Flash Node A:**
    * Open `Firmware/Node_A_Stimulus` in STM32CubeIDE.
    * Build & Debug (Flash) to the first board.
4.  **Flash Node B (Digital Twin):**
    * Open `Firmware/Node_B_Controller` in STM32CubeIDE.
    * Build & Debug (Flash) to the second board.

### 4. Visualization (SerialPlot)
To visualize the Digital Twin data in real-time:
1.  Open **SerialPlot** (or similar tool).
2.  Connect to Node B's COM Port.
3.  **Baud Rate:** 115200
4.  **Data Format:** ASCII (CSV)
5.  **Channels:** 7 Columns
    * Col 1: Time
    * Col 2: Speed
    * Col 3: True SoC
    * Col 4: EKF SoC
    * Col 5: Bad SoC (Coulomb Counter)
    * Col 6: Prop Torque
    * Col 7: Regen Torque
    * Col 8: Friction Torque

## Results Summary
* **Coasting Test:** Validated EKF drift rejection; Estimator tracks True SoC while Coulomb Counter diverges due to sensor bias.
* **Emergency Braking:** Validated Arbitration Logic; System caps Regen at -2500Nm and blends Friction torque to achieve target deceleration, maintaining EKF stability during 690A current spikes.  

## Future Scope: Phase II (Migration to Hybrid Architectures)

As per the M.Tech Project roadmap, the **Zonal Architecture** developed in Phase I will serve as the foundational **Digital Twin** for a **Hybrid Powertrain Control Unit**.

### Core Objectives

Phase II will focus on the following key areas:

1. **Platform Migration**
* Porting the validated "Torque Arbitration" logic to a heterogeneous SoC (NXP i.MX8 or similar) running a Type-1 Hypervisor.


2. **Mixed-Criticality Integration**
* **RTOS Domain (Safety):** Will host the safety-critical **Energy Management Strategy (EMS)** for splitting torque between the Internal Combustion Engine (ICE) and the Electric Motor.
* **Linux Domain (Performance):** Will host the "Fuel Efficiency & Diagnostics" dashboard.


3. **Why Hybrid?**
* The noise, vibration, and scheduling complexity of a Hybrid Powertrain necessitates the strict **Freedom From Interference (FFI)** provided by this Virtualized Zonal Architecture.



### Technology Roadmap Comparison

| System Element | Phase I Implementation | Phase II Extension (Target) |
| --- | --- | --- |
| **Compute Platform** | STM32F446RE (Cortex-M4) | NXP i.MX8 / RPi4 (Cortex-A + M) |
| **OS Architecture** | Single-Core Bare Metal | Dual-OS (Linux + FreeRTOS) |
| **Control Focus** | EV Blended Braking | Hybrid Torque Split (EMS) |
| **Isolation** | Physical (Separate PCBs) | Virtual (Type-1 Hypervisor) |
| **Safety Goal** | 100Hz Control Loop | Freedom From Interference (FFI) |

*Table: Roadmap - Migration from Phase I Networked Control to Phase II Hybrid Zonal Architecture.*