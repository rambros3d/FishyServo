# N20 Servo Hardware Specification & Pinout

Custom high-performance micro-servo controller board powered by the **STM32H503KBU6** (Arm Cortex-M33) with integrated dual **DRV8411ARTE** H-bridge motor drivers and dual I2C buses.

---

## 1. Core Architecture

- **Microcontroller**: STM32H503KBU6
  - **Core**: Arm 32-bit Cortex-M33 with TrustZone, FPU, DSP
  - **Clock**: Up to 250 MHz (Internal HSI 64 MHz, CSI 4 MHz, HSI48; no external crystal required)
  - **Memory**: 128 KB Flash, 32 KB SRAM
  - **Package**: UFQFPN-32 (5 × 5 mm, 0.5 mm pitch) with exposed ground pad (EP)
- **Motor Drivers**: Texas Instruments **DRV8411ARTE** (Dual H-Bridge Motor Drivers)
  - Driving 4 independent half/full H-bridge channels: **Channel A, B, C, D**
  - Integrated current sense outputs (`ISENS_A` through `ISENS_D`)
- **Dual I2C Architecture**:
  - **Host Communication Bus**: `PB3` (SCL) / `PB4` (SDA) on `I2C2` (used by BareBoot Katapult bootloader at `0x40`)
  - **Sensor Bus**: `PB6` (3D_SCL) / `PB7` (3D_SDA) on `I2C1` (dedicated to 3D magnetic position sensor, e.g. TMAG5273)

---

## 2. Complete Pinout Table

| Pin # | Pin Name | Net Label | Mode / Alternate Function | Subsystem | Description |
|:---:|:---|:---|:---|:---|:---|
| **1** | `VDD` | `3V3` | Power Supply | Power | Digital 3.3V supply (Decoupled by C2 4.7µF, C9 100nF) |
| **2** | `PC14-OSC32_IN` | *NC* | None (X) | Unused | No 32.768 kHz crystal fitted (Unconnected) |
| **3** | `PC15-OSC32_OUT`| *NC* | None (X) | Unused | No 32.768 kHz crystal fitted (Unconnected) |
| **4** | `NRST` | `RST` | Reset | Control | Active-low system reset (10 kΩ pull-up R31 to 3V3) |
| **5** | `VDDA/VREF+` | `3V3` | Analog Power | Power | Analog supply & reference (Decoupled by C37 4.7µF, C20 100nF) |
| **6** | `PA0` | `LED_B` | GPIO Output | Indicator | Blue Status LED (Active LOW, common anode to 3V3 via 1 kΩ) |
| **7** | `PA1` | `LED_R` | GPIO Output | Indicator | Red Status LED (Active LOW, common anode to 3V3 via 1 kΩ) |
| **8** | `PA2` | `ISENS_A` | `ADC1_INP14` | Current Sense | Driver Channel A Current Sense feedback |
| **9** | `PA3` | `ISENS_B` | `ADC1_INP15` | Current Sense | Driver Channel B Current Sense feedback |
| **10**| `PA4` | `ISENS_C` | `ADC1_INP18` | Current Sense | Driver Channel C Current Sense feedback |
| **11**| `PA5` | `ISENS_D` | `ADC1_INP19` | Current Sense | Driver Channel D Current Sense feedback |
| **12**| `PA6` | `IN_C1` | `TIM3_CH1` / GPIO | Motor Driver | Channel C PWM / Direction Input 1 |
| **13**| `PA7` | `IN_C2` | `TIM3_CH2` / GPIO | Motor Driver | Channel C PWM / Direction Input 2 |
| **14**| `PB0` | `IN_D2` | `TIM3_CH3` / GPIO | Motor Driver | Channel D PWM / Direction Input 2 |
| **15**| `PB1` | `VSENS` | `ADC1_INP1` | Voltage Sense | Motor / Bus Supply Voltage sense divider |
| **16**| `VCAP` | `VCAP` | Internal Regulator | Power | LDO core supply bypass (C6 4.7µF to GND) |
| **17**| `VDD` | `3V3` | Power Supply | Power | Digital 3.3V supply (Decoupled by C1 4.7µF, C7 100nF) |
| **18**| `PB15`| `IN_B2` | `TIM1_CH3N` / GPIO | Motor Driver | Channel B PWM / Direction Input 2 |
| **19**| `PA8` | `IN_B1` | `TIM1_CH1` / GPIO | Motor Driver | Channel B PWM / Direction Input 1 |
| **20**| `PA9` | `IN_D1` | `TIM1_CH2` / GPIO | Motor Driver | Channel D PWM / Direction Input 1 |
| **21**| `PA11`| `RX` | `USART1_RX` (AF7) | Serial Comms | UART Receive (routed to test point `TP2`) |
| **22**| `PA12`| `TX` | `USART1_TX` (AF7) | Serial Comms | UART Transmit (routed to test point `TP1`) |
| **23**| `PA13`| `SWDIO` | `SWDIO` (AF0) | Debug | Serial Wire Debug Data (SWD) |
| **24**| `PA14`| `SWCLK` | `SWCLK` (AF0) | Debug | Serial Wire Debug Clock (SWD) |
| **25**| `PA15`| `IN_A1` | `TIM2_CH1` / GPIO | Motor Driver | Channel A PWM / Direction Input 1 |
| **26**| `PB3` | `SCL` | `I2C2_SCL` (AF8) | Host Comms | Host I2C Clock (BareBoot bootloader target) |
| **27**| `PB4` | `SDA` | `I2C2_SDA` (AF8) | Host Comms | Host I2C Data (BareBoot bootloader target) |
| **28**| `PB5` | `IN_A2` | `TIM3_CH2` / GPIO | Motor Driver | Channel A PWM / Direction Input 2 |
| **29**| `PB6` | `3D_SCL` | `I2C1_SCL` (AF4) | Sensor Bus | 3D Magnetic Sensor I2C Clock |
| **30**| `PB7` | `3D_SDA` | `I2C1_SDA` (AF4) | Sensor Bus | 3D Magnetic Sensor I2C Data |
| **31**| `BOOT0`| `BOOT` | Boot Selector | Control | Boot configuration (10 kΩ pull-down R30 to GND) |
| **32**| `PB8` | *NC* | None (X) | Unused | Unconnected / Not bonded out |
| **33**| `EP` | `GND` | Thermal Exposed Pad| Ground | Central thermal & ground pad to system GND |

---

## 3. Subsystem Breakdown

### 3.1 Motor Drivers (TI DRV8411ARTE)
The board incorporates dual H-bridge motor drivers (DRV8411A) configuring 4 H-bridge circuits:

| Channel | Input 1 (`IN_x1`) | Input 2 (`IN_x2`) | Current Sense (`ISENS_x`) |
|:---:|:---:|:---:|:---:|
| **Bridge A** | `PA15` | `PB5` | `PA2` (`ADC1_INP14`) |
| **Bridge B** | `PA8` | `PB15` | `PA3` (`ADC1_INP15`) |
| **Bridge C** | `PA6` | `PA7` | `PA4` (`ADC1_INP18`) |
| **Bridge D** | `PA9` | `PB0` | `PA5` (`ADC1_INP19`) |

#### Current Sensing Circuitry:
- **Mirror Resistors**: Each current sense line (`ISENS_A`, `ISENS_B`, `ISENS_C`, `ISENS_D`) is terminated with a **1 kΩ (1000 Ω)** resistor to GND.
- **DRV8411A Mirror Ratio**: $A_{IPROPI} = 200\ \mu\text{A/A}$ (1:5000 ratio).
- **Transfer Function**:
  $$V_{ISENS} = I_{motor} \times A_{IPROPI} \times R_{SENS} = I_{motor} \times (200 \times 10^{-6}\text{ A/A}) \times 1000\ \Omega = I_{motor} \times 0.2\text{ V/A}$$
  $$I_{motor} = \frac{V_{ISENS}}{0.2\text{ V/A}} = 5.0 \times V_{ISENS}$$
- **ADC Scaling (12-bit, 3.3V reference)**:
  - 1 ADC count $\approx \frac{3.3\text{ V}}{4095 \times 0.2\text{ V/A}} \approx 4.029\text{ mA/count}$.
  - Full-scale current measurable: $\approx 16.5\text{ A}$ (well above DRV8411A 1.5A peak limit).

#### Control Logic (DRV8411A IN/IN Mode):
| `IN_x1` | `IN_x2` | Output State | Function |
|:---:|:---:|:---:|:---|
| **0** | **0** | Hi-Z | **Coast** (Outputs disabled, lowest power) |
| **1** | **0** | OUT1=H, OUT2=L | **Forward** drive |
| **0** | **1** | OUT1=L, OUT2=H | **Reverse** drive |
| **1** | **1** | OUT1=L, OUT2=L | **Brake** (Low-side slow decay) |

> [!TIP]
> Keep `IN_x1 = 0` and `IN_x2 = 0` during initialization and boot to ensure the motor driver bridges remain high-impedance (Coast mode) and draw zero drive current.

---

### 3.2 Dual I2C Bus Topology

1. **Host Communication Bus (`I2C2`)**:
   - **Pins**: `PB3` (SCL), `PB4` (SDA)
   - **Alternate Function**: AF8 (`I2C2_SCL`, `I2C2_SDA`)
   - **Purpose**: External communication with host controller, USB-to-I2C bridge (e.g. CH341), and BareBoot bootloader (Target Address `0x40`).

2. **Sensor Bus (`I2C1`)**:
   - **Pins**: `PB6` (3D_SCL), `PB7` (3D_SDA)
   - **Alternate Function**: AF4 (`I2C1_SCL`, `I2C1_SDA`)
   - **Purpose**: Dedicated onboard magnetic angle / position sensor (e.g. TMAG5273 3D Hall-effect sensor).
   - **Pull-ups**: Requires internal pull-ups (`GPIO_PULLUP` / ~40 kΩ) or onboard external pull-ups.

---

### 3.3 Status LEDs

- **`PA0`** $\rightarrow$ **`LED_B` (Blue)**
- **`PA1`** $\rightarrow$ **`LED_R` (Red)**
- **Circuit**: Common-anode tied to `3V3` via 1 kΩ series resistors.
- **Drive Logic**:
  - `digitalWrite(LED, LOW)` / `Level::Low` $\rightarrow$ **LED ON**
  - `digitalWrite(LED, HIGH)` / `Level::High` $\rightarrow$ **LED OFF**

---

### 3.4 Power & Clock Recommendations

* **Supply Rails**:
  - `VDD` and `VDDA` must be supplied with 3.3V.
  - When debugging via SWD, ensure the debugger's 3.3V regulator can supply at least 150–200 mA if operating at high clock rates, or provide external 3.3V/5V power to avoid LDO brownout.
* **Internal Oscillators**:
  - **Recommended Operational Clock**: 64 MHz HSI (`RCC_HSI_DIV1`) with `PWR_REGULATOR_VOLTAGE_SCALE3` for low-power, cool-running operation.
  - **High Performance Mode**: Up to 250 MHz using internal CSI/HSI48 with PLL1 and Scale 0 when running from full board power.
* **Sleep Mode**: Use `WFE`/`WFI` during idle/delay periods to keep die temperature near ambient (~45–50 °C).
