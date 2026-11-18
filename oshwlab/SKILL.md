---
name: oshwlab-project-docs
description: Standardize, extract, and generate publication-ready OSHWLab project documentation (README.md) and asset structures for any open-source hardware and embedded firmware repository.
---

# OSHWLab Project Documentation Skill

This skill provides a project-agnostic framework for synthesizing hardware designs, firmware codebases, mechanical assets, and sponsorship details into a standardized, publication-ready `README.md` formatted specifically for OSHWLab project pages.

---

## 1. Directory & Asset Conventions

Any repository implementing this standard must adhere to the following directory layout:

```
<project-root>/
├── README.md               <-- Primary OSHWLab-ready project documentation
├── oshwlab/
│   ├── SKILL.md            <-- This skill definition
│   └── assets/             <-- Visual media (PCB photos, bench setups, CAD renders)
│       └── pcb_spark.jpg   <-- Hardware photo for Spark-sponsored projects
├── firmware/               <-- Firmware source files and PlatformIO / build configs
├── hardware/               <-- Schematics, PCB layout files, Gerber/BOM exports
└── mechanical/             <-- 3D models (STEP, STL, DXF, 3MF)
```

---

## 2. Standardized README Section Architecture

The root `README.md` must strictly follow this order of sections:

```
1. Title & Header
2. Overview (Single-paragraph project summary)
3. Key Features
4. Hardware Specifications
5. Hardware Architecture & Pinout
6. Mechanical Assembly
7. Firmware & Control System
8. EasyEDA Spark Sponsorship (Conditional)
```

### Section Breakdown & Rules

#### Section 1: Overview
- Must be a single opening paragraph directly beneath the project title.
- States the primary identity of the project, its core problem statement, target applications, and the main architectural highlight (e.g., target MCU, power topology, control paradigm) using neutral, professional engineering language.

#### Section 2: Key Features
- Bulleted list of 5 to 8 key functional technical specifications.
- Use direct functional labels (e.g. `Microcontroller`, `Power Stage`, `Sensor Interface`, `Host Bus`, `Form Factor`) rather than promotional banners (e.g. avoid `High-Performance Compute`, `Ultra-Compact Form Factor`).
- Emphasizes computing core, power stages, sensor interfaces, host bus capabilities, and form factor objectively.

#### Section 3: Hardware Specifications
- **Specifications Table**: Concise table listing core parameters (MCU, input voltage, power stage, sensors, host communication, programming interface).
- **Implementation Notes**: Explicit technical notes documenting:
  - Hardware strap configurations (e.g. mode pins floating vs. pulled).
  - Missing or omitted signals (e.g. unrouted fault indicators, absent voltage sense).
  - Power envelopes and current limits (e.g. USB port budget vs. external battery limits).
  - Sensor bus sharing constraints or timing considerations.

#### Section 4: Hardware Architecture & Pinout
- **System Diagram**:
  - Render directly as inline Mermaid (` ```mermaid ... ``` `) code blocks. OSHWLab natively parses and renders Mermaid diagrams in its markdown viewer.
  - Must use a compact vertical Mermaid `flowchart TD` layout (two-column design with control pipeline on the left, telemetry/feedback on the right) to avoid horizontal scrolling on web and markdown viewers.
  - All edge labels with parentheses, commas, or special characters must be enclosed in double quotes (e.g., `-->|"PB15 (nSLEEP)"|`).
- **Pinout Tables**:
  - Tables divided into logical functional groups: Power Stage / Actuation, Sensors, Analog Telemetry, Host Bus / Debug / Indicators.
  - Columns: `Signal Name`, `MCU Pin`, `Peripheral / Alternate Function`, `Role / Description`.

#### Section 5: Mechanical Assembly
- Direct link to `mechanical/` (or the GitHub CAD directory).
- Kinematic specifications, reduction ratios, signs, and physical design notes.

#### Section 6: Firmware & Control System
- **Source Link**: Must contain a direct link to the `firmware/` directory and corresponding GitHub path.
- **Control Strategy**: Summary of control loops, commutation math, PID cascades, or state machines. Include compact Mermaid diagram if applicable.
- **Bootloader & Upload**: Instructions for flashing the target board, including bootloader profiles (UART, I2C, USB, SWD), soft-reset mechanisms, and CLI commands.

#### Section 7: EasyEDA Spark Sponsorship (Conditional)
Include this section **only** if the project was sponsored by the OSHWLab Spark campaign.

> **Note**: Do not include a text heading (such as `## EasyEDA Spark`) above this section. The animated `spark.gif` banner itself functions as the visual section header.

```markdown
![EasyEDA Spark](https://raw.githubusercontent.com/<owner>/<repo>/main/oshwlab/assets/spark.gif)

![Spark PCB Boards](https://raw.githubusercontent.com/<owner>/<repo>/main/oshwlab/assets/spark-pcb.jpg)

This project's PCB and PCBA service was sponsored by the [OSHWLab Spark campaign](https://oshwlab.com/).
Project was completed in [Month YYYY].
```

---

## 3. URL Resolution Rules for OSHWLab

**CRITICAL**: When markdown is pasted into or rendered by EasyEDA / OSHWLab, relative local paths will not resolve. All media and repository references must use absolute GitHub URLs:

1. **Images & Photos**: Must use GitHub raw CDN links:
   ```markdown
   ![Alt Text](https://raw.githubusercontent.com/<owner>/<repo>/main/oshwlab/assets/<filename>)
   ```
2. **Directory & File Links**: Links to `firmware/`, `mechanical/`, or specific code files must point to GitHub web URLs:
   ```markdown
   [firmware/](https://github.com/<owner>/<repo>/tree/main/firmware)
   [mechanical/](https://github.com/<owner>/<repo>/tree/main/mechanical)
   ```

---

## 4. Repository Discovery & Audit Procedure

When applying this skill to any target repository, follow this execution sequence:

1. **Hardware Inspection**:
   - Locate MCU family, clock frequency, memory specs, and package from design files or board definitions.
   - Scan schematics, `.ioc` files, or pinout documents for complete pin assignments.
   - Check component datasheets for mode pin straps, shunt gains, and voltage limits.
2. **Firmware Inspection**:
   - Locate `platformio.ini`, `CMakeLists.txt`, `Cargo.toml`, or `Makefile` to verify build frameworks.
   - Check `#define` macros and pin assignments in firmware source to confirm pin alignment with hardware.
   - Identify communication protocols (UART, I2C, SPI, CAN) and bootloader mechanisms.
3. **Mechanical Inspection**:
   - Scan for CAD models in `mechanical/`, `cad/`, or `3d/`.
   - Record gear ratios, kinematic signs, and assembly fixtures.
4. **Sponsorship & Timeline Verification**:
   - Verify if the project was funded under the OSHWLab Spark initiative.
   - Record the completion month and year, and verify the presence of the PCB photo.
5. **Asset Staging**:
   - Place all referenced images (photos, CAD captures) into `oshwlab/assets/`.
   - Diagrams remain inline as native Mermaid code blocks within `README.md`.
   - Ensure all image links in the markdown use absolute raw GitHub URLs (`https://raw.githubusercontent.com/<owner>/<repo>/main/oshwlab/assets/<filename>`) so they resolve properly both in GitHub and on EasyEDA / OSHWLab.

---

## 5. Guardrails & Style Rules

- **Strict Repository Scope Boundary**: Only inspect, read, copy, or reference files located within the current target repository root. NEVER access external directories, search unrelated project folders, or import 3D models, renders, or metadata from outside the repository boundaries. All documentation, pinouts, firmware facts, and assets must be sourced strictly from within the target repository itself.
- **Professional Engineering Tone (Zero Hype)**:
  - Maintain an objective, concise, and professional engineering tone across all sections, tables, and diagrams.
  - **Prohibit Promotional Hype & Superlatives**: Never use exaggerated or promotional buzzwords such as "ultra-compact", "high-performance", "versatile", "smart", "blazing", "revolutionary", "seamless", "effortless", "cutting-edge", or "game-changing".
  - **Factual Technical Descriptors**: State exact part numbers, measured parameters, electrical ratings, clock frequencies, tolerances, and operational facts plainly and accurately.
  - **Functional Feature Labels**: Use clear functional category labels (e.g. `Microcontroller`, `Motor Driver Stage`, `Position Sensing Interface`, `Host Bus Topology`, `Form Factor`) rather than promotional banners (e.g. avoid `High-Performance Compute`, `Ultra-Compact Form Factor`).
- **Generous Tables & Diagrams**: Use structured Markdown tables and Mermaid diagrams generously across all technical sections whenever they improve clarity. High-density reference tables (logic truth tables, pinout mappings, electrical characteristics, register maps) and diagrams (bus topologies, cascaded control loops, state machines) should be prioritized over dense prose.
- **Zero Emojis**: Emojis must never be used anywhere in UI text, code, documentation, tables, logs, or commit messages. Use clean, standardized technical text labels.
- **Compact Diagram Width**: Avoid horizontal `flowchart LR` layouts spanning more than 3 columns. Use `flowchart TD` with compact subgraphs.
- **LaTeX Math KaTeX**: Use KaTeX syntax (`$inline$` or `$$display$$`) for mathematical relations, gear reductions, and formulas.
- **Self-Contained Markdown**: Ensure all tables, code snippets, and blockquotes render cleanly in standard GitHub-flavored Markdown without requiring proprietary plugins.
