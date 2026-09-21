# Varunetra — Software-Defined Sonar Transmitter Payload

**वरुणनेत्र** *(Varuna — Vedic god of the ocean, + Netra — "eye")*
**"The eye of Varuna" — a real-time, environment-adaptive LFM sonar chirp transmitter for AUVs.**


  Add a hero image/GIF here once available:
  ![VaruNaad hardware prototype](chirp.jpg)

[![Platform](https://img.shields.io/badge/MCU-STM32F407VE-blue)](https://www.st.com/en/microcontrollers-microprocessors/stm32f407ve.html)
[![Language](https://img.shields.io/badge/firmware-C%20%2F%20HAL-orange)]()
[![License](https://img.shields.io/badge/license-MIT-green)](LICENSE)

---

## Overview

Traditional sonar transmitters emit a fixed-frequency pulse — the waveform is baked in at design time, forcing a permanent trade-off between range (low frequency) and resolution (high frequency). Real underwater environments don't stay fixed: turbidity, depth, and target distance change constantly during a single AUV mission.

**Varunetra** is a physical, self-contained embedded transmitter that behaves like a Software-Defined Radio for sonar. It synthesizes a Linear Frequency Modulated (LFM) chirp entirely in real time on an STM32F407VE, and reshapes that waveform on the fly based on three environmental control inputs — with **zero CPU load during transmission**, since the DAC is streamed purely by DMA under hardware-timer control.

This repository contains the firmware, the analog reconstruction filter design, and a Python "digital twin" simulator used to validate the firmware's math independently of the hardware.

---

## Key Features

- **Real-time LFM chirp synthesis** via a phase-accumulator DDS algorithm running on the Cortex-M4F's hardware FPU
- **Zero-CPU-overhead transmission** — TIM6 (hardware timer) triggers DAC1 via DMA1 in Normal mode; the core is free for the entire burst
- **Live environmental adaptation** — 3 potentiometers (simulating sensor inputs) reshape the chirp instantly:
  - Sweep bandwidth (clarity/resolution)
  - Pulse duration (energy/range)
  - Amplitude (transmit power)
- **Hamming-windowed envelope** to suppress spectral sidelobes and protect the analog front-end from sharp edge transients
- **Active analog reconstruction filter** (Sallen-Key + unity-gain buffer) to convert the DAC's staircase output into a clean sine sweep
- **Python digital twin** — a matplotlib/scipy simulator that mirrors the firmware's exact math, used as ground truth for oscilloscope validation
- **UART debug telemetry** — live raw ADC + computed chirp parameter readout for verification without a logic analyzer

---

## System Architecture

```mermaid
flowchart TD
    A["3 Potentiometers<br/>PA1 / PA2 / PA3"] --> B["ADC1<br/>PA1 / PA2 / PA3"]
    B --> C["sensors_read&#40;&#41;<br/>raw counts to f0, f1, duration, amplitude"]
    C --> D["chirp_gen_compute_var&#40;&#41;<br/>Phase-accumulator DDS + Hamming window"]
    D --> E["DMA1 Stream5 Ch7<br/>Normal mode, buffer to DAC1_DHR12R1"]
    F["TIM6 to TRGO<br/>Update Event, fs approx 1.2537 MSPS"] -->|triggers| E
    E --> G["DAC1_CH1 to PA4<br/>12-bit staircase output"]
    G --> H["Sallen-Key LPF<br/>LM318, R=2.2k, C1=100pF, C2=220pF<br/>fc approx 488 kHz"]
    K["ICL7660<br/>plus 5V to minus 5V"] -->|dual supply| H
    H --> J["Clean sine LFM sweep<br/>to transducer / oscilloscope"]

    style A fill:#e8f4fd,stroke:#2b7cb3
    style B fill:#e8f4fd,stroke:#2b7cb3
    style C fill:#eafaf1,stroke:#27ae60
    style D fill:#eafaf1,stroke:#27ae60
    style E fill:#fff3e0,stroke:#e67e22
    style F fill:#fff3e0,stroke:#e67e22
    style G fill:#fdecea,stroke:#c0392b
    style H fill:#f4ecf7,stroke:#8e44ad
    style K fill:#f4ecf7,stroke:#8e44ad
    style J fill:#eafaf1,stroke:#27ae60
```

### Hardware Components

| Component | Part / Value |
|---|---|
| MCU | STM32F407VE (DevEBox), Cortex-M4F, Hard-FPU |
| DAC output | DAC1_CH1 → PA4, 12-bit right-aligned |
| Trigger timer | TIM6, PSC=0, ARR=66 → fs ≈ 1,253,731 Hz |
| DMA | DMA1 Stream5 Ch7, Normal mode, half-word |
| Sensor ADC | ADC1, channels 1/2/3 → PA1/PA2/PA3 |
| Debug UART | USART1, PA9(TX)/PA10(RX), 115200 baud |
| Trigger button | PE4 (active-low) |
| Scope sync pin | PB0 (high during active burst) |
| Filter op-amp | LM318 |
| Negative rail generator | ICL7660 (charge-pump voltage inverter) |

---

## Potentiometer / Environmental Sensor Mapping

| Pot | Pin | Controls | Range | Simulates |
|---|---|---|---|---|
| POT1 | PA1 (ADC1_IN1) | Sweep bandwidth (f0 fixed @ 100 kHz, f1 scales) | f1: 150 kHz – 500 kHz | Water turbidity — clearer water enables a wider, higher-resolution sweep |
| POT2 | PA2 (ADC1_IN2) | Pulse duration | 0.5 ms – 5 ms | Target distance / required acoustic energy |
| POT3 | PA3 (ADC1_IN3) | Amplitude scale | 10% – 100% | Water depth / attenuation compensation |

---

## Analog Reconstruction Filter

The STM32's DAC output is a zero-order-hold staircase — each sample holds its voltage until the next one, which is visible as steps rather than a smooth sine. An active Sallen-Key low-pass filter cleans this into a proper analog sine sweep.

**Sallen-Key low-pass (unity gain, LM318), dual-supply generated via ICL7660**

| Component | Value |
|---|---|
| R1 = R2 | 2.2 kΩ |
| C1 (node-to-GND) | 56 pF |
| C2 (feedback) | 220 pF |
| Cutoff frequency | ≈ 625 kHz |
| Q | ≈ 0.6 (gentle rolloff, no peaking) |


**Power — ICL7660 charge-pump voltage inverter**
LM318 is not rail-to-rail, so a single 5V supply leaves little headroom for the signal's peaks. The ICL7660 generates a −5V rail from the STM32's existing +5V, giving LM318 a proper ±5V dual supply.

| ICL7660 Pin | Connection |
|---|---|
| 8 (V+) | +5V (from STM32) |
| 3 (GND) | GND |
| 2 (CAP+) ↔ 4 (CAP−) | 10 µF capacitor between them |
| 7 (VOUT) | −5V output → LM318 V− |
| 7 (VOUT) → GND | 10 µF output smoothing capacitor |
| 5 (OSC), 6 (LV), 1 (NC) | Left unconnected (internal oscillator, >3.5V operation) |

<!--
  Add filter schematic photo/render here:
  ![Sallen-Key filter schematic](docs/images/filter_schematic.png)
-->

---

## Repository Structure

```
.
├── Core/
│   ├── Inc/
│   │   ├── chirp_gen.h        # LFM generator public interface
│   │   └── sensors.h          # Potentiometer reading interface
│   └── Src/
│       ├── chirp_gen.c        # Phase-accumulator DDS + Hamming window + DMA control
│       ├── sensors.c          # ADC reading + pot-to-parameter mapping
│       └── main.c             # Peripheral init, button handler, UART debug
├── tools/
│   └── chirp_simulation_twin.py   # Python digital-twin simulator (matplotlib + scipy)
├── docs/
│   ├── CUBEMX_CHECKLIST_AND_NOTES.md   # Full CubeMX peripheral configuration reference
│   └── images/                          # Oscilloscope captures, schematics, board photos
└── README.md
```

---

## Getting Started

### Prerequisites
- [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html)
- [STM32CubeProgrammer](https://www.st.com/en/development-tools/stm32cubeprog.html) (or use CubeIDE's built-in flashing)
- Python 3.x with `numpy`, `scipy`, `matplotlib` (for the digital twin)
- STM32F407VE board (DevEBox or equivalent), oscilloscope for validation

### Build & Flash
```bash
git clone https://github.com/Dharani-Sundharam/SPECTRUM_SIX_SIH2026.git
```
1. Open the project in STM32CubeIDE
2. Verify peripheral configuration against `docs/CUBEMX_CHECKLIST_AND_NOTES.md`
3. Build (Ctrl+B) and flash to the board
4. Wire 3 potentiometers to PA1/PA2/PA3 (wiper → pin, outer legs → 3.3V and GND)
5. Press the button on PE4 to fire a chirp burst
6. Probe PA4 (raw DAC) or the filter output (clean sine) with an oscilloscope

### Run the Digital Twin
```bash
pip install numpy scipy matplotlib
python tools/chirp_simulation_twin.py
```
Set the 3 sliders to match your physical pot positions and compare the simulated waveform/FFT against your oscilloscope capture — this is the ground-truth reference used throughout development to separate firmware bugs from hardware/wiring issues.

---

## Validation Results

<!--
  Add oscilloscope captures here, e.g.:

  ### Full-band chirp (100–500 kHz)
  ![Full sweep waveform](docs/images/full_sweep_waveform.jpg)

  ### FFT — confirms energy spans f0 to f1
  ![FFT spectrum](docs/images/fft_spectrum.jpg)

  ### Digital twin vs hardware comparison
  ![Digital twin comparison](docs/images/twin_vs_hardware.png)
-->

- Hamming-windowed envelope confirmed on oscilloscope, matching the digital twin's simulated output
- AC-coupled FFT capture confirms transmitted energy spans the intended f0–f1 band (a DC-coupled FFT is dominated by the DAC's ~1.65V bias and should not be used for spectral verification)
- Pot-driven parameter changes verified via UART debug readout of raw ADC counts alongside computed `f0, f1, duration, amplitude`

---

## Literature Survey / References

1. Yang, H. et al., *"Implementation of DDS Chirp Signal Generator on FPGA,"* Ajou University / Korea Aerospace Research Institute — architectural reference for the phase-accumulator DDS approach used in this firmware.
2. *Adaptive Transmit Waveform Design*, Naval Undersea Warfare Center — [arXiv:2111.08746](https://arxiv.org/abs/2111.08746) — cognitive active sonar systems that adapt waveform parameters to the sensed acoustic environment.
3. *Enhanced target detection using a new cognitive sonar waveform design in shallow water* — [ScienceDirect](https://www.sciencedirect.com/science/article/abs/pii/S0003682X23000683)
4. *AUVs for Seabed Surveying: A Comprehensive Review of Side-Scan Sonar-Based Target Detection* — [DOAJ](https://doaj.org/article/7bc1dac49dd744e08cd3bd21dc729dc6)
5. *Pulse Compression Radar System Analysis* (windowing/sidelobe suppression) — [Microwave Journal](https://www.microwavejournal.com/articles/25733-pulse-compression-radar-system-analysis)

---

## Team

**Team SPECTRUM_SIX** — Smart India Hackathon 2026

<!-- Add team member names/roles here -->

---

## License

<!-- Confirm and add a LICENSE file matching your choice -->
This project is licensed under the MIT License — see [LICENSE](LICENSE) for details.
