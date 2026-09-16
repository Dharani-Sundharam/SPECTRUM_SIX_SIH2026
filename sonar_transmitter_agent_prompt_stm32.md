# AGENT BRIEF — Software-Defined Sonar Transmitter Payload (STM32 DDS Firmware)

You are implementing FIRMWARE ONLY, in C, using STM32 HAL/LL drivers. Do not invent requirements outside this document. Every numeric parameter below is a starting point pinned for THIS chip — if the user's actual STM32 part differs, ask for its exact part number and DAC/timer specs before changing numbers; do not silently substitute your own.

## 1. Goal (plain English)
Build STM32 firmware that:
1. Reads an environmental "sensor" (a potentiometer) via the onboard ADC.
2. Uses that reading to select one of several waveform presets (frequency range, pulse duration, amplitude).
3. Generates an LFM chirp (and optionally a plain tone / phase-coded pulse) as a precomputed sample buffer.
4. Applies a Hamming (or Hann/Blackman) window envelope to taper pulse edges.
5. Streams the buffer to the onboard DAC via DMA, triggered by a hardware timer, so the CPU is free once triggered — no per-sample CPU involvement, no blocking loops during output.

## 2. Assumed target hardware (CONFIRM WITH USER BEFORE CODING)
- Chip: STM32F411 (e.g. "Black Pill" board) — CONFIRM actual part number and datasheet DAC max conversion rate before finalizing Fs below.
- DAC: 1x onboard 12-bit DAC channel (DAC_OUT1), driven via DMA circular mode
- Timer: TIM6 (or TIM7) as the DAC trigger, generating the sample-rate clock
- ADC: onboard 12-bit ADC, single channel, polled or DMA, reading a potentiometer 0–3.3V
- Sample rate `Fs`: START at 1 MSPS as a target; THIS MUST BE VALIDATED against the actual DAC/timer achievable rate on hardware before trusting any chirp math — do not assume it works until measured on a scope.
- Chirp frequency range: 100 kHz to 500 kHz, both endpoints configurable (not hardcoded)
- Pulse duration `T`: configurable, default presets 1 ms and 4 ms
- Sample buffer resolution: 12-bit unsigned (DAC native format), centered at half-scale (2048) for the AC waveform

## 3. Required module/file structure
- `dds_core.c/.h` — 32-bit phase accumulator arithmetic (plain C uint32_t, no floating point in the per-sample path), quarter-wave sine LUT (precompute offline in Python, store as a `const uint16_t` array via `#include` or a generated `.c` file — do not compute sin() at runtime for the LUT contents themselves, only at table-generation time offline)
- `chirp_gen.c/.h` — given f0, f1, T, Fs: compute FTW0, FTW1, Nsamp, ΔFTW (this math CAN use floating point since it runs once per pulse trigger, not per sample); fills a sample buffer array by running the ramped-FTW loop (FTW += ΔFTW; phase_acc += FTW; sample = LUT[top bits of phase_acc]) — integer/fixed-point inside this loop
- `window.c/.h` — precomputed window coefficient table generation (can be done once at startup for each supported Nsamp length, or precomputed offline and stored as a const array); a function that multiplies the window into a chirp buffer in place
- `adc_env_sensor.c/.h` — reads the ADC channel, returns 12-bit reading
- `preset_selector.c/.h` — pure lookup: ADC reading → threshold comparison → returns a preset struct {f0, f1, T, amplitude_scale}. Use placeholder evenly-spaced thresholds across 0–4095 for now, clearly marked `// PLACEHOLDER — CALIBRATE WITH REAL POTENTIOMETER VOLTAGES`
- `dac_dma_output.c/.h` — configures TIM6 as DAC trigger, configures DMA circular/normal transfer from the sample buffer to DAC_OUT1, exposes `start_pulse()` / `is_pulse_active()`
- `main.c` — on a trigger (button or periodic), reads ADC, selects preset, calls chirp_gen to fill buffer, applies window, kicks off DMA transfer via dac_dma_output

## 4. Numeric format rules
- Per-sample DDS path (inside `chirp_gen`'s sample loop): integer arithmetic only — `uint32_t` phase accumulator and FTW, no floats, no doubles.
- Preset/parameter computation (FTW0, FTW1, ΔFTW, Nsamp from user-facing f0/f1/T/Fs): floating point is FINE here since it runs once per pulse trigger, not once per sample — do not over-engineer this part into fixed-point.
- Window multiply: window coefficients can be stored as `float` (0.0–1.0) precomputed once; the buffer multiply itself (window × sample) should be done as: `sample = (uint16_t)((int32_t)(sample - 2048) * window_coeff) + 2048` — i.e. center around the DAC midpoint before scaling, then re-add the offset. Do not apply the window before centering or you'll introduce DC offset errors.

## 5. Explicit "do not" list
- Do NOT call `sinf()`/`sin()` inside the per-sample chirp-fill loop — LUT lookup only.
- Do NOT recompute t² per sample for the chirp — use the ramped-FTW technique in Section 3.
- Do NOT use a blocking `HAL_Delay()`-based loop to "stream" samples to the DAC — must be DMA + timer trigger, matching the "hardware timer/DMA, don't stall the CPU" requirement.
- Do NOT hardcode f0/f1/T as `#define` constants used directly by the DAC path — they must flow through the preset struct set by `preset_selector`, since real-time adaptation is the whole point.
- Do NOT skip the window multiply — required to avoid sidelobe artifacts and protect the analog output stage, per the problem statement.
- Do NOT assume `Fs = 1 MSPS` is achievable without checking — validate the actual timer/DMA/DAC timing on hardware with a scope before treating any chirp bandwidth calculation as correct.

## 6. Test/validation requirements
- Provide a way to dump the generated sample buffer over UART or to a debugger-visible array, so it can be checked offline (Python/MATLAB FFT) against the expected chirp bandwidth for at least two different (f0, f1, T) presets.
- Provide a simple "tone test" mode (constant FTW, no ramp) to validate the base DDS core in isolation before testing the chirp ramp on top of it.

## 7. Acceptance criteria
- Triggering a pulse produces a windowed LFM chirp on the DAC pin sweeping from configured f0 to f1 over configured T, at the validated actual Fs.
- Changing the potentiometer changes which preset is active on the NEXT pulse (mid-pulse switching not required).
- Output verified clean on an oscilloscope FFT function against the offline-computed reference.
- CPU is free (not blocked) during sample output — confirm via a toggling GPIO or debugger that main-loop code keeps running while DMA streams the pulse.

## 8. Explicitly out of scope for you (the agent)
- Analog reconstruction filter design (Sallen-Key/RC), op-amp gain stage, potentiometer voltage calibration table, enclosure design, and power measurement are physical hardware tasks the human handles separately — do not attempt to simulate or design these.
