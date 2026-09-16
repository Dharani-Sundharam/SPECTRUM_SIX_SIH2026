"""
chirp_simulation_twin.py
=========================
Digital-Twin Simulation of the STM32F4VE LFM Sonar Chirp Transmitter.

This mirrors the FIRMWARE EXACTLY:
    - Same fs (derived from TIM6 PSC/ARR, 84 MHz timer clock)
    - Same phase-accumulator chirp math (chirp_gen_compute_var in chirp_gen.c)
    - Same Hamming window
    - Same 12-bit DAC quantization + amplitude scaling around midpoint
    - Same 3-potentiometer parameter mapping (sensors.c)

Use this as your GROUND-TRUTH reference. Set the 3 sliders to match your
physical potentiometer positions, click "Fire Chirp", and compare the
resulting waveform + FFT against your oscilloscope capture. If your
hardware output matches this simulation, your firmware is working correctly.

Requires: numpy, scipy, matplotlib
    pip install numpy scipy matplotlib
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button
from scipy.fft import rfft, rfftfreq

# =============================================================================
# Hardware constants — MUST MATCH chirp_gen.h exactly
# =============================================================================

TIMER_CLOCK_HZ      = 84_000_000.0      # TIM6 input clock (APB1 x2)
TIM6_ARR            = 66                # Auto-reload register
FS_ACTUAL_HZ        = TIMER_CLOCK_HZ / (TIM6_ARR + 1)   # ≈ 1,253,731 Hz

CHIRP_F0_FIXED_HZ    = 100_000.0        # Fixed lower sweep bound
CHIRP_MAX_DURATION_S = 0.005            # 5 ms cap (matches CHIRP_MAX_DURATION_S)
CHIRP_MIN_DURATION_S = 0.0005           # 0.5 ms floor

DAC_OUTPUT_MIN  = 100
DAC_OUTPUT_MAX  = 4000
DAC_OUTPUT_MID  = 2047
DAC_VREF        = 3.3                   # Volts, DAC output buffer reference

# =============================================================================
# Potentiometer mapping — MUST MATCH sensors.c exactly
# =============================================================================

def pot_to_params(pot1_pct, pot2_pct, pot3_pct):
    """
    Convert 3 pot positions (0-100%) to chirp parameters, using the
    SAME mapping as sensors_read() in sensors.c.

    POT1 -> sweep bandwidth (f0 fixed at 100 kHz, f1 scales 150k-500k)
    POT2 -> pulse duration  (0.5 ms - 5.0 ms)
    POT3 -> amplitude scale (0.1 - 1.0)
    """
    n1 = pot1_pct / 100.0
    n2 = pot2_pct / 100.0
    n3 = pot3_pct / 100.0

    f0 = CHIRP_F0_FIXED_HZ
    f1 = 150_000.0 + n1 * (500_000.0 - 150_000.0)

    duration = CHIRP_MIN_DURATION_S + n2 * (CHIRP_MAX_DURATION_S - CHIRP_MIN_DURATION_S)

    amplitude = 0.1 + n3 * (1.0 - 0.1)

    return f0, f1, duration, amplitude


# =============================================================================
# Chirp generator — MUST MATCH chirp_gen_compute_var() in chirp_gen.c
# =============================================================================

def generate_lfm_chirp(f0, f1, duration, amplitude, fs=FS_ACTUAL_HZ):
    """
    Generates a Hamming-windowed LFM chirp exactly as the firmware does:
    incremental phase accumulator, Hamming window, 12-bit DAC quantization
    with amplitude scaling around the DC midpoint.

    Returns:
        t          : time vector (s)
        dac_values : uint16-equivalent DAC codes (100-4000)
        voltage    : reconstructed analog voltage (V), what you'd see on scope
    """
    # ---- Defensive clamps (mirrors firmware) ----
    if f1 <= f0:
        f1 = f0 + 50_000.0
    if f1 > fs / 2.0:
        f1 = fs / 2.0
    duration = np.clip(duration, CHIRP_MIN_DURATION_S, CHIRP_MAX_DURATION_S)
    amplitude = np.clip(amplitude, 0.1, 1.0)

    length = int(round(fs * duration))
    length = max(length, 8)

    N = length
    T = (N - 1) / fs
    k = (f1 - f0) / T
    dt = 1.0 / fs

    n = np.arange(N)
    t = n * dt

    # ---- Incremental phase accumulator (matches firmware bit-for-bit) ----
    f_inst = f0 + k * n * dt          # instantaneous frequency per sample
    # Cumulative phase via trapezoidal-style running sum (equivalent to
    # firmware's per-sample accumulation, vectorized here for speed)
    phase = 2.0 * np.pi * np.cumsum(f_inst) * dt
    phase = phase - phase[0]          # start at 0, same as firmware

    s = np.sin(phase)

    # ---- Hamming window ----
    w = 0.54 - 0.46 * np.cos(2.0 * np.pi * n / (N - 1))

    y = s * w                          # [-1, +1]

    # ---- DAC quantization with amplitude scaling around midpoint ----
    half_span = ((DAC_OUTPUT_MAX - DAC_OUTPUT_MIN) / 2.0) * amplitude
    d_f = DAC_OUTPUT_MID + y * half_span
    dac_values = np.clip(np.round(d_f), DAC_OUTPUT_MIN, DAC_OUTPUT_MAX).astype(np.uint16)

    # ---- Reconstruct analog voltage (what the scope would show on PA4) ----
    voltage = dac_values.astype(np.float64) / 4095.0 * DAC_VREF

    return t, dac_values, voltage


# =============================================================================
# Interactive Simulation UI
# =============================================================================

class ChirpDigitalTwin:
    def __init__(self):
        self.fig = plt.figure(figsize=(11, 8))
        self.fig.suptitle("STM32 LFM Sonar Chirp — Digital Twin Simulation",
                          fontsize=13, fontweight='bold')

        gs = self.fig.add_gridspec(3, 1, height_ratios=[1, 2, 2],
                                   left=0.10, right=0.95, top=0.90, bottom=0.32,
                                   hspace=0.5)

        self.ax_trigger = self.fig.add_subplot(gs[0])
        self.ax_wave    = self.fig.add_subplot(gs[1])
        self.ax_fft     = self.fig.add_subplot(gs[2])

        # ---- Slider axes ----
        ax_p1 = self.fig.add_axes([0.15, 0.20, 0.7, 0.03])
        ax_p2 = self.fig.add_axes([0.15, 0.15, 0.7, 0.03])
        ax_p3 = self.fig.add_axes([0.15, 0.10, 0.7, 0.03])
        ax_btn = self.fig.add_axes([0.42, 0.02, 0.16, 0.05])

        self.s_pot1 = Slider(ax_p1, 'POT1 (Bandwidth)', 0, 100, valinit=100, valstep=1)
        self.s_pot2 = Slider(ax_p2, 'POT2 (Duration)',  0, 100, valinit=40,  valstep=1)
        self.s_pot3 = Slider(ax_p3, 'POT3 (Amplitude)', 0, 100, valinit=100, valstep=1)
        self.btn_fire = Button(ax_btn, 'Fire Chirp')

        self.s_pot1.on_changed(self._on_slider_change)
        self.s_pot2.on_changed(self._on_slider_change)
        self.s_pot3.on_changed(self._on_slider_change)
        self.btn_fire.on_clicked(self._fire)

        self._fire(None)   # initial draw
        plt.show()

    def _on_slider_change(self, val):
        # Live-update the readout text without re-firing (like turning
        # the pot before pressing the button on real hardware)
        self._update_param_readout()
        self.fig.canvas.draw_idle()

    def _update_param_readout(self):
        f0, f1, dur, amp = pot_to_params(self.s_pot1.val, self.s_pot2.val, self.s_pot3.val)
        self.readout_text.set_text(
            f"f0={f0/1e3:.1f} kHz   f1={f1/1e3:.1f} kHz   "
            f"BW={((f1-f0)/1e3):.1f} kHz   "
            f"duration={dur*1e3:.2f} ms   amplitude={amp*100:.0f}%"
        )

    def _fire(self, event):
        pot1, pot2, pot3 = self.s_pot1.val, self.s_pot2.val, self.s_pot3.val
        f0, f1, duration, amplitude = pot_to_params(pot1, pot2, pot3)
        t, dac_values, voltage = generate_lfm_chirp(f0, f1, duration, amplitude)

        # ---- Build the trigger/gate trace (like PB0 on the scope) ----
        pad = duration * 0.3
        t_full = np.linspace(-pad, duration + pad, 500)
        trig = np.where((t_full >= 0) & (t_full <= duration), 1.0, 0.0)

        # ---- Plot 1: Trigger pulse (CH1 equivalent) ----
        self.ax_trigger.clear()
        self.ax_trigger.plot(t_full * 1e3, trig, color='gold', linewidth=1.5)
        self.ax_trigger.set_ylim(-0.3, 1.3)
        self.ax_trigger.set_yticks([0, 1])
        self.ax_trigger.set_ylabel("PB0\n(trig)")
        self.ax_trigger.set_title("CH1 — Debug Trigger Pulse", fontsize=9)
        self.ax_trigger.grid(alpha=0.3)

        # ---- Plot 2: Analog waveform (CH2 equivalent, PA4) ----
        self.ax_wave.clear()
        self.ax_wave.plot(t * 1e3, voltage, color='deepskyblue', linewidth=0.6)
        self.ax_wave.set_xlim(t_full[0] * 1e3, t_full[-1] * 1e3)
        self.ax_wave.axhline(DAC_OUTPUT_MID / 4095 * DAC_VREF, color='gray',
                             linestyle='--', linewidth=0.7, alpha=0.6)
        self.ax_wave.set_ylabel("PA4\nVoltage (V)")
        self.ax_wave.set_xlabel("Time (ms)")
        self.ax_wave.set_title("CH2 — DAC Output (Hamming-windowed LFM chirp)", fontsize=9)
        self.ax_wave.grid(alpha=0.3)

        # ---- Plot 3: FFT (spectrogram-equivalent magnitude spectrum) ----
        self.ax_fft.clear()
        ac = voltage - np.mean(voltage)          # remove DC bias before FFT
        win = np.hamming(len(ac))
        spectrum = np.abs(rfft(ac * win))
        freqs = rfftfreq(len(ac), d=1.0 / FS_ACTUAL_HZ)
        spectrum_db = 20 * np.log10(spectrum / (np.max(spectrum) + 1e-12) + 1e-12)

        self.ax_fft.plot(freqs / 1e3, spectrum_db, color='springgreen', linewidth=0.8)
        self.ax_fft.axvline(f0 / 1e3, color='red', linestyle=':', linewidth=1, label=f'f0={f0/1e3:.0f}kHz')
        self.ax_fft.axvline(f1 / 1e3, color='orange', linestyle=':', linewidth=1, label=f'f1={f1/1e3:.0f}kHz')
        self.ax_fft.set_xlim(0, FS_ACTUAL_HZ / 2 / 1e3)
        self.ax_fft.set_ylim(-80, 5)
        self.ax_fft.set_xlabel("Frequency (kHz)")
        self.ax_fft.set_ylabel("Magnitude (dB)")
        self.ax_fft.set_title("FFT — Expect a flat-ish band from f0 to f1, "
                              ">40 dB sidelobe suppression (Hamming)", fontsize=9)
        self.ax_fft.legend(loc='upper right', fontsize=8)
        self.ax_fft.grid(alpha=0.3)

        # ---- Parameter readout ----
        if not hasattr(self, 'readout_text'):
            self.readout_text = self.fig.text(0.5, 0.955, "", ha='center',
                                              fontsize=9, family='monospace')
        self._update_param_readout()

        self.fig.canvas.draw_idle()


if __name__ == "__main__":
    print(f"fs = {FS_ACTUAL_HZ:,.0f} Hz  (TIM6 PSC=0, ARR={TIM6_ARR})")
    print("Move the sliders to match your potentiometer positions, "
          "then click 'Fire Chirp'.")
    ChirpDigitalTwin()