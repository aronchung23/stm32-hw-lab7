"""
HW-Lab7: figures of the two known-signal tests (same data / coefficients as the firmware)

  Test 1  CMSIS arm_fir_example: 1 kHz + 15 kHz, fs = 48 kHz, fir1(28, 6/24)
  Test 2  accelerometer LPF:     1 Hz + 16 Hz,   fs = 64 Hz,  fir1(28, 4/32)

The board prints the same numbers on the UART (SNR, FFT bins), see uart_log.txt.
Usage:  py plot_known_signal.py      -> known_signal_test.png
"""
import re
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
from scipy import signal

ROOT = Path(__file__).parent
DATA_C = ROOT / "lab07_dsp" / "Core" / "Src" / "arm_fir_data.c"


def c_array(text, name):
    body = re.search(name + r"\s*\[\s*\d+\s*\]\s*=\s*\{(.*?)\}", text, re.S).group(1)
    return np.array([float(v.rstrip("f")) for v in re.findall(r"[-+]?\d+\.\d+f?", body)])


def spectrum(x):
    return 2 * np.abs(np.fft.rfft(x)) / len(x)


def main():
    # ---------------- Test 1 ----------------
    txt = DATA_C.read_text()
    x1 = c_array(txt, "testInput_f32_1kHz_15kHz")
    ref1 = c_array(txt, "refOutput")
    h1 = signal.firwin(29, 6 / 24)
    y1 = signal.lfilter(h1, 1, x1)
    snr1 = 10 * np.log10(np.sum(ref1 ** 2) / np.sum((ref1 - y1) ** 2))
    fs1 = 48000

    # ---------------- Test 2 ----------------
    fs2, n2 = 64, 288
    h2 = signal.firwin(29, 4 / 32)
    n = np.arange(n2)
    x2 = np.sin(2 * np.pi * 1 * n / fs2) + 0.5 * np.sin(2 * np.pi * 16 * n / fs2)
    y2 = signal.lfilter(h2, 1, x2)
    g1 = abs(signal.freqz(h2, worN=[1.0], fs=fs2)[1][0])
    ideal = g1 * np.sin(2 * np.pi * 1 * (n - 14) / fs2)
    seg = slice(32, 288)
    snr2 = 10 * np.log10(np.sum(ideal[seg] ** 2) / np.sum((ideal[seg] - y2[seg]) ** 2))

    fig, ax = plt.subplots(3, 2, figsize=(12, 9.5))
    fig.suptitle("HW-Lab7 known-signal tests of the CMSIS-DSP FIR (arm_fir_f32)", fontsize=13)

    ax[0, 0].plot(x1, lw=0.9, label="input 1 kHz + 15 kHz")
    ax[0, 0].plot(y1, lw=1.6, label="output (fir1(28, 6/24))")
    ax[0, 0].plot(ref1, "k:", lw=1.2, label="MATLAB refOutput")
    ax[0, 0].set_title(f"Test 1: arm_fir_example, fs = 48 kHz\nSNR vs MATLAB: PC float64 {snr1:.1f} dB, board float32 142.6 dB",
                       fontsize=10)
    ax[0, 0].set_xlabel("sample n")
    ax[0, 0].legend(fontsize=8, loc="lower left")
    f1 = np.fft.rfftfreq(len(x1), 1 / fs1) / 1000
    ax[0, 1].plot(f1, spectrum(x1), label="|X(f)|")
    ax[0, 1].plot(f1, spectrum(y1), label="|Y(f)|")
    ax[0, 1].set_title("Test 1 spectrum: 15 kHz removed, 1 kHz kept")
    ax[0, 1].set_xlabel("frequency [kHz]")
    ax[0, 1].legend(fontsize=8)

    ax[1, 0].plot(n, x2, lw=0.9, label="input sin(1 Hz) + 0.5 sin(16 Hz)")
    ax[1, 0].plot(n, y2, lw=1.8, label="output y[n]")
    ax[1, 0].plot(n, ideal, "k:", lw=1.2, label="ideal |H(1Hz)| sin(1 Hz, delay 14)")
    ax[1, 0].axvspan(0, 32, color="0.9", label="start-up transient (skipped)")
    ax[1, 0].set_title(f"Test 2: accelerometer LPF, fs = 64 Hz   SNR = {snr2:.1f} dB")
    ax[1, 0].set_xlabel("sample n")
    ax[1, 0].legend(fontsize=8, loc="lower left")
    f2 = np.fft.rfftfreq(256, 1 / fs2)
    ax[1, 1].semilogy(f2, spectrum(x2[seg]) + 1e-6, label="input")
    ax[1, 1].semilogy(f2, spectrum(y2[seg]) + 1e-6, label="output")
    ax[1, 1].set_ylim(1e-5, 2)
    ax[1, 1].set_title("Test 2 spectrum (256-point FFT): 16 Hz attenuated -61 dB")
    ax[1, 1].set_xlabel("frequency [Hz]")
    ax[1, 1].legend(fontsize=8)

    for h, fs, fc, a, title in ((h1, fs1, 6000, ax[2, 0], "Test 1 filter"), (h2, fs2, 4, ax[2, 1], "accelerometer filter")):
        w, H = signal.freqz(h, worN=2048, fs=fs)
        scale = 1000 if fs > 1000 else 1
        a.plot(w / scale, 20 * np.log10(np.abs(H) + 1e-9))
        a.axvline(fc / scale, color="tab:red", ls="--", lw=1, label=f"fc = {fc / scale:g} {'kHz' if scale > 1 else 'Hz'}")
        a.set_ylim(-90, 5)
        a.set_ylabel("|H| [dB]")
        a.set_xlabel("frequency [kHz]" if scale > 1 else "frequency [Hz]")
        a.set_title(f"{title}: 29 taps, Hamming window")
        a.legend(fontsize=8)

    for a in ax.flat:
        a.grid(alpha=0.3)
    fig.tight_layout()
    out = ROOT / "known_signal_test.png"
    fig.savefig(out, dpi=130)
    print(f"Test 1 SNR = {snr1:.2f} dB, Test 2 SNR = {snr2:.2f} dB  -> {out.name}")


if __name__ == "__main__":
    main()
