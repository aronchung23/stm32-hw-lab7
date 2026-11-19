"""
HW-Lab7 optional problem: PC side (BLE GATT client + live plot)

The STM32 IoT node (lab07_dsp) advertises as "LAB7DSP" and notifies every
accelerometer sample before and after the CMSIS-DSP FIR low-pass filter:

  service a0070000-8e22-4541-9d4c-21edae82ed19
  DATA    a0070001-8e22-4541-9d4c-21edae82ed19   notify, 14 bytes little endian:
          uint16 seq, int16 raw_x, raw_y, raw_z, int16 filt_x, filt_y, filt_z   [mg]

Usage (Windows, Python with bleak + numpy + matplotlib, e.g. the "py" launcher):
  py lab07_client.py                       # live plot until the window is closed
  py lab07_client.py --save plot.png       # also save the last figure when closing
  py lab07_client.py --csv data.csv        # also log every sample to a CSV file
  py lab07_client.py --no-plot --duration 10   # text only (check the BLE link)
"""
import argparse
import asyncio
import collections
import csv
import struct
import sys
import threading
import time

from bleak import BleakClient, BleakScanner

DATA_UUID = "a0070001-8e22-4541-9d4c-21edae82ed19"
FS_HZ = 64            # sampling frequency on the board
FIR_DELAY = 14        # group delay of the 29-tap FIR (samples)


class Receiver:
    """BLE link running in a background thread with its own asyncio loop."""

    def __init__(self, name, address, window_s, csv_path):
        self.name = name
        self.address = address
        self.buf = collections.deque(maxlen=int(window_s * FS_HZ))
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.connected = threading.Event()
        self.status = "scanning..."
        self.received = 0
        self.lost = 0
        self.last_seq = None
        self.seq_base = 0
        self.t_first = None
        self.csv_file = open(csv_path, "w", newline="") if csv_path else None
        self.csv = csv.writer(self.csv_file) if self.csv_file else None
        if self.csv:
            self.csv.writerow(["seq", "t_s", "raw_x", "raw_y", "raw_z", "filt_x", "filt_y", "filt_z"])

    # ---- notification handler ----
    def on_notify(self, _sender, data: bytearray):
        if len(data) < 14:
            return
        seq16, rx, ry, rz, fx, fy, fz = struct.unpack("<H6h", bytes(data[:14]))
        with self.lock:
            # unwrap the 16-bit sequence number and count lost samples
            if self.last_seq is not None:
                step = (seq16 - self.last_seq) & 0xFFFF
                if step == 0 or step > 0x8000:
                    return                      # duplicate / out of order
                if step > 1:
                    self.lost += step - 1
                if seq16 < self.last_seq:
                    self.seq_base += 0x10000
            self.last_seq = seq16
            seq = self.seq_base + seq16
            if self.t_first is None:
                self.t_first = time.time()
            self.received += 1
            self.buf.append((seq, rx, ry, rz, fx, fy, fz))
        if self.csv:
            self.csv.writerow([seq, f"{seq / FS_HZ:.4f}", rx, ry, rz, fx, fy, fz])

    def snapshot(self):
        with self.lock:
            return list(self.buf)

    def rate(self):
        with self.lock:
            if not self.t_first:
                return 0.0
            dt = time.time() - self.t_first
            return self.received / dt if dt > 0 else 0.0

    # ---- BLE coroutine ----
    async def run_ble(self):
        if self.address:
            device = self.address
            self.status = f"connecting to {self.address}..."
        else:
            print(f"Scanning for '{self.name}' ...")
            device = None
            tries = 0
            # keep scanning: the board may still be flashing / resetting / running its self-tests
            while device is None and not self.stop.is_set():
                tries += 1
                self.status = f"scanning for '{self.name}' (try {tries}) ..."
                device = await BleakScanner.find_device_by_name(self.name, timeout=10.0)
                if device is None:
                    print(f"'{self.name}' not found yet (try {tries}), scanning again ... "
                          "(board powered? phone app disconnected? press RESET on the board)")
            if device is None:
                return
            print(f"Found {device.name} [{device.address}]")
            self.status = f"connecting to {device.address}..."

        def on_disconnect(_client):
            print("BLE link lost (disconnected by the device or the radio)")

        # do not use the Windows GATT cache: the board may have run another firmware before
        async with BleakClient(device, timeout=20.0, disconnected_callback=on_disconnect,
                               winrt=dict(use_cached_services=False)) as client:
            print("Connected, enabling notifications ...")
            await client.start_notify(DATA_UUID, self.on_notify)
            self.status = "connected"
            self.connected.set()
            while not self.stop.is_set() and client.is_connected:
                await asyncio.sleep(0.2)
            print(f"BLE loop ended: stop={self.stop.is_set()} is_connected={client.is_connected}")
            if client.is_connected:
                try:
                    await client.stop_notify(DATA_UUID)
                except Exception:
                    pass
        self.status = "disconnected"
        print("Disconnected")

    def thread_main(self):
        # reconnect automatically if the link drops (until the user stops the client)
        while not self.stop.is_set():
            try:
                asyncio.run(self.run_ble())
            except Exception as e:                  # show BLE errors instead of dying silently
                self.status = f"BLE error: {e}"
                print(self.status)
            if not self.stop.is_set():
                print("Reconnecting in 2 s ...")
                time.sleep(2.0)

    def close(self):
        if self.csv_file:
            self.csv_file.close()


def run_text(rx: Receiver, duration):
    t0 = time.time()
    last = 0
    while not rx.stop.is_set():
        time.sleep(1.0)
        data = rx.snapshot()
        if data:
            s = data[-1]
            print(f"t={s[0] / FS_HZ:7.2f}s  raw X={s[1]:5d} Y={s[2]:5d} Z={s[3]:5d}  |  "
                  f"filt X={s[4]:5d} Y={s[5]:5d} Z={s[6]:5d} mg   "
                  f"({rx.received - last} samples/s, lost {rx.lost})")
            last = rx.received
        if duration and time.time() - t0 > duration:
            break


def run_plot(rx: Receiver, window_s, save_path, duration=0):
    import numpy as np
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    fig = plt.figure(figsize=(11, 8))
    gs = fig.add_gridspec(4, 1, height_ratios=[1, 1, 1, 1.1], hspace=0.45)
    axes = [fig.add_subplot(gs[i, 0]) for i in range(3)]
    ax_f = fig.add_subplot(gs[3, 0])
    raw_lines, filt_lines = [], []
    for ax, name in zip(axes, "XYZ"):
        (lr,) = ax.plot([], [], color="tab:gray", lw=0.9, alpha=0.8, label="raw (before)")
        (lf,) = ax.plot([], [], color="tab:red", lw=1.8, label="FIR LPF (after)")
        raw_lines.append(lr)
        filt_lines.append(lf)
        ax.set_ylabel(f"{name} [mg]")
        ax.grid(alpha=0.3)
    axes[0].legend(loc="upper left", fontsize=8, ncol=2)
    axes[2].set_xlabel("time [s]  (filtered curve is delayed by 14 samples = 0.22 s)")
    (sr,) = ax_f.plot([], [], color="tab:gray", lw=1.0, label="raw")
    (sf,) = ax_f.plot([], [], color="tab:red", lw=1.6, label="filtered")
    ax_f.axvline(4.0, color="tab:blue", ls="--", lw=1, label="fc = 4 Hz")
    ax_f.set_xlim(0, FS_HZ / 2)
    ax_f.set_yscale("log")
    ax_f.set_ylim(1e-2, 1e2)
    ax_f.set_xlabel("frequency [Hz]")
    ax_f.set_ylabel("|FFT| of |a| [mg]")
    ax_f.grid(alpha=0.3, which="both")
    ax_f.legend(loc="upper right", fontsize=8)
    title = fig.suptitle("HW-Lab7  STM32 LSM6DSL -> CMSIS-DSP FIR LPF -> BLE", fontsize=12)

    def update(_frame):
        data = rx.snapshot()
        if len(data) < 2:
            title.set_text(f"HW-Lab7  status: {rx.status}")
            return []
        a = np.array(data, dtype=float)
        t = a[:, 0] / FS_HZ
        for i in range(3):
            raw_lines[i].set_data(t, a[:, 1 + i])
            filt_lines[i].set_data(t, a[:, 4 + i])
            ax = axes[i]
            lo = min(a[:, 1 + i].min(), a[:, 4 + i].min())
            hi = max(a[:, 1 + i].max(), a[:, 4 + i].max())
            pad = max(20.0, 0.1 * (hi - lo))
            ax.set_ylim(lo - pad, hi + pad)
            ax.set_xlim(max(t[-1] - window_s, t[0]), max(t[-1], t[0] + 1))
        # spectrum of the acceleration magnitude (last 256 samples, DC removed)
        n = min(256, len(a))
        if n >= 64:
            win = np.hanning(n)
            for line, cols in ((sr, slice(1, 4)), (sf, slice(4, 7))):
                mag = np.linalg.norm(a[-n:, cols], axis=1)
                spec = 2 * np.abs(np.fft.rfft((mag - mag.mean()) * win)) / win.sum()
                line.set_data(np.fft.rfftfreq(n, 1 / FS_HZ), spec + 1e-3)
            ax_f.set_ylim(1e-2, max(10.0, sr.get_ydata().max() * 2))
        title.set_text(f"HW-Lab7  STM32 LSM6DSL -> CMSIS-DSP FIR LPF (fc 4 Hz) -> BLE   |   "
                       f"{rx.status}, {rx.rate():.1f} samples/s, lost {rx.lost}")
        return []

    anim = FuncAnimation(fig, update, interval=200, cache_frame_data=False)  # noqa: F841
    if duration:
        closer = fig.canvas.new_timer(interval=int(duration * 1000))
        closer.single_shot = True
        closer.add_callback(plt.close, fig)
        closer.start()
    try:
        plt.show()
    finally:
        if save_path:
            update(0)
            fig.savefig(save_path, dpi=130)
            print(f"Figure saved to {save_path}")


def main():
    p = argparse.ArgumentParser(description="HW-Lab7 BLE client: raw vs CMSIS-DSP filtered accelerometer")
    p.add_argument("--name", default="LAB7DSP", help="BLE device name (default LAB7DSP)")
    p.add_argument("--address", help="connect to this BLE address directly (skip scanning)")
    p.add_argument("--window", type=float, default=10.0, help="seconds shown in the plot (default 10)")
    p.add_argument("--duration", type=float, default=0, help="stop after N seconds (0 = until Ctrl+C / window closed)")
    p.add_argument("--no-plot", action="store_true", help="text output only")
    p.add_argument("--csv", help="log every sample to this CSV file")
    p.add_argument("--save", help="save the figure to this PNG when the window is closed")
    args = p.parse_args()

    rx = Receiver(args.name, args.address, args.window, args.csv)
    th = threading.Thread(target=rx.thread_main, daemon=True)
    th.start()
    try:
        if args.no_plot:
            run_text(rx, args.duration)
        else:
            run_plot(rx, args.window, args.save, args.duration)
    except KeyboardInterrupt:
        pass
    finally:
        rx.stop.set()
        th.join(timeout=5)
        rx.close()
        print(f"received {rx.received} samples, lost {rx.lost}")


if __name__ == "__main__":
    sys.exit(main())
