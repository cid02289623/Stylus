# pip install pyserial
#
# Comprehensive gesture data logger for PIC18 Stylus project.
#
# Collects every measurable quantity from the UART stream and writes
# two output files per session:
#
#   data_log_<timestamp>.txt   — human-readable line-by-line log
#                                (same format as before, with timestamps)
#
#   data_<timestamp>.csv       — one row per GESTURE EVENT with every
#                                derived quantity ready for analysis:
#
#       session_id, gesture_id, result,
#       bias_gx, bias_gz,
#       acc_v, acc_h,
#       peak_v_pos, peak_v_neg, peak_h_pos, peak_h_neg,
#       mag_v, mag_h,                        <- max(pos,neg) per axis
#       flags,
#       n_frames,                            <- btn=1 frames in window
#       duration_ms,                         <- button held time (ms)
#       latency_ms,                          <- button release to PIC line (ms)
#       t_release_ms,                        <- absolute timestamp of release
#       t_pic_ms,                            <- absolute timestamp of PIC line
#       peak_gx, peak_gz,                    <- raw sensor peak magnitudes
#       mean_gx, mean_gz,                    <- mean raw sensor during capture
#       std_gx, std_gz,                      <- std dev of raw sensor
#       max_abs_gx, max_abs_gz,              <- max |gx|, |gz| during capture
#       frame_gx_<n>, frame_gz_<n>           <- per-frame raw gx/gz (up to 30)
#                                               for time-series / velocity trace
#
# Usage:
#   python3 data_logger.py
#
# Stop with Ctrl-C.  Files are flushed continuously so no data is lost.

import csv
import math
import os
import serial
import time
from datetime import datetime

# -------------------------------------------------
# User config
# -------------------------------------------------
PORT  = "COM5"       # change if needed
BAUD  = 38400
MAX_FRAMES_PER_GESTURE = 30   # columns in CSV for per-frame data

# -------------------------------------------------
# Protocol
# -------------------------------------------------
RAW_SOF  = 0xAA
DBG_SOF  = 0xE2
BIAS_SOF = 0xB1

RAW_LEN  = 9
DBG_LEN  = 31
BIAS_LEN = 6

# -------------------------------------------------
# Helpers
# -------------------------------------------------
def s16(lo, hi):
    v = lo | (hi << 8)
    return v - 0x10000 if v & 0x8000 else v

def s32(b0, b1, b2, b3):
    v = b0 | (b1 << 8) | (b2 << 16) | (b3 << 24)
    return v - 0x100000000 if v & 0x80000000 else v

def decode_result(b):
    return chr(b) if 32 <= b <= 126 else f"0x{b:02X}"

def flag_text(flags):
    names = []
    if flags & 0x01: names.append("BTN")
    if flags & 0x02: names.append("V>TH")
    if flags & 0x04: names.append("H>TH")
    if flags & 0x08: names.append("VDOM")
    if flags & 0x10: names.append("HDOM")
    if flags & 0x20: names.append("CHOSE_V")
    if flags & 0x40: names.append("CHOSE_H")
    return "|".join(names) if names else "NONE"

def safe_std(vals):
    if len(vals) < 2:
        return 0.0
    m = sum(vals) / len(vals)
    return math.sqrt(sum((x - m) ** 2 for x in vals) / (len(vals) - 1))

# -------------------------------------------------
# CSV column definitions
# -------------------------------------------------
STATIC_COLS = [
    "session_id", "gesture_id", "result",
    "bias_gx", "bias_gz",
    "proc_gx", "proc_gy", "proc_gz", "proc_btn",
    "acc_v", "acc_h",
    "peak_v_pos", "peak_v_neg", "peak_h_pos", "peak_h_neg",
    "mag_v", "mag_h",
    "flags", "flags_text",
    "n_frames",
    "duration_ms",
    "latency_ms",
    "t_press_ms",
    "t_release_ms",
    "t_pic_ms",
    "peak_raw_gx", "peak_raw_gz",
    "mean_gx", "mean_gz",
    "std_gx", "std_gz",
    "max_abs_gx", "max_abs_gz",
    "min_gx", "max_gx",
    "min_gz", "max_gz",
    "mean_gy", "std_gy",
    "cal_bias_computed_gx", "cal_bias_computed_gz",  # from bias frame
]

# Per-frame columns: gx and gz for each frame slot
FRAME_COLS = []
for i in range(MAX_FRAMES_PER_GESTURE):
    FRAME_COLS.append(f"frame_{i:02d}_gx")
    FRAME_COLS.append(f"frame_{i:02d}_gy")
    FRAME_COLS.append(f"frame_{i:02d}_gz")
    FRAME_COLS.append(f"frame_{i:02d}_t_ms")   # timestamp relative to press

ALL_COLS = STATIC_COLS + FRAME_COLS


# -------------------------------------------------
# Logger state
# -------------------------------------------------
class DataLogger:
    def __init__(self, port, baud):
        self.t0 = time.monotonic()
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.session_id = ts

        self.log_path = f"data_log_{ts}.txt"
        self.csv_path = f"data_{ts}.csv"

        self.log_f = open(self.log_path, "w", buffering=1, encoding="utf-8")
        self.csv_f = open(self.csv_path, "w", newline="", encoding="utf-8")
        self.csv_w = csv.DictWriter(self.csv_f, fieldnames=ALL_COLS)
        self.csv_w.writeheader()
        self.csv_f.flush()

        # Calibration bias (from B1 frame)
        self.cal_bias_gx = None
        self.cal_bias_gz = None

        # Gesture window tracking
        self.gesture_id      = 0
        self.in_gesture      = False
        self.gesture_frames  = []   # list of (t_ms, gx, gy, gz) during btn=1
        self.t_press_ms      = None
        self.t_release_ms    = None
        self.pending_release = False  # waiting for PIC line after release

        # Counts per result for progress display
        self.counts = {"U": 0, "D": 0, "L": 0, "R": 0, "?": 0}

        # Serial buffer
        self._buf = bytearray()

        # Open serial
        self.ser = None
        try:
            self.ser = serial.Serial(port, baud, timeout=0.05)
            time.sleep(2)
            self._log(f"SESSION_START port={port} baud={baud} "
                      f"log={self.log_path} csv={self.csv_path}")
            print(f"Logging to {self.log_path} and {self.csv_path}")
            print("Perform gestures.  Ctrl-C to stop.\n")
            print(f"{'Result':>8} {'U':>6} {'D':>6} {'L':>6} {'R':>6} {'?':>6}")
            print("-" * 45)
        except Exception as e:
            print(f"Serial error: {e}")
            self._log(f"SERIAL_ERROR {e}")

    # --------------------------------------------------
    def close(self):
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass
        try:
            self.log_f.close()
        except Exception:
            pass
        try:
            self.csv_f.close()
        except Exception:
            pass

    def _ms(self):
        return (time.monotonic() - self.t0) * 1000.0

    def _log(self, line):
        stamped = f"[{self._ms():10.1f}] {line}"
        print(stamped)
        self.log_f.write(stamped + "\n")

    # --------------------------------------------------
    def poll(self):
        if self.ser is None:
            return
        try:
            waiting = self.ser.in_waiting
            if waiting:
                self._buf.extend(self.ser.read(waiting))
        except Exception as e:
            self._log(f"SERIAL_READ_ERROR {e}")
            return

        if len(self._buf) > 8192:
            self._buf = self._buf[-256:]

        SIZES    = {RAW_SOF: RAW_LEN,  BIAS_SOF: BIAS_LEN,  DBG_SOF: DBG_LEN}
        CHK_SPAN = {RAW_SOF: (1, 8, 8), BIAS_SOF: (1, 5, 5), DBG_SOF: (1, 30, 30)}

        while self._buf:
            sof = self._buf[0]
            if sof not in SIZES:
                del self._buf[0]
                continue
            needed = SIZES[sof]
            if len(self._buf) < needed:
                break
            frame = bytes(self._buf[:needed])

            # Validate checksum BEFORE consuming.
            # On failure: discard only the SOF byte and rescan.
            # This means the real next SOF (which may lie inside
            # the current frame bytes) will be found on the next
            # iteration, rather than the whole frame being thrown away.
            lo, hi, ci = CHK_SPAN[sof]
            if sum(frame[lo:hi]) & 0xFF != frame[ci]:
                self._log(f"BAD_FRAME {' '.join(f'{x:02X}' for x in frame)}")
                del self._buf[0]
                continue

            del self._buf[:needed]
            if   sof == RAW_SOF:  self._handle_raw(frame)
            elif sof == BIAS_SOF: self._handle_bias(frame)
            elif sof == DBG_SOF:  self._handle_dbg(frame)

    # --------------------------------------------------
    def _handle_raw(self, frame):
        t   = self._ms()
        gx  = s16(frame[1], frame[2])
        gy  = s16(frame[3], frame[4])
        gz  = s16(frame[5], frame[6])
        btn = frame[7]

        self._log(
            f"RAW  bytes={' '.join(f'{x:02X}' for x in frame)}  "
            f"gx={gx} gy={gy} gz={gz} btn=0x{btn:02X}"
        )

        btn_pressed = bool(btn & 0x01)

        if btn_pressed and not self.in_gesture:
            # Rising edge
            self.in_gesture     = True
            self.gesture_frames = [(t, gx, gy, gz)]
            self.t_press_ms     = t
            self.t_release_ms   = None
            self.pending_release = False

        elif btn_pressed and self.in_gesture:
            self.gesture_frames.append((t, gx, gy, gz))

        elif not btn_pressed and self.in_gesture:
            # Falling edge
            self.in_gesture      = False
            self.t_release_ms    = t
            self.pending_release = True
            # Record the falling-edge frame too (this is what proc_* will be)
            self.gesture_frames.append((t, gx, gy, gz))

    # --------------------------------------------------
    def _handle_bias(self, frame):
        payload = frame[1:5]
        self.cal_bias_gx = s16(payload[0], payload[1])
        self.cal_bias_gz = s16(payload[2], payload[3])
        self._log(f"BIAS bias_gx={self.cal_bias_gx} bias_gz={self.cal_bias_gz}")

    # --------------------------------------------------
    def _handle_dbg(self, frame):
        payload = frame[1:30]
        t_pic = self._ms()

        proc_gx  = s16(payload[0],  payload[1])
        proc_gy  = s16(payload[2],  payload[3])
        proc_gz  = s16(payload[4],  payload[5])
        proc_btn = payload[6]
        bias_gx  = s16(payload[7],  payload[8])
        bias_gz  = s16(payload[9],  payload[10])
        acc_v    = s32(payload[11], payload[12], payload[13], payload[14])
        acc_h    = s32(payload[15], payload[16], payload[17], payload[18])
        pv_pos   = s16(payload[19], payload[20])
        pv_neg   = s16(payload[21], payload[22])
        ph_pos   = s16(payload[23], payload[24])
        ph_neg   = s16(payload[25], payload[26])
        flags    = payload[27]
        result   = decode_result(payload[28])

        mag_v = max(pv_pos, pv_neg)
        mag_h = max(ph_pos, ph_neg)

        self._log(
            f"PIC  proc_gx={proc_gx} proc_gy={proc_gy} proc_gz={proc_gz} "
            f"proc_btn=0x{proc_btn:02X}  "
            f"bias_gx={bias_gx} bias_gz={bias_gz}  "
            f"acc_v={acc_v} acc_h={acc_h}  "
            f"peak_v_pos={pv_pos} peak_v_neg={pv_neg}  "
            f"peak_h_pos={ph_pos} peak_h_neg={ph_neg}  "
            f"flags=0x{flags:02X}({flag_text(flags)})  "
            f"result={result}"
        )

        # Only write CSV row if this follows a gesture window we tracked
        if not self.pending_release:
            return

        self.pending_release = False
        self.gesture_id += 1

        # Separate btn=1 frames from the trailing btn=0 frame
        # (last frame in gesture_frames is the falling-edge btn=0)
        if self.gesture_frames and self.gesture_frames[-1][3] == 0:
            capture_frames = self.gesture_frames[:-1]
        else:
            capture_frames = self.gesture_frames

        n_frames = len(capture_frames)

        # Time series — relative to press
        t_press = self.t_press_ms or t_pic

        # Per-frame gx, gy, gz, relative timestamp
        gx_vals = [f[1] for f in capture_frames]
        gy_vals = [f[2] for f in capture_frames]
        gz_vals = [f[3] for f in capture_frames]
        t_vals  = [f[0] - t_press for f in capture_frames]

        # Derived statistics
        mean_gx    = sum(gx_vals) / len(gx_vals) if gx_vals else 0.0
        mean_gz    = sum(gz_vals) / len(gz_vals) if gz_vals else 0.0
        mean_gy    = sum(gy_vals) / len(gy_vals) if gy_vals else 0.0
        std_gx     = safe_std(gx_vals)
        std_gz     = safe_std(gz_vals)
        std_gy     = safe_std(gy_vals)
        peak_raw_gx = max((abs(x) for x in gx_vals), default=0)
        peak_raw_gz = max((abs(x) for x in gz_vals), default=0)
        max_abs_gx  = peak_raw_gx
        max_abs_gz  = peak_raw_gz
        min_gx      = min(gx_vals, default=0)
        max_gx      = max(gx_vals, default=0)
        min_gz      = min(gz_vals, default=0)
        max_gz      = max(gz_vals, default=0)

        duration_ms = ((self.t_release_ms or t_pic) - t_press)
        latency_ms  = t_pic - (self.t_release_ms or t_pic)

        # Build CSV row
        row = {
            "session_id":            self.session_id,
            "gesture_id":            self.gesture_id,
            "result":                result,
            "bias_gx":               bias_gx,
            "bias_gz":               bias_gz,
            "proc_gx":               proc_gx,
            "proc_gy":               proc_gy,
            "proc_gz":               proc_gz,
            "proc_btn":              proc_btn,
            "acc_v":                 acc_v,
            "acc_h":                 acc_h,
            "peak_v_pos":            pv_pos,
            "peak_v_neg":            pv_neg,
            "peak_h_pos":            ph_pos,
            "peak_h_neg":            ph_neg,
            "mag_v":                 mag_v,
            "mag_h":                 mag_h,
            "flags":                 f"0x{flags:02X}",
            "flags_text":            flag_text(flags),
            "n_frames":              n_frames,
            "duration_ms":           round(duration_ms, 2),
            "latency_ms":            round(latency_ms, 2),
            "t_press_ms":            round(t_press, 2),
            "t_release_ms":          round(self.t_release_ms or t_pic, 2),
            "t_pic_ms":              round(t_pic, 2),
            "peak_raw_gx":           peak_raw_gx,
            "peak_raw_gz":           peak_raw_gz,
            "mean_gx":               round(mean_gx, 2),
            "mean_gz":               round(mean_gz, 2),
            "std_gx":                round(std_gx, 2),
            "std_gz":                round(std_gz, 2),
            "max_abs_gx":            max_abs_gx,
            "max_abs_gz":            max_abs_gz,
            "min_gx":                min_gx,
            "max_gx":                max_gx,
            "min_gz":                min_gz,
            "max_gz":                max_gz,
            "mean_gy":               round(mean_gy, 2),
            "std_gy":                round(std_gy, 2),
            "cal_bias_computed_gx":  self.cal_bias_gx if self.cal_bias_gx is not None else "",
            "cal_bias_computed_gz":  self.cal_bias_gz if self.cal_bias_gz is not None else "",
        }

        # Per-frame columns (pad with empty string if fewer than MAX_FRAMES)
        for i in range(MAX_FRAMES_PER_GESTURE):
            if i < len(capture_frames):
                row[f"frame_{i:02d}_gx"]   = capture_frames[i][1]
                row[f"frame_{i:02d}_gy"]   = capture_frames[i][2]
                row[f"frame_{i:02d}_gz"]   = capture_frames[i][3]
                row[f"frame_{i:02d}_t_ms"] = round(t_vals[i], 2)
            else:
                row[f"frame_{i:02d}_gx"]   = ""
                row[f"frame_{i:02d}_gy"]   = ""
                row[f"frame_{i:02d}_gz"]   = ""
                row[f"frame_{i:02d}_t_ms"] = ""

        self.csv_w.writerow(row)
        self.csv_f.flush()

        # Update counts and print progress
        r = result if result in self.counts else "?"
        self.counts[r] = self.counts.get(r, 0) + 1
        total = sum(self.counts.values())
        print(
            f"\r  #{total:4d}   "
            f"U={self.counts['U']:4d}  "
            f"D={self.counts['D']:4d}  "
            f"L={self.counts['L']:4d}  "
            f"R={self.counts['R']:4d}  "
            f"?={self.counts['?']:4d}   "
            f"last={result}",
            end="", flush=True
        )

        # Clear tracked window
        self.gesture_frames = []
        self.t_press_ms     = None
        self.t_release_ms   = None


# -------------------------------------------------
# Main
# -------------------------------------------------
def main():
    logger = DataLogger(PORT, BAUD)

    try:
        while True:
            logger.poll()
            time.sleep(0.005)   # 5ms sleep — responsive without burning CPU

    except KeyboardInterrupt:
        print("\n\nStopped.")
        counts = logger.counts
        total  = sum(counts.values())
        print(f"\nSession summary:")
        print(f"  Total gestures : {total}")
        for r in ("U", "D", "L", "R", "?"):
            print(f"  {r:>8}        : {counts.get(r,0)}")
        print(f"\nFiles written:")
        print(f"  {logger.log_path}")
        print(f"  {logger.csv_path}")

    finally:
        logger.close()


if __name__ == "__main__":
    main()