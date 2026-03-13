import argparse
import csv
import math
import random
import serial
import time

MARKER_TX = 0xAA
BIAS_SOF = 0xB1
CAPTURE_SOF = 0xD1


def int_auto(text):
    return int(text, 0)


def build_frame(gx, gy, gz, btn):
    """
    Packet sent to the PIC:

      byte 0 = 0xAA
      byte 1 = Gx low
      byte 2 = Gx high
      byte 3 = Gy low
      byte 4 = Gy high
      byte 5 = Gz low
      byte 6 = Gz high
      byte 7 = button
      byte 8 = checksum over bytes 1..7 mod 256
    """
    gx &= 0xFFFF
    gy &= 0xFFFF
    gz &= 0xFFFF

    payload = [
        gx & 0xFF,
        (gx >> 8) & 0xFF,
        gy & 0xFF,
        (gy >> 8) & 0xFF,
        gz & 0xFF,
        (gz >> 8) & 0xFF,
        btn & 0xFF,
    ]
    checksum = sum(payload) & 0xFF
    return bytes([MARKER_TX] + payload + [checksum])


def noisy_value(base, noise_amplitude):
    if noise_amplitude <= 0:
        return base
    return base + random.randint(-noise_amplitude, noise_amplitude)


def to_signed16(lo, hi):
    value = lo | (hi << 8)
    if value & 0x8000:
        value -= 0x10000
    return value


def try_read_pic_message(ser, result_codes, verbose_rx=False):
    """
    Read one message from the PIC TX stream.

    Supported messages:
      - one-byte classifier result
      - bias frame:
          0xB1, bias_gx_l, bias_gx_h, bias_gy_l, bias_gy_h, checksum
      - capture report frame:
          0xD1, dx_l, dx_h, dy_l, dy_h, checksum
    """
    b = ser.read(1)
    if not b:
        return None

    value = b[0]

    if value in result_codes:
        return {
            "kind": "result",
            "code": value,
            "label": result_codes[value],
            "time": time.perf_counter(),
        }

    if value == BIAS_SOF:
        rest = ser.read(5)
        if len(rest) != 5:
            if verbose_rx:
                print("RX incomplete bias frame")
            return None

        gx_l, gx_h, gy_l, gy_h, checksum = rest
        calc = (gx_l + gx_h + gy_l + gy_h) & 0xFF
        if calc != checksum:
            if verbose_rx:
                print(
                    f"RX bad bias checksum: got 0x{checksum:02X}, expected 0x{calc:02X}"
                )
            return None

        return {
            "kind": "bias",
            "gx": to_signed16(gx_l, gx_h),
            "gy": to_signed16(gy_l, gy_h),
            "time": time.perf_counter(),
        }

    if value == CAPTURE_SOF:
        rest = ser.read(5)
        if len(rest) != 5:
            if verbose_rx:
                print("RX incomplete capture frame")
            return None

        dx_l, dx_h, dy_l, dy_h, checksum = rest
        calc = (dx_l + dx_h + dy_l + dy_h) & 0xFF
        if calc != checksum:
            if verbose_rx:
                print(
                    f"RX bad capture checksum: got 0x{checksum:02X}, expected 0x{calc:02X}"
                )
            return None

        dx = to_signed16(dx_l, dx_h)
        dy = to_signed16(dy_l, dy_h)

        return {
            "kind": "capture",
            "dx": dx,
            "dy": dy,
            "abs_dx": abs(dx),
            "abs_dy": abs(dy),
            "envelope": max(abs(dx), abs(dy)),
            "time": time.perf_counter(),
        }

    if verbose_rx:
        print(f"RX ignored byte: 0x{value:02X}")

    return None


def handle_rx_message(pkt, state, show_result_bytes):
    if pkt is None:
        return

    if pkt["kind"] == "bias":
        state["last_bias"] = pkt
        print(f"PIC learned bias: Gx={pkt['gx']}  Gy={pkt['gy']}")
        return

    if pkt["kind"] == "capture":
        state["last_capture"] = pkt
        print(
            "PIC capture report: "
            f"Dx={pkt['dx']:6d}  Dy={pkt['dy']:6d}  "
            f"|Dx|={pkt['abs_dx']:6d}  |Dy|={pkt['abs_dy']:6d}  "
            f"envelope={pkt['envelope']:6d}"
        )
        return

    if pkt["kind"] == "result":
        state["last_result"] = pkt
        if show_result_bytes:
            print(f"PIC result byte: {pkt['label']} (0x{pkt['code']:02X})")


def pump_rx_until(ser, deadline, result_codes, state, verbose_rx, show_result_bytes):
    while time.perf_counter() < deadline:
        pkt = try_read_pic_message(ser, result_codes, verbose_rx=verbose_rx)
        if pkt is None:
            continue
        handle_rx_message(pkt, state, show_result_bytes)


def drain_rx(ser, result_codes, duration_s, verbose_rx=False, show_result_bytes=False):
    state = {"last_bias": None, "last_capture": None, "last_result": None}
    pump_rx_until(
        ser=ser,
        deadline=time.perf_counter() + duration_s,
        result_codes=result_codes,
        state=state,
        verbose_rx=verbose_rx,
        show_result_bytes=show_result_bytes,
    )
    return state


def send_segment(
    ser,
    segment_name,
    frames,
    gx,
    gy,
    gz,
    btn,
    rate_hz,
    result_codes,
    verbose_rx,
    global_frame_index,
    noise_gx,
    noise_gy,
    noise_gz,
    state,
    show_result_bytes,
):
    period = 1.0 / rate_hz

    for i in range(frames):
        gx_i = noisy_value(gx, noise_gx)
        gy_i = noisy_value(gy, noise_gy)
        gz_i = noisy_value(gz, noise_gz)

        frame = build_frame(gx_i, gy_i, gz_i, btn)
        t0 = time.perf_counter()
        ser.write(frame)

        print(
            f"TX frame {global_frame_index + i + 1:4d}: "
            f"{segment_name:18s} "
            f"gx={gx_i:6d} gy={gy_i:6d} gz={gz_i:6d} btn=0x{btn:02X}"
        )

        pump_rx_until(
            ser=ser,
            deadline=t0 + period,
            result_codes=result_codes,
            state=state,
            verbose_rx=verbose_rx,
            show_result_bytes=show_result_bytes,
        )

    return global_frame_index + frames


def run_no_motion_trial(ser, trial_index, args, result_codes, global_frame_index):
    print(f"\n=== Phase 6 pilot trial {trial_index}/{args.trials} ===")
    print(
        "Goal: send only the learned stillness level during capture, "
        "so the final Dx/Dy should stay small."
    )

    state = {"last_bias": None, "last_capture": None, "last_result": None}
    release_time = None

    segments = [
        ("idle-before", args.idle_frames, args.bias_gx, args.bias_gy, args.bias_gz, 0x00),
        ("press-settle", args.press_frames, args.bias_gx, args.bias_gy, args.bias_gz, args.button_mask),
        ("capture-hold", args.capture_frames, args.bias_gx, args.bias_gy, args.bias_gz, args.button_mask),
        ("hold-still", args.hold_frames, args.bias_gx, args.bias_gy, args.bias_gz, args.button_mask),
        ("release", 1, args.bias_gx, args.bias_gy, args.bias_gz, 0x00),
        ("idle-after", args.idle_frames, args.bias_gx, args.bias_gy, args.bias_gz, 0x00),
    ]

    for segment_name, frames, gx, gy, gz, btn in segments:
        global_frame_index = send_segment(
            ser=ser,
            segment_name=segment_name,
            frames=frames,
            gx=gx,
            gy=gy,
            gz=gz,
            btn=btn,
            rate_hz=args.rate,
            result_codes=result_codes,
            verbose_rx=args.verbose_rx,
            global_frame_index=global_frame_index,
            noise_gx=args.trial_noise_gx,
            noise_gy=args.trial_noise_gy,
            noise_gz=args.trial_noise_gz,
            state=state,
            show_result_bytes=args.show_result_bytes,
        )

        if segment_name == "release":
            release_time = time.perf_counter()

    timeout_end = time.perf_counter() + (args.capture_timeout_ms / 1000.0)
    while state["last_capture"] is None and time.perf_counter() < timeout_end:
        pkt = try_read_pic_message(ser, result_codes, verbose_rx=args.verbose_rx)
        if pkt is None:
            continue
        handle_rx_message(pkt, state, args.show_result_bytes)

    row = {
        "trial_index": trial_index,
        "dx": None,
        "dy": None,
        "abs_dx": None,
        "abs_dy": None,
        "envelope": None,
        "capture_latency_ms": None,
    }

    if state["last_capture"] is not None:
        pkt = state["last_capture"]
        row["dx"] = pkt["dx"]
        row["dy"] = pkt["dy"]
        row["abs_dx"] = pkt["abs_dx"]
        row["abs_dy"] = pkt["abs_dy"]
        row["envelope"] = pkt["envelope"]

        if release_time is not None:
            row["capture_latency_ms"] = (pkt["time"] - release_time) * 1000.0

        print(
            "Trial summary: "
            f"Dx={row['dx']}  Dy={row['dy']}  "
            f"|Dx|={row['abs_dx']}  |Dy|={row['abs_dy']}  "
            f"envelope={row['envelope']}"
            + (
                f"  report_latency_ms={row['capture_latency_ms']:.1f}"
                if row["capture_latency_ms"] is not None
                else ""
            )
        )
    else:
        print("Trial summary: NO capture report received")

    return row, global_frame_index


def write_csv(csv_path, rows, learned_bias):
    with open(csv_path, "w", newline="") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "trial_index",
                "learned_bias_gx",
                "learned_bias_gy",
                "dx",
                "dy",
                "abs_dx",
                "abs_dy",
                "envelope",
                "capture_latency_ms",
            ],
        )
        writer.writeheader()
        for r in rows:
            writer.writerow(
                {
                    "trial_index": r["trial_index"],
                    "learned_bias_gx": None if learned_bias is None else learned_bias["gx"],
                    "learned_bias_gy": None if learned_bias is None else learned_bias["gy"],
                    "dx": r["dx"],
                    "dy": r["dy"],
                    "abs_dx": r["abs_dx"],
                    "abs_dy": r["abs_dy"],
                    "envelope": r["envelope"],
                    "capture_latency_ms": r["capture_latency_ms"],
                }
            )


def print_threshold_recommendation(rows, margin):
    valid_rows = [r for r in rows if r["envelope"] is not None]
    if not valid_rows:
        print("\nNo valid capture reports were received, so no Tdisp recommendation can be computed.")
        return

    max_abs_dx = max(r["abs_dx"] for r in valid_rows)
    max_abs_dy = max(r["abs_dy"] for r in valid_rows)
    max_envelope = max(r["envelope"] for r in valid_rows)

    recommended = int(math.ceil(max_envelope * margin))
    recommended &= 0xFFFF

    tdisp_h = (recommended >> 8) & 0xFF
    tdisp_l = recommended & 0xFF

    print("\n=== Phase 6 no-motion envelope summary ===")
    print(f"Valid pilot trials: {len(valid_rows)}")
    print(f"max |Dx| observed: {max_abs_dx}")
    print(f"max |Dy| observed: {max_abs_dy}")
    print(f"max envelope observed = {max_envelope}")
    print(f"Recommended threshold margin: {margin:.3f}")
    print(f"Recommended Tdisp = ceil({margin:.3f} * {max_envelope}) = {recommended}")
    print("\nPut these into gesture.s:")
    print(f"TDISP_H         equ 0x{tdisp_h:02X}")
    print(f"TDISP_L         equ 0x{tdisp_l:02X}")


def main():
    ap = argparse.ArgumentParser(
        description="Phase 6 pilot tester: startup bias calibration + end-of-capture Dx/Dy logging"
    )

    ap.add_argument("--port", default="COM5", help="Serial port, e.g. COM5")
    ap.add_argument("--baud", type=int, default=115200, help="Baud rate")
    ap.add_argument("--rate", type=float, default=50.0, help="Frame send rate in Hz")

    ap.add_argument("--bias-gx", type=int, default=120, help="Synthetic stillness bias on Gx")
    ap.add_argument("--bias-gy", type=int, default=-80, help="Synthetic stillness bias on Gy")
    ap.add_argument("--bias-gz", type=int, default=0, help="Synthetic stillness bias on Gz")

    ap.add_argument(
        "--startup-frames",
        type=int,
        default=75,
        help="Number of startup stillness frames sent for calibration",
    )
    ap.add_argument(
        "--arm-delay-ms",
        type=float,
        default=3000.0,
        help="Delay after opening COM port before startup stream begins; reset PIC during this time",
    )

    ap.add_argument("--trials", type=int, default=12, help="Number of no-motion pilot capture trials")
    ap.add_argument("--press-frames", type=int, default=2)
    ap.add_argument("--capture-frames", type=int, default=8)
    ap.add_argument("--hold-frames", type=int, default=1)
    ap.add_argument("--idle-frames", type=int, default=3)

    ap.add_argument("--button-mask", type=int_auto, default=0x01)
    ap.add_argument("--capture-timeout-ms", type=float, default=400.0)
    ap.add_argument("--inter-trial-gap-ms", type=float, default=100.0)

    ap.add_argument(
        "--threshold-margin",
        type=float,
        default=1.2,
        help="Recommended Tdisp = ceil(threshold_margin * max_envelope)",
    )

    ap.add_argument("--csv", default="", help="Optional CSV output file")
    ap.add_argument("--verbose-rx", action="store_true", help="Print ignored RX bytes")
    ap.add_argument(
        "--show-result-bytes",
        action="store_true",
        help="Also print the normal one-byte classifier outputs",
    )

    ap.add_argument("--seed", type=int, default=12345)

    ap.add_argument("--startup-noise-gx", type=int, default=0)
    ap.add_argument("--startup-noise-gy", type=int, default=0)
    ap.add_argument("--startup-noise-gz", type=int, default=0)

    ap.add_argument("--trial-noise-gx", type=int, default=0)
    ap.add_argument("--trial-noise-gy", type=int, default=0)
    ap.add_argument("--trial-noise-gz", type=int, default=0)

    ap.add_argument("--code-unknown", type=int_auto, default=0x3F)
    ap.add_argument("--code-left", type=int_auto, default=0x4C)
    ap.add_argument("--code-right", type=int_auto, default=0x52)
    ap.add_argument("--code-up", type=int_auto, default=0x55)
    ap.add_argument("--code-down", type=int_auto, default=0x44)

    args = ap.parse_args()
    random.seed(args.seed)

    result_codes = {
        args.code_unknown: "UNKNOWN",
        args.code_left: "LEFT",
        args.code_right: "RIGHT",
        args.code_up: "UP",
        args.code_down: "DOWN",
    }

    with serial.Serial(args.port, args.baud, timeout=0.002) as ser:
        print(f"Opened {args.port} at {args.baud}")
        print("Phase 6 pilot objective:")
        print("  1) let the PIC learn startup Gx/Gy bias from still samples")
        print("  2) run repeated no-motion capture trials")
        print("  3) log final Dx/Dy from the PIC at each release")
        print("  4) recommend Tdisp from the measured no-motion envelope")
        print(
            "Synthetic startup stillness mean values: "
            f"Gx={args.bias_gx}, Gy={args.bias_gy}, Gz={args.bias_gz}"
        )
        print(
            "Startup noise amplitudes: "
            f"Gx=±{args.startup_noise_gx}, "
            f"Gy=±{args.startup_noise_gy}, "
            f"Gz=±{args.startup_noise_gz}"
        )
        print(
            "Pilot no-motion noise amplitudes: "
            f"Gx=±{args.trial_noise_gx}, "
            f"Gy=±{args.trial_noise_gy}, "
            f"Gz=±{args.trial_noise_gz}"
        )
        print(
            f"Port armed. Reset the PIC now. Waiting {args.arm_delay_ms:.0f} ms "
            "before startup stillness frames begin..."
        )

        drain_rx(
            ser,
            result_codes=result_codes,
            duration_s=0.2,
            verbose_rx=args.verbose_rx,
            show_result_bytes=args.show_result_bytes,
        )
        time.sleep(args.arm_delay_ms / 1000.0)

        startup_state = {"last_bias": None, "last_capture": None, "last_result": None}

        print("\n=== Startup stillness stream begins ===")
        global_frame_index = send_segment(
            ser=ser,
            segment_name="startup-still",
            frames=args.startup_frames,
            gx=args.bias_gx,
            gy=args.bias_gy,
            gz=args.bias_gz,
            btn=0x00,
            rate_hz=args.rate,
            result_codes=result_codes,
            verbose_rx=args.verbose_rx,
            global_frame_index=0,
            noise_gx=args.startup_noise_gx,
            noise_gy=args.startup_noise_gy,
            noise_gz=args.startup_noise_gz,
            state=startup_state,
            show_result_bytes=args.show_result_bytes,
        )
        print("=== Startup stillness stream complete ===")

        learned_bias = startup_state["last_bias"]

        if learned_bias is None:
            timeout_end = time.perf_counter() + 0.5
            while learned_bias is None and time.perf_counter() < timeout_end:
                pkt = try_read_pic_message(ser, result_codes, verbose_rx=args.verbose_rx)
                if pkt is None:
                    continue
                handle_rx_message(pkt, startup_state, args.show_result_bytes)
                learned_bias = startup_state["last_bias"]

        rows = []
        for trial_index in range(1, args.trials + 1):
            row, global_frame_index = run_no_motion_trial(
                ser=ser,
                trial_index=trial_index,
                args=args,
                result_codes=result_codes,
                global_frame_index=global_frame_index,
            )
            rows.append(row)
            time.sleep(args.inter_trial_gap_ms / 1000.0)

        print("\n=== Pilot trial table ===")
        for r in rows:
            print(
                f"trial {r['trial_index']:2d}: "
                f"Dx={str(r['dx']):>6s}  Dy={str(r['dy']):>6s}  "
                f"|Dx|={str(r['abs_dx']):>6s}  |Dy|={str(r['abs_dy']):>6s}  "
                f"env={str(r['envelope']):>6s}"
            )

        if learned_bias is not None:
            print(
                f"\nLearned startup bias from PIC: "
                f"Gx={learned_bias['gx']}  Gy={learned_bias['gy']}"
            )
        else:
            print("\nNo learned bias frame was received from the PIC.")

        print_threshold_recommendation(rows, margin=args.threshold_margin)

        if args.csv:
            write_csv(args.csv, rows, learned_bias)
            print(f"\nCSV written to {args.csv}")


if __name__ == "__main__":
    main()
