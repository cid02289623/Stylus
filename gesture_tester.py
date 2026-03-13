import serial
import time
import random
import argparse
import csv

MARKER_TX = 0xAA


def int_auto(text):
    return int(text, 0)


def build_frame(gx, gy, gz, btn):
    """
    Build a frame matching parser.s exactly:

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


def next_delay(mode, base_period, jitter_ms, burst_short_ms, burst_long_ms,
               dropout_every, dropout_extra_ms, frame_index):
    if mode == "steady":
        return base_period

    if mode == "jitter":
        jitter = random.uniform(-jitter_ms / 1000.0, jitter_ms / 1000.0)
        return max(0.0, base_period + jitter)

    if mode == "burstgap":
        return burst_short_ms / 1000.0 if (frame_index % 2 == 0) else burst_long_ms / 1000.0

    if mode == "dropout":
        if dropout_every > 0 and ((frame_index + 1) % dropout_every == 0):
            return base_period + (dropout_extra_ms / 1000.0)
        return base_period

    return base_period


def try_read_result_byte(ser, result_codes, verbose_rx=False):
    """
    Read exactly one byte from PIC TX.
    Only configured result bytes are treated as classifier outputs.
    Any other byte is ignored unless verbose_rx is enabled.
    """
    b = ser.read(1)
    if not b:
        return None

    value = b[0]
    if value in result_codes:
        return {
            "code": value,
            "label": result_codes[value],
            "time": time.perf_counter(),
        }

    if verbose_rx:
        print(f"RX ignored byte: 0x{value:02X}")

    return None


def gesture_to_vector(label, amp, unknown_amp, invert_gx=False, invert_gy=False):
    """
    Default mapping:
      +Gx -> RIGHT
      -Gx -> LEFT
      +Gy -> UP
      -Gy -> DOWN

    Use --invert-gx and/or --invert-gy if the firmware uses the opposite sign convention.
    """
    sx = -1 if invert_gx else 1
    sy = -1 if invert_gy else 1

    if label == "RIGHT":
        return sx * amp, 0
    if label == "LEFT":
        return -sx * amp, 0
    if label == "UP":
        return 0, sy * amp
    if label == "DOWN":
        return 0, -sy * amp
    if label == "UNKNOWN":
        return unknown_amp, 0

    raise ValueError(f"Unknown gesture label: {label}")


def build_trial_sequence(label, amp, unknown_amp, press_frames, capture_frames,
                         hold_frames, idle_frames, button_mask,
                         invert_gx=False, invert_gy=False):
    """
    Build one synthetic gesture trial.

    Sequence:
      idle-before
      press-settle
      capture
      hold-still
      release
      idle-after
    """
    gx, gy = gesture_to_vector(
        label=label,
        amp=amp,
        unknown_amp=unknown_amp,
        invert_gx=invert_gx,
        invert_gy=invert_gy,
    )

    seq = []

    for _ in range(idle_frames):
        seq.append(("idle-before", 0, 0, 0, 0))

    for _ in range(press_frames):
        seq.append(("press-settle", 0, 0, 0, button_mask))

    for _ in range(capture_frames):
        seq.append(("capture", gx, gy, 0, button_mask))

    for _ in range(hold_frames):
        seq.append(("hold-still", 0, 0, 0, button_mask))

    seq.append(("release", 0, 0, 0, 0))

    for _ in range(idle_frames):
        seq.append(("idle-after", 0, 0, 0, 0))

    return seq


def run_trial(ser, expected_label, args, result_codes, global_frame_index):
    base_period = 1.0 / args.rate

    sequence = build_trial_sequence(
        label=expected_label,
        amp=args.amp,
        unknown_amp=args.unknown_amp,
        press_frames=args.press_frames,
        capture_frames=args.capture_frames,
        hold_frames=args.hold_frames,
        idle_frames=args.idle_frames,
        button_mask=args.button_mask,
        invert_gx=args.invert_gx,
        invert_gy=args.invert_gy,
    )

    print(f"\n=== Trial expected: {expected_label} ===")

    first_result = None
    release_time = None

    for i, (tag, gx, gy, gz, btn) in enumerate(sequence):
        frame = build_frame(gx, gy, gz, btn)
        send_time = time.perf_counter()
        ser.write(frame)

        print(
            f"TX frame {global_frame_index + i + 1:4d}: "
            f"{tag:12s} "
            f"gx={gx:6d} gy={gy:6d} gz={gz:6d} btn=0x{btn:02X}"
        )

        if tag == "release":
            release_time = send_time

        delay = next_delay(
            args.mode,
            base_period,
            args.jitter_ms,
            args.burst_short_ms,
            args.burst_long_ms,
            args.dropout_every,
            args.dropout_extra_ms,
            global_frame_index + i,
        )

        end_time = send_time + delay
        while time.perf_counter() < end_time:
            pkt = try_read_result_byte(ser, result_codes, verbose_rx=args.verbose_rx)
            if pkt is not None and first_result is None:
                print(f"RX result: {pkt['label']} (0x{pkt['code']:02X})")
                first_result = pkt

    # After release, give PIC time to classify and transmit one result byte.
    timeout_end = time.perf_counter() + (args.result_timeout_ms / 1000.0)
    while first_result is None and time.perf_counter() < timeout_end:
        pkt = try_read_result_byte(ser, result_codes, verbose_rx=args.verbose_rx)
        if pkt is not None:
            print(f"RX result: {pkt['label']} (0x{pkt['code']:02X})")
            first_result = pkt
            break

    observed = first_result["label"] if first_result is not None else "NO_RESULT"
    latency_ms = None
    if first_result is not None and release_time is not None:
        latency_ms = (first_result["time"] - release_time) * 1000.0

    passed = (observed == expected_label)

    print(
        f"Trial outcome: expected={expected_label} observed={observed} pass={passed}"
        + (f" latency_ms={latency_ms:.1f}" if latency_ms is not None else "")
    )

    return {
        "expected": expected_label,
        "observed": observed,
        "pass": passed,
        "latency_ms": latency_ms,
        "frames_sent": len(sequence),
    }


def make_trial_plan(gesture_mode, trials_per_class, shuffle):
    if gesture_mode == "all":
        labels = ["LEFT", "RIGHT", "UP", "DOWN", "UNKNOWN"]
    else:
        labels = [gesture_mode.upper()]

    plan = []
    for _ in range(trials_per_class):
        for label in labels:
            plan.append(label)

    if shuffle:
        random.shuffle(plan)

    return plan


def main():
    ap = argparse.ArgumentParser(description="Phase 5 synthetic gesture tester for PIC parser/classifier")

    ap.add_argument("--port", default="COM5", help="Serial port, e.g. COM5 or /dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=115200, help="Baud rate")

    ap.add_argument("--gesture", choices=["left", "right", "up", "down", "unknown", "all"], default="all")
    ap.add_argument("--trials-per-class", type=int, default=2)
    ap.add_argument("--shuffle", action="store_true", help="Shuffle trial order")

    ap.add_argument("--rate", type=float, default=50.0, help="Frame send rate in Hz")
    ap.add_argument("--mode", choices=["steady", "jitter", "burstgap", "dropout"], default="steady")

    ap.add_argument("--amp", type=int, default=1200, help="Strong capture amplitude for LEFT/RIGHT/UP/DOWN")
    ap.add_argument("--unknown-amp", type=int, default=40, help="Small capture amplitude for UNKNOWN trials")

    ap.add_argument("--press-frames", type=int, default=2, help="Frames with button pressed before motion")
    ap.add_argument("--capture-frames", type=int, default=8, help="Frames carrying gesture motion")
    ap.add_argument("--hold-frames", type=int, default=1, help="Frames with button still pressed after motion")
    ap.add_argument("--idle-frames", type=int, default=3, help="Idle frames before and after the gesture")

    ap.add_argument("--button-mask", type=int_auto, default=0x01, help="Button byte value meaning pressed")
    ap.add_argument("--invert-gx", action="store_true", help="Invert Gx sign convention")
    ap.add_argument("--invert-gy", action="store_true", help="Invert Gy sign convention")

    ap.add_argument("--jitter-ms", type=float, default=8.0)
    ap.add_argument("--burst-short-ms", type=float, default=2.0)
    ap.add_argument("--burst-long-ms", type=float, default=38.0)
    ap.add_argument("--dropout-every", type=int, default=10)
    ap.add_argument("--dropout-extra-ms", type=float, default=40.0)

    ap.add_argument("--result-timeout-ms", type=float, default=400.0, help="Wait after release for result byte")
    ap.add_argument("--inter-trial-gap-ms", type=float, default=50.0)

    ap.add_argument("--csv", default="", help="Write results to CSV file")
    ap.add_argument("--verbose-rx", action="store_true", help="Print ignored RX bytes")

    # Default result code mapping. Override these to match Person A firmware exactly.
    ap.add_argument("--code-unknown", type=int_auto, default=0x00)
    ap.add_argument("--code-left", type=int_auto, default=0x01)
    ap.add_argument("--code-right", type=int_auto, default=0x02)
    ap.add_argument("--code-up", type=int_auto, default=0x03)
    ap.add_argument("--code-down", type=int_auto, default=0x04)

    args = ap.parse_args()

    result_codes = {
        args.code_unknown: "UNKNOWN",
        args.code_left: "LEFT",
        args.code_right: "RIGHT",
        args.code_up: "UP",
        args.code_down: "DOWN",
    }

    plan = make_trial_plan(args.gesture, args.trials_per_class, args.shuffle)

    with serial.Serial(args.port, args.baud, timeout=0.002) as ser:
        print(f"Opened {args.port} at {args.baud}")
        print(f"Trial count: {len(plan)}")
        print("Starting Phase 5 synthetic gesture test...")

        rows = []
        global_frame_index = 0

        # Drain any stale bytes before starting
        drain_end = time.perf_counter() + 0.2
        while time.perf_counter() < drain_end:
            _ = try_read_result_byte(ser, result_codes, verbose_rx=args.verbose_rx)

        for trial_index, expected_label in enumerate(plan, start=1):
            print(f"\n##### Running trial {trial_index}/{len(plan)} #####")

            row = run_trial(
                ser=ser,
                expected_label=expected_label,
                args=args,
                result_codes=result_codes,
                global_frame_index=global_frame_index,
            )

            row["trial_index"] = trial_index
            rows.append(row)
            global_frame_index += row["frames_sent"]

            time.sleep(args.inter_trial_gap_ms / 1000.0)

        print("\n=== Summary ===")
        total = len(rows)
        passed = sum(1 for r in rows if r["pass"])
        failed = total - passed

        print(f"Trials run: {total}")
        print(f"Passed:     {passed}")
        print(f"Failed:     {failed}")

        for label in ["LEFT", "RIGHT", "UP", "DOWN", "UNKNOWN"]:
            subset = [r for r in rows if r["expected"] == label]
            if subset:
                ok = sum(1 for r in subset if r["pass"])
                print(f"{label:7s}: {ok}/{len(subset)} correct")

        latencies = [r["latency_ms"] for r in rows if r["latency_ms"] is not None]
        if latencies:
            avg_latency = sum(latencies) / len(latencies)
            worst_latency = max(latencies)
            print(f"Average host-observed latency: {avg_latency:.1f} ms")
            print(f"Worst host-observed latency:   {worst_latency:.1f} ms")
        else:
            print("No latency data available because no valid result bytes were received.")

        if args.csv:
            with open(args.csv, "w", newline="") as f:
                writer = csv.DictWriter(
                    f,
                    fieldnames=["trial_index", "expected", "observed", "pass", "latency_ms", "frames_sent"]
                )
                writer.writeheader()
                writer.writerows(rows)
            print(f"CSV written to {args.csv}")


if __name__ == "__main__":
    main()
