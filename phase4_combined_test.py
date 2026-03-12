import serial
import time
import random
import argparse

MARKER_TX = 0xAA
MARKER_RX = 0xD1

def build_frame(gx, gy, gz, btn):
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

def read_exact(ser, n):
    data = b""
    while len(data) < n:
        chunk = ser.read(n - len(data))
        if not chunk:
            return None
        data += chunk
    return data

def try_read_debug_packet(ser):
    b = ser.read(1)
    if not b:
        return None
    if b[0] != MARKER_RX:
        return None

    payload = read_exact(ser, 6)
    if payload is None:
        return None

    tick_l, tick_h, gx_l, gx_h, flags, checksum = payload
    calc = (tick_l + tick_h + gx_l + gx_h + flags) & 0xFF
    if calc != checksum:
        return {"bad_checksum": True, "raw": payload}

    tick = tick_l | (tick_h << 8)
    gx = gx_l | (gx_h << 8)
    if gx & 0x8000:
        gx -= 0x10000

    return {
        "tick": tick,
        "gx": gx,
        "new": 1 if (flags & 0x01) else 0,
        "no_valid": 1 if (flags & 0x02) else 0,
        "bad_checksum": False,
    }

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

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM5")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--mode", choices=["steady", "jitter", "burstgap", "dropout"], default="steady")
    ap.add_argument("--count", type=int, default=50)
    ap.add_argument("--rate", type=float, default=50.0)
    ap.add_argument("--jitter-ms", type=float, default=8.0)
    ap.add_argument("--burst-short-ms", type=float, default=2.0)
    ap.add_argument("--burst-long-ms", type=float, default=38.0)
    ap.add_argument("--dropout-every", type=int, default=10)
    ap.add_argument("--dropout-extra-ms", type=float, default=40.0)
    args = ap.parse_args()

    base_period = 1.0 / args.rate

    with serial.Serial(args.port, args.baud, timeout=0.002) as ser:
        print(f"Opened {args.port} at {args.baud}")
        print("Sending frames and reading PIC debug packets...\n")

        gx_seq = 1
        next_send_time = time.perf_counter()

        sent = 0
        received_debug = 0
        t_end_guard = time.perf_counter() + max(5, args.count * base_period * 3)

        while True:
            now = time.perf_counter()

            if sent < args.count and now >= next_send_time:
                frame = build_frame(gx_seq, 0, 0, 1)
                ser.write(frame)
                print(f"TX frame {sent+1:3d}: gx={gx_seq:5d}")

                delay = next_delay(
                    args.mode,
                    base_period,
                    args.jitter_ms,
                    args.burst_short_ms,
                    args.burst_long_ms,
                    args.dropout_every,
                    args.dropout_extra_ms,
                    sent
                )
                next_send_time = now + delay
                gx_seq += 1
                sent += 1

            pkt = try_read_debug_packet(ser)
            if pkt is not None:
                if pkt.get("bad_checksum"):
                    print("RX debug: BAD CHECKSUM")
                else:
                    print(
                        f"RX debug: tick={pkt['tick']:5d}  "
                        f"gx={pkt['gx']:6d}  "
                        f"new={pkt['new']}  "
                        f"no_valid={pkt['no_valid']}"
                    )
                received_debug += 1

            if sent >= args.count and (time.perf_counter() > next_send_time + 1.0):
                break

            if time.perf_counter() > t_end_guard:
                break

        print("\nDone.")
        print(f"Frames sent: {sent}")
        print(f"Debug packets seen: {received_debug}")

if __name__ == "__main__":
    main()
