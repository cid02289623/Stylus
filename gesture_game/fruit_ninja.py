# pip install pyserial pygame

import math
import random
import time

import pygame
import serial


# -------------------------------------------------
# User config
# -------------------------------------------------
PORT = "COM5"                  # change if needed
BAUD = 38400
OUTFILE = "gesture_fruit_ninja_log.txt"

ENABLE_KEYBOARD_TEST = True

GESTURE_MIN_GAP_SEC = 0.08
GESTURE_REPEAT_ALLOW_SEC = 0.28

WINDOW_W = 1000
WINDOW_H = 700
FPS = 60

PHYSICS_HZ = 120
FIXED_DT = 1.0 / PHYSICS_HZ
MAX_FRAME_DT = 1.0 / 30.0

SPAWN_INTERVAL_SEC = 0.92
PACK_SIZE_MIN = 1
PACK_SIZE_MAX = 3

GRAVITY = 980.0
FRUIT_RADIUS = 34
FRUIT_START_Y_MIN = WINDOW_H + FRUIT_RADIUS + 30
FRUIT_START_Y_MAX = WINDOW_H + FRUIT_RADIUS + 95
FRUIT_APEX_MIN_Y = 150
FRUIT_APEX_MAX_Y = 280
FRUIT_TARGET_X_MARGIN = 170
FRUIT_MAX_SIDE_SPEED = 260.0

SLICE_WINDOW_TOP = 120
SLICE_WINDOW_BOTTOM = 560

SLASH_VISUAL_SEC = 0.10

START_LIVES = 10
POINTS_PER_FRUIT = 100
COMBO_BONUS_PER_EXTRA_FRUIT = 40

PRINT_RAW_FRAMES = False
PRINT_BIAS_FRAMES = False
PRINT_PIC_FRAMES = False
PRINT_INPUT_EVENTS = True

HIGHSCORE_FILE = "highscore.txt"

# -------------------------------------------------
# Protocol constants
# -------------------------------------------------
RAW_SOF  = 0xAA
DBG_SOF  = 0xE2
BIAS_SOF = 0xB1

RAW_LEN  = 9    # SOF + 6 gyro + btn + checksum
DBG_LEN  = 31   # SOF + 29 payload + checksum
BIAS_LEN = 6    # SOF + 4 payload + checksum


# -------------------------------------------------
# Helpers
# -------------------------------------------------
def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def s16(lo, hi):
    v = lo | (hi << 8)
    if v & 0x8000:
        v -= 0x10000
    return v

def s32(b0, b1, b2, b3):
    v = b0 | (b1 << 8) | (b2 << 16) | (b3 << 24)
    if v & 0x80000000:
        v -= 0x100000000
    return v

def decode_result(b):
    if 32 <= b <= 126:
        return chr(b)
    return f"0x{b:02X}"

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


# -------------------------------------------------
# High score persistence
# -------------------------------------------------
def load_records():
    try:
        with open(HIGHSCORE_FILE) as f:
            parts = f.read().strip().split()
            return int(parts[0]), int(parts[1])
    except Exception:
        return 0, 0

def save_records(high_score, best_combo):
    try:
        with open(HIGHSCORE_FILE, "w") as f:
            f.write(f"{high_score} {best_combo}\n")
    except Exception:
        pass


# -------------------------------------------------
# Serial reader — atomic frame reading
# -------------------------------------------------
class SerialGestureReader:
    """
    Reads the mixed PIC echo stream.

    Key fix vs original: once a SOF byte is identified, the full frame
    uses a non-blocking byte buffer.  This means 0xE2 bytes that
    appear inside a 0xAA gyro payload are never mistaken for a new
    classifier frame SOF, eliminating the RAW_BAD drop bug.

    All log lines are prefixed with milliseconds since session start.
    """

    def __init__(self, port, baud, outfile):
        self.port = port
        self.baud = baud
        self.t0   = time.monotonic()

        self.ser = None
        self.pending_gestures = []
        self.last_action_time = 0.0
        self.latched_result   = None
        self._buf = bytearray()

        self.f = open(outfile, "w", buffering=1, encoding="utf-8")

        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
            time.sleep(2)
            self._log(f"SESSION_START port={self.port} baud={self.baud}", important=True)
        except Exception as e:
            self.ser = None
            self._log(f"SERIAL_ERROR {e}", important=True)

    def close(self):
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass
        try:
            self.f.close()
        except Exception:
            pass

    def _ms(self):
        return (time.monotonic() - self.t0) * 1000.0

    def _log(self, line, important=False):
        stamped = f"[{self._ms():10.1f}] {line}"
        if important:
            print(stamped)
        self.f.write(stamped + "\n")

    def poll(self):
        """
        Non-blocking poll — called every game frame.
        Drains waiting bytes into self._buf, then parses complete frames.
        A frame is only consumed once ALL its bytes are buffered, so this
        never blocks. The SOF byte is only treated as a frame header when
        the full frame is already buffered — 0xE2 inside a 0xAA payload
        can never trigger the classifier handler (fixes the RAW_BAD bug).
        """
        if self.ser is None:
            return
        try:
            waiting = self.ser.in_waiting
            if waiting:
                self._buf.extend(self.ser.read(waiting))
        except Exception as e:
            self._log(f"SERIAL_READ_ERROR {e}", important=True)
            return

        if len(self._buf) > 8192:
            self._buf = self._buf[-256:]

        SIZES = {RAW_SOF: RAW_LEN, BIAS_SOF: BIAS_LEN, DBG_SOF: DBG_LEN}
        while self._buf:
            sof = self._buf[0]
            if sof not in SIZES:
                del self._buf[0]
                continue
            needed = SIZES[sof]
            if len(self._buf) < needed:
                break
            frame = bytes(self._buf[:needed])
            del self._buf[:needed]
            if   sof == RAW_SOF:  self._handle_raw(frame)
            elif sof == BIAS_SOF: self._handle_bias(frame)
            elif sof == DBG_SOF:  self._handle_dbg(frame)

    def _handle_raw(self, frame):

        if sum(frame[1:8]) & 0xFF != frame[8]:
            self._log(f"RAW_BAD {' '.join(f'{x:02X}' for x in frame)}",
                      important=PRINT_RAW_FRAMES)
            return

        gx  = s16(frame[1], frame[2])
        gy  = s16(frame[3], frame[4])
        gz  = s16(frame[5], frame[6])
        btn = frame[7]
        self._log(
            f"RAW  bytes={' '.join(f'{x:02X}' for x in frame)}  "
            f"gx={gx} gy={gy} gz={gz} btn=0x{btn:02X}",
            important=PRINT_RAW_FRAMES)

    def _handle_bias(self, frame):
        payload = frame[1:5]
        rx_chk  = frame[5]

        if sum(payload) & 0xFF != rx_chk:
            self._log(f"BIAS_BAD B1 {' '.join(f'{x:02X}' for x in frame[1:])}",
                      important=PRINT_BIAS_FRAMES)
            return

        bias_gx = s16(payload[0], payload[1])
        bias_gz = s16(payload[2], payload[3])
        self._log(f"BIAS bias_gx={bias_gx} bias_gz={bias_gz}",
                  important=PRINT_BIAS_FRAMES)

    def _handle_dbg(self, frame):
        payload = frame[1:30]
        rx_chk  = frame[30]

        if sum(payload) & 0xFF != rx_chk:
            self._log(f"DBG_BAD E2 {' '.join(f'{x:02X}' for x in frame[1:])}",
                      important=PRINT_PIC_FRAMES)
            return

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
        result   = payload[28]
        result_text = decode_result(result)

        self._log(
            f"PIC  proc_gx={proc_gx} proc_gy={proc_gy} proc_gz={proc_gz} "
            f"proc_btn=0x{proc_btn:02X}  "
            f"bias_gx={bias_gx} bias_gz={bias_gz}  "
            f"acc_v={acc_v} acc_h={acc_h}  "
            f"peak_v_pos={pv_pos} peak_v_neg={pv_neg}  "
            f"peak_h_pos={ph_pos} peak_h_neg={ph_neg}  "
            f"flags=0x{flags:02X}({flag_text(flags)})  "
            f"result={result_text}",
            important=PRINT_PIC_FRAMES)

        self._handle_result(result_text)

    def _handle_result(self, result_text):
        now = time.monotonic()
        if result_text in ("U", "D", "L", "R"):
            enough_gap    = (now - self.last_action_time) >= GESTURE_MIN_GAP_SEC
            repeated_long = (now - self.last_action_time) >= GESTURE_REPEAT_ALLOW_SEC
            is_new        = (result_text != self.latched_result)
            if enough_gap and (is_new or repeated_long):
                self.pending_gestures.append(result_text)
                self.last_action_time = now
                self._log(f"INPUT {result_text}", important=PRINT_INPUT_EVENTS)
            self.latched_result = result_text
        elif result_text == "?":
            self.latched_result = None

    def pop_gestures(self):
        out = self.pending_gestures[:]
        self.pending_gestures.clear()
        return out


# -------------------------------------------------
# Game helpers
# -------------------------------------------------
GESTURE_COLORS  = {"U": (90, 220, 120), "D": (255, 180, 70),
                   "L": (90, 170, 255),  "R": (220, 120, 255)}
GESTURE_SYMBOLS = {"U": "↑", "D": "↓", "L": "←", "R": "→"}


def build_background():
    bg = pygame.Surface((WINDOW_W, WINDOW_H)).convert()
    for y in range(WINDOW_H):
        t = y / WINDOW_H
        pygame.draw.line(bg, (int(18+28*t), int(24+34*t), int(38+52*t)),
                         (0, y), (WINDOW_W, y))
    return bg

def build_zone_overlay():
    overlay   = pygame.Surface((WINDOW_W, WINDOW_H), pygame.SRCALPHA)
    h         = SLICE_WINDOW_BOTTOM - SLICE_WINDOW_TOP
    band_rect = pygame.Rect(20, SLICE_WINDOW_TOP, WINDOW_W - 40, h)
    pygame.draw.rect(overlay, (255, 255, 255,  20), band_rect, border_radius=18)
    pygame.draw.rect(overlay, (255, 255, 255,  55), band_rect, width=2, border_radius=18)
    return overlay

def new_fruit(spawn_x, target_x):
    gesture    = random.choice(["U", "D", "L", "R"])
    x          = spawn_x
    y          = random.uniform(FRUIT_START_Y_MIN, FRUIT_START_Y_MAX)
    apex_y     = random.uniform(FRUIT_APEX_MIN_Y, FRUIT_APEX_MAX_Y)
    vy         = -math.sqrt(max(1.0, 2.0 * GRAVITY * (y - apex_y)))
    vx         = clamp((target_x - x) / max(0.1, -vy / GRAVITY),
                       -FRUIT_MAX_SIDE_SPEED, FRUIT_MAX_SIDE_SPEED)
    return {"gesture": gesture, "x": x, "y": y, "vx": vx, "vy": vy,
            "r": FRUIT_RADIUS, "color": GESTURE_COLORS[gesture],
            "spin": random.uniform(-220.0, 220.0),
            "rot":  random.uniform(-18.0,  18.0)}

def spawn_pack(fruits):
    count    = random.randint(PACK_SIZE_MIN, PACK_SIZE_MAX)
    center_x = random.uniform(220, WINDOW_W - 220)
    offsets  = {1: [0], 2: [-65, 65], 3: [-120, 0, 120]}
    for offset in offsets[count]:
        sx = clamp(center_x + offset + random.uniform(-18, 18), 90, WINDOW_W - 90)
        tx = clamp(center_x + offset * 0.45 + random.uniform(-45, 45),
                   FRUIT_TARGET_X_MARGIN, WINDOW_W - FRUIT_TARGET_X_MARGIN)
        fruits.append(new_fruit(sx, tx))

def make_particles(x, y, color):
    parts = []
    for _ in range(16):
        angle = random.uniform(0, math.tau)
        speed = random.uniform(90, 300)
        parts.append({"x": x, "y": y,
                       "vx": math.cos(angle)*speed, "vy": math.sin(angle)*speed - 90,
                       "life": random.uniform(0.24, 0.52), "max_life": 0.52, "color": color})
    return parts

def make_slash_effect(gesture):
    return {"gesture": gesture, "life": SLASH_VISUAL_SEC,
            "max_life": SLASH_VISUAL_SEC, "color": GESTURE_COLORS[gesture]}

def slice_matching_fruits(fruits, gesture, particles, slash_effects):
    sliced = 0
    remaining = []
    for fruit in fruits:
        if fruit["gesture"] == gesture and SLICE_WINDOW_TOP <= fruit["y"] <= SLICE_WINDOW_BOTTOM:
            sliced += 1
            particles.extend(make_particles(fruit["x"], fruit["y"], fruit["color"]))
        else:
            remaining.append(fruit)
    fruits[:] = remaining
    slash_effects.append(make_slash_effect(gesture))
    return sliced

def update_fruits(fruits, dt):
    lost = 0; i = 0
    while i < len(fruits):
        f = fruits[i]
        f["x"] += f["vx"] * dt; f["y"] += f["vy"] * dt
        f["vy"] += GRAVITY * dt; f["rot"] += f["spin"] * dt
        if   f["y"] > WINDOW_H + 120:                   fruits.pop(i); lost += 1
        elif f["x"] < -140 or f["x"] > WINDOW_W + 140: fruits.pop(i)
        else: i += 1
    return lost

def update_particles(particles, dt):
    i = 0
    while i < len(particles):
        p = particles[i]
        p["x"] += p["vx"]*dt; p["y"] += p["vy"]*dt
        p["vy"] += 650*dt;     p["life"] -= dt
        if p["life"] <= 0: particles.pop(i)
        else: i += 1

def update_slash_effects(slash_effects, dt):
    i = 0
    while i < len(slash_effects):
        slash_effects[i]["life"] -= dt
        if slash_effects[i]["life"] <= 0: slash_effects.pop(i)
        else: i += 1

def draw_text(surface, font, text, color, x, y, center=False, topright=False):
    img  = font.render(text, True, color)
    rect = img.get_rect()
    if center:   rect.center   = (x, y)
    elif topright: rect.topright = (x, y)
    else:        rect.topleft  = (x, y)
    surface.blit(img, rect)

def draw_fruit(surface, fruit, font):
    x, y, r = int(fruit["x"]), int(fruit["y"]), fruit["r"]
    shadow   = pygame.Surface((r*3, r*3), pygame.SRCALPHA)
    pygame.draw.circle(shadow, (0,0,0,70), (r+6, r+8), r)
    surface.blit(shadow, (x-r, y-r))
    pygame.draw.circle(surface, fruit["color"], (x, y), r)
    pygame.draw.circle(surface, (255,255,255), (x, y), r, 3)
    pygame.draw.circle(surface, (255,255,255), (x-r//3, y-r//3), max(5, r//5))
    draw_text(surface, font, GESTURE_SYMBOLS[fruit["gesture"]], (15,15,15), x, y-1, center=True)

def draw_particles(surface, particles):
    layer = pygame.Surface((WINDOW_W, WINDOW_H), pygame.SRCALPHA)
    for p in particles:
        a = max(0.0, min(1.0, p["life"]/p["max_life"]))
        pygame.draw.circle(layer, (*p["color"], int(180*a)),
                           (int(p["x"]), int(p["y"])), int(3+4*a))
    surface.blit(layer, (0,0))

def draw_slash_effects(surface, slash_effects):
    cy    = (SLICE_WINDOW_TOP + SLICE_WINDOW_BOTTOM) // 2
    cx    = WINDOW_W // 2
    layer = pygame.Surface((WINDOW_W, WINDOW_H), pygame.SRCALPHA)
    for e in slash_effects:
        t = e["life"] / e["max_life"]
        a = int(170*t); w = max(4, int(10+8*t)); c = e["color"]
        if   e["gesture"] == "L": pygame.draw.line(layer, (*c,a), (WINDOW_W-90, cy), (90, cy), w)
        elif e["gesture"] == "R": pygame.draw.line(layer, (*c,a), (90, cy), (WINDOW_W-90, cy), w)
        elif e["gesture"] == "U": pygame.draw.line(layer, (*c,a), (cx, WINDOW_H-90), (cx, 90), w)
        elif e["gesture"] == "D": pygame.draw.line(layer, (*c,a), (cx, 90), (cx, WINDOW_H-90), w)
    surface.blit(layer, (0,0))

def reset_game():
    return {"fruits": [], "particles": [], "slash_effects": [],
            "spawn_timer": 0.0, "score": 0, "combo": 0, "peak_combo": 0,
            "lives": START_LIVES, "last_input": "-",
            "last_message": "Ready", "message_timer": 0.9, "game_over": False}

def update_game_state(state, dt, queued_inputs):
    state["spawn_timer"] += dt
    while state["spawn_timer"] >= SPAWN_INTERVAL_SEC:
        state["spawn_timer"] -= SPAWN_INTERVAL_SEC
        spawn_pack(state["fruits"])

    lost = update_fruits(state["fruits"], dt)
    if lost > 0:
        state["lives"] -= lost; state["combo"] = 0
        state["last_message"] = f"Missed {lost}"; state["message_timer"] = 0.65

    update_particles(state["particles"], dt)
    update_slash_effects(state["slash_effects"], dt)

    for gesture in queued_inputs:
        state["last_input"] = gesture
        sliced = slice_matching_fruits(state["fruits"], gesture,
                                       state["particles"], state["slash_effects"])
        if sliced > 0:
            gained = POINTS_PER_FRUIT * sliced + COMBO_BONUS_PER_EXTRA_FRUIT * max(0, sliced-1)
            state["score"] += gained; state["combo"] += sliced
            if state["combo"] > state["peak_combo"]:
                state["peak_combo"] = state["combo"]
            state["last_message"] = (f"Combo x{sliced}! +{gained}" if sliced > 1
                                     else f"Slice! +{gained}")
            state["message_timer"] = 0.55
        else:
            state["combo"] = 0
            state["last_message"] = "Whiff"; state["message_timer"] = 0.25

    state["message_timer"] = max(0.0, state["message_timer"] - dt)
    if state["lives"] <= 0:
        state["lives"] = 0; state["game_over"] = True


# -------------------------------------------------
# Main
# -------------------------------------------------
def main():
    pygame.init()
    try:
        screen = pygame.display.set_mode((WINDOW_W, WINDOW_H), vsync=1)
    except TypeError:
        screen = pygame.display.set_mode((WINDOW_W, WINDOW_H))
    pygame.display.set_caption("PIC Fruit Ninja")
    clock = pygame.time.Clock()

    font_small = pygame.font.SysFont("arial", 24)
    font_med   = pygame.font.SysFont("arial", 34, bold=True)
    font_big   = pygame.font.SysFont("arial", 56, bold=True)

    bg           = build_background()
    zone_overlay = build_zone_overlay()

    reader     = SerialGestureReader(PORT, BAUD, OUTFILE)
    state      = reset_game()
    high_score, best_combo = load_records()
    accumulator = 0.0
    running     = True

    while running:
        frame_dt = min(clock.tick(FPS) / 1000.0, MAX_FRAME_DT)

        # Events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                if state["game_over"] and event.key == pygame.K_r:
                    if state["score"]      > high_score: high_score = state["score"]
                    if state["peak_combo"] > best_combo: best_combo = state["peak_combo"]
                    save_records(high_score, best_combo)
                    state = reset_game(); accumulator = 0.0
                if ENABLE_KEYBOARD_TEST and not state["game_over"]:
                    if   event.key == pygame.K_UP:    reader.pending_gestures.append("U")
                    elif event.key == pygame.K_DOWN:  reader.pending_gestures.append("D")
                    elif event.key == pygame.K_LEFT:  reader.pending_gestures.append("L")
                    elif event.key == pygame.K_RIGHT: reader.pending_gestures.append("R")

        # Serial
        reader.poll()
        queued_inputs = reader.pop_gestures() if not state["game_over"] else []

        # Physics
        if not state["game_over"]:
            accumulator += frame_dt
            while accumulator >= FIXED_DT:
                update_game_state(state, FIXED_DT, queued_inputs)
                queued_inputs = []; accumulator -= FIXED_DT

        # Save records live
        if state["game_over"]:
            if state["score"]      > high_score: high_score = state["score"]
            if state["peak_combo"] > best_combo: best_combo = state["peak_combo"]
            save_records(high_score, best_combo)

        # Draw
        screen.blit(bg, (0, 0))
        screen.blit(zone_overlay, (0, 0))
        for fruit in state["fruits"]: draw_fruit(screen, fruit, font_med)
        draw_particles(screen, state["particles"])
        draw_slash_effects(screen, state["slash_effects"])

        # HUD left
        draw_text(screen, font_small, f"Score:  {state['score']}",      (255,255,255), 20, 20)
        draw_text(screen, font_small, f"Combo:  {state['combo']}",      (255,255,255), 20, 46)
        draw_text(screen, font_small, f"Lives:  {state['lives']}",      (255,255,255), 20, 72)
        draw_text(screen, font_small, f"Input:  {state['last_input']}", (255,255,255), 20, 98)

        # HUD right — high score and best combo
        hs_surf = font_small.render(f"Best score: {high_score}", True, (255,220,80))
        bc_surf = font_small.render(f"Best combo: {best_combo}", True, (255,220,80))
        screen.blit(hs_surf, hs_surf.get_rect(topright=(WINDOW_W-20, 20)))
        screen.blit(bc_surf, bc_surf.get_rect(topright=(WINDOW_W-20, 46)))

        # Serial status
        if reader.ser is None:
            draw_text(screen, font_small,
                      "Serial not connected — keyboard test mode only",
                      (255,140,140), 20, 660)
        else:
            draw_text(screen, font_small, f"Serial: {PORT} @ {BAUD}",
                      (160,220,255), 20, 660)

        draw_text(screen, font_small,
                  "Slice fruits by doing the matching gesture shown on the fruit.",
                  (235,235,235), 20, 620)
        draw_text(screen, font_small,
                  "Keyboard: arrow keys  |  Restart: R  |  Quit: Esc",
                  (235,235,235), 20, 590)

        if state["message_timer"] > 0:
            draw_text(screen, font_big, state["last_message"],
                      (255,255,255), WINDOW_W//2, 84, center=True)

        # Game over screen
        if state["game_over"]:
            overlay = pygame.Surface((WINDOW_W, WINDOW_H), pygame.SRCALPHA)
            overlay.fill((0,0,0,150))
            screen.blit(overlay, (0,0))

            new_hs = state["score"]      >= high_score and high_score > 0
            new_bc = state["peak_combo"] >= best_combo and best_combo > 0

            draw_text(screen, font_big,  "GAME OVER",
                      (255,255,255), WINDOW_W//2, 200, center=True)
            draw_text(screen, font_med,
                      f"Score: {state['score']}" +
                      (" ★ NEW BEST!" if new_hs else f"  (best: {high_score})"),
                      (255,220,80) if new_hs else (255,255,255),
                      WINDOW_W//2, 285, center=True)
            draw_text(screen, font_med,
                      f"Best combo this game: {state['peak_combo']}" +
                      (" ★ NEW BEST!" if new_bc else f"  (all-time best: {best_combo})"),
                      (255,220,80) if new_bc else (255,255,255),
                      WINDOW_W//2, 335, center=True)
            draw_text(screen, font_small,
                      "Press R to restart or Esc to quit",
                      (255,255,255), WINDOW_W//2, 400, center=True)

        pygame.display.flip()

    reader.close()
    pygame.quit()


if __name__ == "__main__":
    main()