# PIC18F87K22 Gesture Firmware

modules:

- `main.s` orchestration and scheduling only.
- `board.s` owns startup pin and LED setup.
- `uart1.s` owns the UART peripheral and software RX ring state.
- `timer1.s` owns the 20 ms scheduler tick state.
- `isr.s` owns interrupt context and interrupt service flow.
- `parser.s` owns packet parsing and parser statistics.
- `sample_store.s` owns committed and per-tick sample data.
- `gesture_*` files split calibration, capture, classification, and output.
- `debug_frames.s` owns UART instrumentation frames.
