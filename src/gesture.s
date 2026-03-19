            PROCESSOR 18F87K22
            #include <xc.inc>

;===========================================================
; gesture.s
; This file is intentionally empty.
; All gesture functionality has been split into:
;   gesture_core.s        - GestureInit, ProcessTickService,
;                           GestureService, GestureAccumulate
;   gesture_capture.s     - Peak update helpers, MakeAbs
;   gesture_calibration.s - Calibration service and bias compute
;   gesture_classify.s    - Classification and dominance checks
;   gesture_output.s      - LED output and debug frame TX
;===========================================================

            end
