# Real-Time Smart Posture Monitoring System

A real-time embedded and desktop application for spine posture tracking and ergonomic feedback, developed as part of the *Data Acquisition Systems Practicum* at the School of Electrical Engineering, University of Belgrade.

## System Architecture
* **Hardware (Arduino Uno R3):**
  * **3-Axis Analog Accelerometer** (`A0`, `A1`, `A2`) for tilt sensing.
  * **Capacitive Touch Sensor** (`D2`, interrupt-driven) for triggering calibration and testing phases.
  * **TM1637 4-Digit 7-Segment Display** (`D8`, `D9`) for real-time violation counter display.
  * **On-board LED** for visual posture alerts.
* **Firmware (Embedded C++):**
  * Uses `TimerOne` hardware interrupts for deterministic **100 ms** sampling intervals.
  * Communicates with the host PC over UART (`115200 baud`) via a lightweight ASCII protocol.
* **Desktop Application & Signal Processing (Python):**
  * Multi-stage state machine (`Calibration` -> `Dynamic Tilt Test` -> `Active Monitoring`) built with `Tkinter` and `pyserial`.
  * **3-sample median filter** per axis to suppress electrical spikes and outliers.
  * **Least-squares zero-g bias estimation** across multi-orientation samples to isolate true gravitational acceleration.
  * **Exponentially Weighted Moving Average (EWMA)** angle smoothing and automatic session logging (`DZ2_session_log.txt`).

## How to Run
1. Upload the `.ino` firmware to an **Arduino Uno R3** wired according to the pinout above.
2. Install the serial communication library:
   ```bash
   pip install pyserial
   ```
3. Run the Python desktop application:
   ```bash
   python posture_monitor.py
   ```
