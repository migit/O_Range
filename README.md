# O_Range — Budget-Friendly Self-Balancing Robot with AI-Enhanced PID Tuning 

[![License: MIT](https://img.shields.io/badge/license-MIT-blue.svg)](#license)
[![Languages](https://img.shields.io/badge/languages-C%2B%2B%20%7C%20Python%20%7C%20Processing-green.svg)](#tech-stack) [![GitHub repo size](https://img.shields.io/github/repo-size/migit/O_Range)](#)
---

## Story

I built this robot named **O_Range** to explore control theory and IoT on a budget. Initially, manual PID tuning via sliders was effective but time-consuming. Switching to BTS7960 drivers added power (up to 43A), and WiFi freed me from USB cables. The real breakthrough was AI-based tuning: I modeled the robot as an inverted pendulum in Python, using SciPy's Differential Evolution to find optimal PID values (e.g., `Kp ≈ 200`, `Ki ≈ 0`, `Kd ≈ 1.6`). These stabilized the simulation, and with minor tweaks, the real robot balanced smoothly.

The Processing GUI lets me monitor tilt and fine-tune wirelessly, making iterations fast and fun!

  <img src="https://github.com/user-attachments/assets/a4e3065c-3613-4de9-91ed-de61130422eb" width="30%" />
  <img src="https://github.com/user-attachments/assets/ea26151e-7a25-4ae7-bd1c-7a5db88d7190" width="30%" />
  <img src="https://github.com/user-attachments/assets/93246395-bc40-4c87-a073-9017291b710d" width="30%" />


**Highlights**:

* **Hardware**: ESP32 DevKit, MPU6050 IMU, BTS7960 drivers, two DC motors
* **Wireless Control**: WiFi-based communication with Processing GUI
* **AI Tuning**: Python script using Differential Evolution to optimize PID parameters
* **GUI**: Sci-fi-themed Processing interface with neon graphs, sliders, and status indicators
* **Scalable**: Add encoders, Bluetooth, or real-time ML for advanced control

---

## Materials

* ESP32 DevKit V1 (or similar)
* MPU6050 (Gyroscope + Accelerometer IMU)
* 2× BTS7960 Motor Driver Modules (high-current H-bridges)
* 2× DC Motors (12V, geared for torque, encoders optional)
* Chassis (3D-printed or acrylic, two-wheel design)
* 2× Wheels (rubber-tread for grip)
* Battery Pack (7.4V–12V LiPo, high-capacity)
* Jumper Wires, Perfboard
* USB Cable (for programming)
* Computer with Python 3 (NumPy, SciPy, python-control)

---

## Installation & Setup

**Arduino IDE**:

1. Install Arduino IDE: [arduino.cc](https://www.arduino.cc/en/software)
2. Add ESP32 support (follow ESP32 Arduino setup instructions)
3. Install required libraries: `MPU6050` (Electronic Cats), `PID_v1` via Library Manager
4. Upload `ESP32_Balancing_Robot_WiFi_BTS7960.ino`

**Processing GUI**:

1. Install Processing: [processing.org/download](https://processing.org/download)
2. Add libraries: `controlP5`, `processing.net` via Contribution Manager
3. Update `PID_Tuner_WiFi.pde` with your ESP32 IP address

**Python AI PID Tuning**:

1. Install Python 3.9+: [python.org/downloads](https://www.python.org/downloads/)
2. Set up a virtual environment:


```bash
python3 -m venv control_env![20250813_190154_xi42kVpOFT](https://github.com/user-attachments/assets/9f2bdb2c-4491-42c0-8631-ef9eea49b7b7)

source control_env/bin/activate  # Windows: control_env\Scripts\activate
```

3. Install dependencies:


```bash
pip3 install numpy scipy matplotlib control
```

4. Run `ai_pid_tuner.py` to get optimized PID values. The script simulates an inverted pendulum and uses Differential Evolution to find the best `Kp`, `Ki`, `Kd`.

---

## Code Overview

**Arduino Sketch** (`O_range_Balancing_Robot_WiFi.ino`):

* Reads MPU6050 angles
* Performs PID balancing control
* Runs a WiFi TCP server for the Processing GUI
* Controls motors via BTS7960 (PWM)

> **Tip**: Update WiFi SSID and password before uploading

**Processing GUI** (`PID_Tuner_WiFi.pde`):

* Neon-style graphs for raw/smoothed tilt and PID output
* Sliders for `Kp`, `Ki`, `Kd`
* Status indicators for CoG, jitter, connection
* Communicates with ESP32 via WiFi TCP

> **Tip**: Update `esp32IP` with ESP32 IP

**AI PID Tuning** (`ai_pid_tuner.py`):

```python
import control
import numpy as np
from scipy.optimize import differential_evolution

g = 9.81
l = 0.3
J = 0.006

def cost(params):
    Kp, Ki, Kd = params
    try:
        A_closed = np.array([[0, -1, 0], [0, 0, 1], [Ki/J, g/l - Kp/J, -Kd/J]])
        B_closed = np.zeros((3,1))
        C_closed = np.eye(3)
        D_closed = np.zeros((3,1))
        sys_closed = control.ss(A_closed, B_closed, C_closed, D_closed)
        t = np.linspace(0,5,500)
        X0 = [0, 0.087, 0]
        t_out, y_out = control.forced_response(sys_closed, T=t, U=0, X0=X0)
        theta = y_out[1,:]
        if np.any(np.isnan(theta)) or np.max(np.abs(theta))>10:
            return 1e6
        itae = np.trapezoid(t*np.abs(theta), t)
        return itae
    except:
        return 1e6

bounds = [(0,200),(0,50),(0,100)]
result = differential_evolution(cost, bounds, maxiter=50, popsize=15)
print("Optimized PID: Kp=%.2f, Ki=%.2f, Kd=%.2f"%tuple(result.x))
```

> Example optimized PID: `Kp=200.00, Ki=0.00, Kd=1.64`. Update Arduino sketch and fine-tune using Processing GUI.

---

## Usage

1. Upload Arduino sketch to ESP32
2. Note ESP32 IP from Serial Monitor
3. Run Python AI PID script to generate optimized PID values
4. Open Processing GUI, set ESP32 IP, and monitor/tune live
5. Adjust physical parameters (`l`, `J`) in Python to match your robot

---

## Future Works

* Add motor encoders for velocity feedback
* Implement real-time ML on ESP32 (TensorFlow Lite)
* Extend GUI with remote drive controls

---

## References

* Optimize PID Controller using Genetic Algorithm for Robot Manipulators (arXiv, 2025)
* Auto-Tuning PID Controller Based on Genetic Algorithm (IntechOpen, 2023)
* PID Parameters Optimization Using Genetic Algorithm (arXiv, 2015)
* Tuning of PID Controller Parameters with Genetic Algorithm (ASCEE, 2021)
* Genetic Algorithm Tuned PID Controller for Process Control (ResearchGate, 2015)

---

## Credits

Thanks to Arduino, Processing, and SciPy communities, plus GA-PID research for inspiration.

---

## License

This project is licensed under the **MIT License**. Modify, distribute, and use with attribution.
