
# Story
I built this robot named O_Range to explore control theory and IoT on a budget. Initially, manual PID tuning via sliders was effective but time-consuming. Switching to BTS7960 drivers added power (up to 43A), and WiFi freed me from USB cables. The real breakthrough was AI-based tuning: I modeled the robot as an inverted pendulum in Python, using SciPy's Differential Evolution to find optimal PID values (e.g., Kp ≈ 200, Ki ≈ 0, Kd ≈ 1.6). These stabilized the simulation, and with minor tweaks, the real robot balanced smoothly. The Processing GUI lets me monitor tilt and fine-tune wirelessly, making iterations fast and fun!

![O_Range balancing Bot](https://hackster.imgix.net/uploads/attachments/1874478/_qXWhfp0v36.blob?auto=compress%2Cformat&w=900&h=675&fit=min)

Hardware: ESP32 DevKit, MPU6050 IMU, BTS7960 drivers, two DC motors.
Wireless Control: WiFi-based communication with a Processing GUI for real-time monitoring and PID tuning.
AI Tuning: Python script using Differential Evolution to optimize PID parameters in a simulated inverted pendulum.
GUI: Sci-fi-themed Processing interface with neon graphs, sliders, and status indicators.
Scalable: Add encoders, Bluetooth, or real-time ML for advanced control.

# Materials

ESP32 DevKit V1 (or similar)
MPU6050 (Gyroscope + Accelerometer IMU)
2x BTS7960 Motor Driver Modules (high-current H-bridges)
2x DC Motors (geared for torque, 12V, encoders optional)
Chassis (3D-printed or acrylic, two-wheel design)
2x Wheels (rubber-tread for grip)
Battery Pack (7.4V–12V LiPo, high-capacity)
Jumper Wires, Perfboard (for secure connections)
USB Cable (for initial programming)
Computer with Python 3 (for AI tuning, with NumPy, SciPy, python-control)


# Installation
Arduino IDE

Install Arduino IDE: https://www.arduino.cc/en/software
Add ESP32 support: Follow ESP32 Arduino setup.
Install libraries: MPU6050 (Electronic Cats), PID_v1 via Library Manager.
Upload ESP32_Balancing_Robot_WiFi_BTS7960.ino.

# Processing

Install Processing: https://processing.org/download
Add libraries: controlP5, processing.net via Contribution Manager.
Update PID_Tuner_WiFi.pde with ESP32’s IP address.

# Python (AI Tuning)

Install Python 3.9+: https://www.python.org/downloads/
Set up a virtual environment:python3 -m venv control_env
source control_env/bin/activate  # Windows: control_env\Scripts\activate


Install dependencies:pip3 install numpy scipy matplotlib control


Run ai_pid_tuner.py to get optimized PID values.

Code
Arduino Sketch
Located in O_range_Balancing_Robot_WiFi.ino. Handles:

-MPU6050 angle reading
-PID control for balancing
-WiFi TCP server for Processing GUI
-BTS7960 motor control (PWM)

Update ssid and password before uploading.
Processing GUI
In PID_Tuner_WiFi.pde. Features:

- Neon graphs for raw tilt, smoothed tilt, PID output
- Sliders for Kp, Ki, Kd
- Status indicators (CoG, jitter, connection)
- WiFi communication via TCP

Update esp32IP with your ESP32’s IP.
AI PID Tuning
In ai_pid_tuner.py. Simulates an inverted pendulum and optimizes PID using Differential Evolution.
```
import control
import numpy as np
from scipy.optimize import differential_evolution

g = 9.81  # Gravity
l = 0.3   # Pendulum length (m)
J = 0.006 # Moment of inertia (kg m^2)

def cost(params):
    Kp, Ki, Kd = params
    try:
        A_closed = np.array([
            [0, -1, 0],
            [0, 0, 1],
            [Ki / J, g / l - Kp / J, -Kd / J]
        ])
        B_closed = np.zeros((3, 1))
        C_closed = np.eye(3)
        D_closed = np.zeros((3, 1))
        sys_closed = control.ss(A_closed, B_closed, C_closed, D_closed)
        
        t = np.linspace(0, 5, 500)
        X0 = [0, 0.087, 0]  # Initial: integral=0, theta=5° (rad), dtheta=0
        
        t_out, y_out = control.forced_response(sys_closed, T=t, U=0, X0=X0)
        
        theta = y_out[1, :]  # Theta state
        
        if np.any(np.isnan(theta)) or np.max(np.abs(theta)) > 10:
            return 1e6
        
        itae = np.trapz(t * np.abs(theta), t)
        return itae
    except:
        return 1e6

bounds = [(0, 200), (0, 50), (0, 100)]  # Kp, Ki, Kd ranges
result = differential_evolution(cost, bounds, maxiter=50, popsize=15)

print("Optimized PID: Kp=%.2f, Ki=%.2f, Kd=%.2f" % tuple(result.x))
```

Run to get PID values (e.g., Kp=200.00, Ki=0.00, Kd=1.64). Update Arduino sketch and fine-tune with Processing GUI.
Usage

Upload Arduino sketch to ESP32.
Note ESP32’s IP from Serial Monitor.
Run Python script to get AI-optimized PID values; update Arduino code.
Open Processing GUI, set ESP32 IP, and monitor/tune live.
Adjust model params (l, J) in Python script to match your robot’s physics.

Control Theory Papers on GA-PID Tuning

Optimize PID Controller using Genetic Algorithm for Robot Manipulators (arXiv, 2025)
Auto-Tuning PID Controller Based on Genetic Algorithm (IntechOpen, 2023)
PID Parameters Optimization by Using Genetic Algorithm (arXiv, 2015)
Tuning of PID Controller Parameters with Genetic Algorithm (ASCEE, 2021)
Genetic Algorithm Tuned PID Controller for Process Control (ResearchGate, 2015)

SciPy Developers
SciPy is a community-driven project. Key contributors include Travis Oliphant and Pauli Virtanen. See:

GitHub Contributors
SciPy Website
SciPy Mailing List

#future works

- Add motor encoders for velocity feedback.
- Implement real-time ML on ESP32 (e.g., TensorFlow Lite).
- Extend GUI with remote drive controls.

Credits
Thanks to Arduino, Processing, and SciPy communities, plus control theory researchers for GA-PID insights.
