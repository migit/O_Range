
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Open Source Mobile Robot</title>
<style>
  body {
    margin: 0;
    font-family: 'Share Tech Mono', monospace;
    background-color: #0a0a0a;
    color: #ff3c3c;
    height: 100vh;
    display: grid;
    grid-template-columns: 20% 60% 20%;
    grid-template-rows: auto 1fr auto;
    overflow: hidden;
  }
  .grid-bg {
    position: fixed;
    top: 0; left: 0;
    width: 100%; height: 100%;
    background: linear-gradient(90deg, rgba(255,60,60,0.08) 1px, transparent 1px),
                linear-gradient(rgba(255,60,60,0.08) 1px, transparent 1px);
    background-size: 40px 40px;
    z-index: -1;
  }
  header {
    grid-column: span 3;
    text-align: center;
    padding: 1rem;
    border-bottom: 2px solid #ff7a00;
    text-shadow: 0 0 20px #ff7a00;
  }
  header h1 {
    margin: 0;
    font-size: 2rem;
    letter-spacing: 3px;
    animation: flicker 2s infinite;
  }
  header small {
    display: block;
    font-size: 0.7rem;
    color: #ffbb6c;
    margin-bottom: 0.3rem;
  }
  .panel {
    padding: 1rem;
    border: 2px solid #ff7a00;
    margin: 0.5rem;
    background: rgba(10,10,10,0.85);
    border-radius: 8px;
    box-shadow: 0 0 20px #ff7a00 inset;
    overflow-y: auto;
  }
  .left, .right {
    font-size: 0.9rem;
  }
  .terminal {
    height: calc(50% - 1rem);
    font-size: 1rem;
    line-height: 1.4;
    overflow-y: auto;
    position: relative;
    padding-right: 10px;
  }
  .prompt { color: #ff7a00; }
  .cursor {
    display: inline-block;
    width: 10px;
    height: 1em;
    background: #ff7a00;
    margin-left: 5px;
    animation: blink 1s infinite;
  }
  @keyframes blink {
    0%, 50% { opacity: 1; }
    51%, 100% { opacity: 0; }
  }
  .links a {
    display: block;
    margin: 0.5rem 0;
    padding: 0.5rem;
    border: 1px solid #ff7a00;
    color: #ff7a00;
    text-decoration: none;
    border-radius: 4px;
    text-align: center;
    transition: 0.2s;
  }
  .links a:hover {
    background: #ff7a00;
    color: #0a0a0a;
    box-shadow: 0 0 20px #ff7a00;
  }
  .robot {
    width: 160px;
    margin: 1rem auto;
    display: block;
    filter: drop-shadow(0 0 15px #ff7a00);
  }
  .widgets {
    display: flex;
    justify-content: space-around;
    margin-top: 1rem;
  }
  .widget {
    flex: 1;
    margin: 0 0.5rem;
    padding: 0.5rem;
    border: 1px solid #ff7a00;
    border-radius: 6px;
    background: rgba(10,10,10,0.9);
    color: #ffbb6c;
    text-align: center;
    font-size: 0.8rem;
  }
  .widget canvas {
    width: 100%;
    height: 80px;
  }
  footer {
    grid-column: span 3;
    text-align: center;
    padding: 0.5rem;
    border-top: 2px solid #ff7a00;
    font-size: 0.8rem;
    color: #ffbb6c;
  }
  @keyframes flicker {
    0% { opacity: 1; }
    40% { opacity: 0.9; }
    50% { opacity: 0.5; }
    60% { opacity: 0.9; }
    100% { opacity: 1; }
  }
</style>
</head>
<body>
  <div class="grid-bg"></div>

  <header>
    <small>OPEN SOURCE PROJECT</small>
    <h1>Open Source Mobile Robot</h1>
    <small>eDEX-UI inspired futuristic interface • Red/Orange Theme</small>
  </header>

  <div class="panel left">
    <h3>System Info</h3>
    <small>CPU & Memory</small>
    <p>CPU: 12% usage</p>
    <p>RAM: 68% usage</p>
    <small>Network</small>
    <p>Connected</p>
    <p>IP: 192.168.0.42</p>
    <small>Uptime</small>
    <p>03:22:15</p>
  </div>

  <div class="panel">
    <div class="terminal" id="terminal">
      <p><span class="prompt">$</span> Booting robot OS...</p>
      <p><span class="prompt">$</span> Sensors online ✓</p>
      <p><span class="prompt">$</span> Motion drivers engaged ✓</p>
      <p><span class="prompt">$</span> Neural core active ✓</p>
      <p><span class="prompt">$</span> Awaiting commands... <span class="cursor"></span></p>
    </div>

    <div class="widgets">
      <div class="widget"><small>Battery</small><canvas id="batteryChart"></canvas></div>
      <div class="widget"><small>Temperature</small><canvas id="tempChart"></canvas></div>
      <div class="widget"><small>Distance</small><canvas id="distanceChart"></canvas></div>
    </div>

    <img src="https://upload.wikimedia.org/wikipedia/commons/thumb/3/3e/Robot-car-icon.svg/512px-Robot-car-icon.svg.png" alt="Wheeled Robot" class="robot">
  </div>

  <div class="panel right">
    <h3>Project Links</h3>
    <small>Quick Access</small>
    <div class="links">
      <a href="#">GitHub</a>
      <a href="#">Docs</a>
      <a href="#">Community</a>
    </div>
  </div>

  <footer>
    Open Source Mobile Robot • Futuristic Interface • eDEX-UI Inspired
  </footer>

  <script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
  <script>
    // Terminal logs animation
    const terminal = document.getElementById('terminal');
    const logs = [
      "$ Running diagnostics...",
      "$ AI navigation online.",
      "$ Mapping subsystems calibrated.",
      "$ Ready for deployment."
    ];
    let i = 0;
    setInterval(() => {
      if(i < logs.length) {
        const p = document.createElement('p');
        p.innerHTML = `<span class=\"prompt\">$</span> ${logs[i]}`;
        terminal.appendChild(p);
        terminal.scrollTop = terminal.scrollHeight;
        i++;
      }
    }, 2500);

    // Charts
    const batteryCtx = document.getElementById('batteryChart').getContext('2d');
    new Chart(batteryCtx, {
      type: 'doughnut',
      data: { labels:['Used','Free'], datasets:[{data:[40,60], backgroundColor:['#ff3c3c','#ffbb6c']}] },
      options: { responsive:true, plugins:{legend:{display:false}} }
    });

    const tempCtx = document.getElementById('tempChart').getContext('2d');
    new Chart(tempCtx, {
      type: 'line',
      data: { labels:['1','2','3','4','5'], datasets:[{data:[36,37,36,38,37], borderColor:'#ff7a00', fill:false}] },
      options: { responsive:true, plugins:{legend:{display:false}}, scales:{y:{min:30,max:40}} }
    });

    const distanceCtx = document.getElementById('distanceChart').getContext('2d');
    new Chart(distanceCtx, {
      type: 'bar',
      data: { labels:['Front','Left','Right'], datasets:[{data:[120,80,90], backgroundColor:'#ff3c3c'}] },
      options: { responsive:true, plugins:{legend:{display:false}}, scales:{y:{min:0,max:150}} }
    });
  </script>
</body>
</html>




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

# Physical parameters for inverted pendulum model
g = 9.81  # Gravity (m/s^2)
l = 0.3   # Pendulum length (m, adjust to match your robot's CoG height)
J = 0.006 # Moment of inertia (kg m^2, approximate for robot)

def cost(params):
    Kp, Ki, Kd = params
    try:
        # State-space model: [integral error, theta, dtheta]
        A_closed = np.array([
            [0, -1, 0],
            [0, 0, 1],
            [Ki / J, g / l - Kp / J, -Kd / J]
        ])
        B_closed = np.zeros((3, 1))
        C_closed = np.eye(3)
        D_closed = np.zeros((3, 1))
        sys_closed = control.ss(A_closed, B_closed, C_closed, D_closed)
        
        # Simulate response
        t = np.linspace(0, 5, 500)
        X0 = [0, 0.087, 0]  # Initial: integral=0, theta=5° (rad), dtheta=0
        
        t_out, y_out = control.forced_response(sys_closed, T=t, U=0, X0=X0)
        
        theta = y_out[1, :]  # Theta state (angle in radians)
        
        # Check for instability
        if np.any(np.isnan(theta)) or np.max(np.abs(theta)) > 10:
            return 1e6
        
        # Compute ITAE using trapezoid rule
        itae = np.trapezoid(t * np.abs(theta), t)
        return itae
    except:
        return 1e6

# Optimization bounds for Kp, Ki, Kd
bounds = [(0, 200), (0, 50), (0, 100)]
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
