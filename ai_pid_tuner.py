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
