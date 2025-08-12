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
