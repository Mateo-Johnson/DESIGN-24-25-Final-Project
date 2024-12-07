import numpy as np
import matplotlib.pyplot as plt

def smooth_thrust_profile(t):
    """
    Smooth thrust profile with quadratic transitions.
    """
    if 0 <= t < 0.2
        return 14.5 * (t / 0.2)**2
    elif 0.2 <= t <= 0.35:

        a = -10.2 / (0.15**2)  
        return 14.5 + a * (t - 0.2)**2
    elif 0.35 < t <= 1.8:
        return 4.3
    elif 1.8 < t <= 1.85:
        a = -4.3 / (0.05**2)
        return 4.3 + a * (t - 1.8)**2
    else:
        return 0

time = np.linspace(0, 2, 500)
smooth_thrust = [smooth_thrust_profile(t) for t in time]

plt.figure(figsize=(10, 6))
plt.plot(time, smooth_thrust, label='Smooth Thrust Profile', color='blue')
plt.title("Thrust Profile of C6 Engine (Smoothed with Quadratic Transitions)")
plt.xlabel("Time (s)")
plt.ylabel("Thrust (N)")
plt.grid(True)
plt.legend()
plt.show()
