import numpy as np
import matplotlib.pyplot as plt

def thrust_profile(t):
    """
    Composite function defining the thrust profile.
    """
    if t < 0.2:
        return (14.5 / 0.2) * t 
    elif 0.2 <= t <= 0.35:
        return 14.5
    elif 0.35 < t <= 1.8:
        return 4.3
    elif 1.8 < t <= 1.85:
        return 4.3 - (4.3 / 0.05) * (t - 1.8)  
    else:
        return 0 

time = np.linspace(0, 2, 500)
thrust = [thrust_profile(t) for t in time]

plt.figure(figsize=(10, 6))
plt.plot(time, thrust, label='Composite Thrust Profile', color='blue')
plt.title("Thrust Profile of C6 Engine (Composite Function)")
plt.xlabel("Time (s)")
plt.ylabel("Thrust (N)")
plt.grid(True)
plt.legend()

plt.savefig("thrust_profile.png")
print("Plot saved as 'thrust_profile.png'. Check the file explorer in Replit to view it.")
