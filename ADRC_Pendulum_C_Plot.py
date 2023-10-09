import matplotlib.pyplot as plt
import pandas as pd
import math

# Read data from data.txt into a pandas DataFrame
data = pd.read_csv('data.txt', delim_whitespace=True, names=["Time", "Command", "Response", "Setpoint", "f"])

# Create figure
plt.figure()

# Plot response from C code
plt.subplot(3, 1, 1)
plt.plot(data["Time"], data["Response"]*180/math.pi, label=f"Response - from C Code")
plt.plot(data["Time"], data["Setpoint"]*180/math.pi, '--', label=f"Setpoint")
plt.ylabel(r"$\theta$ [deg]")
plt.legend()
plt.grid()

# Plot controller output
plt.subplot(3, 1, 2)
plt.plot(data["Time"], data["Command"], label=f"Command - from C Code")
plt.ylabel(r"$\tau$ [Nm]")
plt.legend()
plt.grid()

# Plot controller output
plt.subplot(3, 1, 3)
plt.plot(data["Time"], data["f"], label=f"f(t) - from C Code")
plt.xlabel("Time [s]")
plt.ylabel("f(t)")
plt.legend()
plt.grid()

# Show plots
plt.show()