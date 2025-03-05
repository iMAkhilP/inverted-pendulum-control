import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint

# Pendulum parameters
b = 0.1     # Damping coefficient (kg*m^2/s)
m = 1.0      # Mass of pendulum (in Kg)
L = 0.5     # Length of the pendulum (in meters)
g = 9.81         # Acceleration due to gravity (in m/s^2)

# Initial conditions
theta0 = np.pi/1.2 # Initial angle (in radians)
omega0 = 0.0     # Initial angular velocity (in rad/s)

# Time array
dt = 0.01        # Time step (in seconds)
total_time = 20  # Total simulation time (in seconds)
time = np.arange(0, total_time, dt)

# Damping factor calculation
beta = b/(m * L**2)

def pendulum_dynamics(state, time):
    
    theta, omega = state

    dtheta_dt = omega
    domega_dt = -(g/L) * np.sin(theta) - beta * omega

    return [dtheta_dt, domega_dt]

intial_state = [theta0, omega0]

solution = odeint(pendulum_dynamics, intial_state, time)

theta = solution[:, 0]
omega = solution[:, 1]

# Convert to Cartesian coordinates for visualization
x = L * np.sin(theta)    # Horizontal position
y = -L * np.cos(theta)   # Vertical position (negative because y increases downward in our convention)

# Create a figure with two subplots
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 12))

# Plot the angle over time
ax1.plot(time, theta, 'b-', label='θ(t)')
ax1.set_xlabel('Time (s)')
ax1.set_ylabel('Angle (rad)')
ax1.set_title('Damped Pendulum - Angle vs Time')
ax1.grid(True)
ax1.legend()

# Plot the phase space (angle vs. angular velocity)
ax2.plot(theta, omega, 'r-', label='Phase Space')
ax2.set_xlabel('Angle θ (rad)')
ax2.set_ylabel('Angular Velocity ω (rad/s)')
ax2.set_title('Damped Pendulum - Phase Space')
ax2.grid(True)
ax2.legend()

plt.tight_layout()
plt.show()

from matplotlib.animation import FuncAnimation

def animate_pendulum():
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_xlim(-L*1.2, L*1.2)
    ax.set_ylim(-L*1.2, L*1.2)
    ax.set_aspect('equal')
    ax.grid(True)
    
    # Plot elements
    line, = ax.plot([], [], 'o-', lw=2, markersize=10)
    time_text = ax.text(0.05, 0.95, '', transform=ax.transAxes)
    energy_text = ax.text(0.05, 0.90, '', transform=ax.transAxes)
    
    def init():
        line.set_data([], [])
        time_text.set_text('')
        energy_text.set_text('')
        return line, time_text, energy_text
    
    def update(frame):
        # Calculate the position of the pendulum
        x_pos = x[frame]
        y_pos = y[frame]
        
        # Update the line data
        line.set_data([0, x_pos], [0, y_pos])
        
        # Calculate kinetic and potential energy
        KE = 0.5 * m * L**2 * omega[frame]**2
        PE = m * g * L * (1 - np.cos(theta[frame]))
        total_energy = KE + PE
        
        # Update text
        time_text.set_text(f'Time: {time[frame]:.2f} s')
        energy_text.set_text(f'Energy: {total_energy:.2f} J')
        
        return line, time_text, energy_text
    
    ani = FuncAnimation(fig, update, frames=len(time), init_func=init, blit=True, interval=dt*1000)
    
    plt.title('Damped Pendulum Simulation')
    plt.show()
    
    return ani

# Create the animation
ani = animate_pendulum()

