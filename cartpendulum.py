import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from scipy.integrate import solve_ivp

#cart-pendulum model
M = 1.0     # mass of cart (kg)
m = 0.5     # mass of pendulum (kg)
g = 9.81    # gravitational acceleration (m/s^2)
l = 1.0     # length of pendulum (m)
b_c = 0.35  # Damping coefficient for cart (kg*m^2/s)
b_p = 0.7   # Damping coefficient for pendulum (kg*m^2/s)
# x             -   displacement of cart
# theta         -   angular displacement of cart
# x_dot         -   velocity of cart
# theta_dot     -   angular velocity of pendulum
# x_Ddot        -   acceleration of cart
# theta_Ddot    -   angular acceleration of pendulum

# Time array
dt = 0.01
t_span = (0, 60)
time_points = np.arange(0, 60, dt)

# simulation parameter
U = lambda time: 0.0    # force (N) acting on cart (control force) is function of time

def cart_pendulum(t, state):
    x, dx, theta, dtheta = state
    
    A = np.array([[M+m, m*l*np.cos(theta)],
                [m*l*np.cos(theta), m*l**2]])
    
    b = np.array([(U(t) - (m*l*np.sin(theta)*dtheta**2) - (b_c*dx)),
                ((-m*g*l*np.sin(theta)) - (2*m*dx*dtheta*l*np.sin(theta)) - (b_p*dtheta))])
    
    ddx, ddtheta = np.linalg.solve(A,b)
    
    return [dx, ddx, dtheta, ddtheta]

initial_state = [0, 0, -20, 0]

solution = solve_ivp(cart_pendulum, t_span=t_span, y0=initial_state, t_eval=time_points)

# Extract results for plotting or analysis
time = solution.t
x = solution.y[0]
theta = solution.y[2]

# Plot results
import matplotlib.pyplot as plt

plt.figure(figsize=(12,6))
plt.plot(time, x, label='Cart Position')
plt.plot(time, theta, label='Pendulum Angle')
plt.xlabel('Time (s)')
plt.ylabel('State Variables')
plt.legend()
plt.grid()
plt.show()

# Create animation
fig, ax = plt.subplots(figsize=(10, 6))
ax.set_xlim(-3, 3)
ax.set_ylim(-2, 2)
ax.grid()

# Create objects to animate
cart_width, cart_height = 0.4, 0.2
cart = plt.Rectangle((-cart_width/2, -cart_height/2), cart_width, cart_height, fc='b')
pendulum, = plt.plot([], [], 'o-', lw=2, color='k')
time_text = ax.text(0.02, 0.95, '', transform=ax.transAxes)
theta_text = ax.text(0.02, 0.90, '', transform=ax.transAxes)
position_text = ax.text(0.02, 0.85, '', transform=ax.transAxes)

def init():
    ax.add_patch(cart)
    time_text.set_text('')
    theta_text.set_text('')
    position_text.set_text('')
    pendulum.set_data([], [])
    return cart, pendulum, time_text, theta_text, position_text

def animate(i):
    cart_x = x[i]
    cart_y = 0
    cart.set_xy((cart_x - cart_width/2, cart_y - cart_height/2))
    
    # Calculate pendulum position
    pendulum_x = [cart_x, cart_x + l * np.sin(theta[i])]
    pendulum_y = [cart_y, cart_y - l * np.cos(theta[i])]
    pendulum.set_data(pendulum_x, pendulum_y)
    
    time_text.set_text(f'Time: {time[i]:.1f}s')
    theta_text.set_text(f'θ: {theta[i]:.2f} rad')
    position_text.set_text(f'x: {cart_x:.2f} m')

    return cart, pendulum, time_text, theta_text, position_text

# Create animation (use fewer frames for smoother performance)
frames = len(time)
if frames > 200:  # Reduce number of frames if too many
    step = len(time) // 200
    frames = range(0, len(time), step)
else:
    frames = range(len(time))

ani = FuncAnimation(fig, animate, frames=frames, init_func=init, 
                    blit=True, interval=dt*1000, repeat=False)

plt.title('Cart-Pendulum Animation')
plt.xlabel('Position (m)')
plt.ylabel('Height (m)')
plt.show()
