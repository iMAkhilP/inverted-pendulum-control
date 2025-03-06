import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp

#cart-pendulum model
M = 1.0     # mass of cart (kg)
m = 0.5     # mass of pendulum (kg)
g = 9.81    # gravitational acceleration (m/s^2)
l = 1.0     # length of pendulum (m)
b_c = 0.35  # Damping coefficient for cart (kg*m^2/s)
b_c = 0.1   # Damping coefficient for pendulum (kg*m^2/s)
# x             -   displacement of cart
# theta         -   angular displacement of cart
# x_dot         -   velocity of cart
# theta_dot     -   angular velocity of pendulum
# x_Ddot        -   acceleration of cart
# theta_Ddot    -   angular acceleration of pendulum

# simulation parameter
U = lambda time: 0.0    -   # force (N) acting on cart (control force) is function of time

def cart_pendulum(state, time):
    x, dx, theta, dtheta = state
    
    A = np.array([M+m, -m*l*np.cos(theta)],
                 [-m*l*np.cos(theta), m*l**2])
    
    b = np.array([(U - (m*l*np.sin(theta)*dtheta**2) - (b_c*dx)),
                  ((m*g*l*np.sin(theta)) - (b_p*dtheta))])
    
    ddx, ddtheta = np.linalg.solve(A,B)
    
    return [dx, ddx, dtheta, ddtheta]

initial_state = [0, 0, np.pi/4, 0.5]