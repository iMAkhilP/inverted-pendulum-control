import numpy as np
from control.matlab import lqr
from scipy.integrate import solve_ivp

# Constants
M = .6  # mass of cart+pendulum
m = .3  # mass of pendulum
Km = 2  # motor torque constant
Kg = .01  # gear ratio
R = 6  # armiture resistance
r = .01  # drive radius
K1 = Km*Kg/(R*r)
K2 = Km**2*Kg**2/(R*r**2)
l = .3  # length of pendulum to CG
I = 0.006  # inertia of the pendulum
L = (I + m*l**2)/(m*l)
g = 9.81  # gravity
Vsat = 20.  # saturation voltage

# System matrices
A11 = -1 * Km**2*Kg**2 / ((M - m*l/L)*R*r**2)
A12 = -1*g*m*l / (L*(M - m*l/L))
A31 = Km**2*Kg**2 / (M*(L - m*l/M)*R*r**2)
A32 = g/(L-m*l/M)

A = np.array([
    [0, 1, 0, 0],
    [0, A11, A12, 0],
    [0, 0, 0, 1],
    [0, A31, A32, 0]
])

B1 = Km*Kg/((M - m*l/L)*R*r)
B2 = -1*Km*Kg/(M*(L-m*l/M)*R*r)

B = np.array([
    [0],
    [B1],
    [0],
    [B2]
])
Q = np.array([
    [10000, 0, 0, 0],
    [0, 1, 0, 0],
    [0, 0, 10000, 0],
    [0, 0, 0, 1]
])

R_control = np.array([[2]])
(K, X, E) = lqr(A, B, Q, R_control)

def constrain(theta):
    theta = theta % (2*np.pi)
    if theta > np.pi:
        theta = -2*np.pi+theta
    return theta

def sat(Vsat, V):
    if abs(V) > Vsat:
        # Replace cmp with the Python 3 equivalent
        _ = np.sign(V)
        return Vsat * _
    return V

theta = []
class Pendulum(object):
    def __init__(self, dt, init_conds, end):
        self.dt = dt
        self.t = 0.0
        self.x = init_conds[:]
        self.end = end

    def derivative(self, t, u):
        V = sat(Vsat, self.control(u))
        #x1 = x, x2 = x_dt, x3 = theta, x4 = theta_dt
        x1, x2, x3, x4 = u
        x1_dt, x3_dt = x2, x4
        x2_dt = (K1*V - K2*x2 - m*l*g*np.cos(x3)*np.sin(x3)/L + m*l*np.sin(x3)*x4**2) / (M - m*l*np.cos(x3)**2/L)
        x4_dt = (g*np.sin(x3) - m*l*x4**2*np.cos(x3)*np.sin(x3)/L - np.cos(x3)*(K1*V + K2*x2)/M) / (L - m*l*np.cos(x3)**2/M)
        x = [x1_dt, x2_dt, x3_dt, x4_dt]
        return x

    def control(self, u):
        u = np.asarray(u)
        c = constrain(u[2])
        if c > -np.pi/5 and c < np.pi/5:
            control_input = np.dot(-K, np.array([u[0], u[1], c, u[3]]).T)
            return control_input.item()
        else:
            return self.swing_up(u)

    def swing_up(self, u):
        E0 = 0.
        k = 1
        w = (m*g*l/(4*I))**(.5)
        E = m*g*l*(.5*(u[3]/w)**2 + np.cos(u[2])-1)
        # Replace cmp with the Python 3 equivalent
        direction = np.sign(u[3]*np.cos(u[2]))
        a = k*(E-E0)*direction
        F = M*a
        V = (F - K2*constrain(u[2]))/K1
        return sat(Vsat, V)

    def integrate(self):
        def wrapped_derivative(t, y):
            return self.derivative(t, y)
        
        sol = solve_ivp(wrapped_derivative, [0, self.end], self.x, t_eval=np.arange(0, self.end, self.dt))
        theta.extend([constrain(state[2]) for state in sol.y.T])
        return np.column_stack((sol.t, sol.y.T))