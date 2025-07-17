# 🌀 Inverted Pendulum on a Cart – Simulation and Control

This repository implements a **non-linear simulation** of the classic **inverted pendulum on a cart** system. It features two-layered control:
1. **Swing-Up Control** to bring the pendulum from the downward position to upright,
2. **LQR Control** to stabilize the pendulum around the upright position.

The simulation visualizes the motion and exports an animation of the full trajectory.

---

## 📌 Problem Statement

Controlling an inverted pendulum on a cart is a benchmark problem in control theory. The system is:
- **Non-linear**, due to the pendulum's rotational dynamics.
- **Underactuated**, as it uses only one actuator (cart force) to control two degrees of freedom (cart position and pendulum angle).

**Goal:** Design a control strategy to swing up the pendulum from its hanging position and stabilize it in the inverted position using a motor-actuated cart.

---

## 🧠 Control Strategy

### 1. Swing-Up Control (Energy-Based)
When the pendulum is far from upright, we apply an **energy-based controller**:
- Compute the **energy error** between current and target energy (upright).
- Apply a cart acceleration that pumps energy into the pendulum to help it reach the upright position.
- The swing direction is determined by the sign of angular velocity × cos(angle).

### 2. LQR Stabilization
Once the pendulum is sufficiently upright (within ±π/5 radians), we switch to **Linear Quadratic Regulator (LQR)**:
- The system is linearized around the upright equilibrium.
- The LQR gain matrix `K` is computed by solving the Riccati equation using specified weight matrices `Q` and `R`.
- Control input: `u = -Kx` for state feedback stabilization.

---

## 🛠️ Implementation Details

### 🔁 Dynamics (in `pendulum.py`)
- Nonlinear ODEs derived from Newton-Euler equations.
- Solved using `scipy.integrate.solve_ivp`.
- Implements both swing-up and LQR control in real-time.

### 🎬 Visualization and Animation (in `simulation.py`)
- Plots time-series of:
  - Cart position (`x`) vs time
  - Pendulum angle (`θ`) vs time
- Animates cart-pendulum motion with dynamic plotting using `matplotlib`.
- Saves animation frames and compiles them into `out.mp4` using `ffmpeg`.

---

## 🧮 System Parameters

| Parameter       | Value   | Description                          |
|----------------|---------|--------------------------------------|
| `M`            | 0.6 kg  | Mass of cart                         |
| `m`            | 0.3 kg  | Mass of pendulum                     |
| `l`            | 0.3 m   | Distance to center of mass of pendulum |
| `I`            | 0.006 kg·m² | Moment of inertia of pendulum  |
| `g`            | 9.81 m/s² | Gravity                            |
| `R`, `Km`, `Kg`, `r` | Motor/drive constants | Used for voltage-force relation |

---

## 📂 File Overview

| File           | Purpose |
|----------------|---------|
| `pendulum.py`  | Defines pendulum dynamics, swing-up controller, LQR setup, and numerical integration |
| `simulation.py`| Runs the simulation, generates plots and animation using `matplotlib` and `ffmpeg` |

---

## ▶️ How to Run

1. Install dependencies:
```bash
pip install numpy scipy matplotlib control
