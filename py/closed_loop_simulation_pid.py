import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt

class PIDController:
    def __init__(self, Kp, Ki, Kd, setpoint, dt):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.dt = dt
        self.integral = 0
        self.prev_error = 0

    def update(self, measurement):
        error = self.setpoint - measurement
        self.integral += error * self.dt
        derivative = (error - self.prev_error) / self.dt
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.prev_error = error
        return output

    def reset(self):
        self.integral = 0
        self.prev_error = 0


def heat_transfer_simulation_with_pid(R_s, r_s, t, pid_controller, h_s_initial=43, h_y_initial=43, k_s=50, rho_s=7800,
                                      c_s=500, rho_y=2700, c_y=900, k_rubber=0.15, T_infinity=100, T_initial=30,
                                      t_end=2000, w=[0.5, 0.5]):
    """
    Perform a closed-loop simulation of heat transfer with PID control between a steel ball and an aluminum yoke with a rubber silicone lining and a step input.

    Parameters:
    R_s (float): Outer radius of the steel ball (in meters)
    r_s (float): Inner radius of the steel ball (in meters)
    t (float): Radial thickness of the aluminum yoke (in meters)
    pid_controller (PIDController): Instance of the PIDController class
    h_s_initial (float): Initial convective heat transfer coefficient for the steel ball (W/m^2K, default: 43)
    h_y_initial (float): Initial convective heat transfer coefficient for the aluminum yoke (W/m^2K, default: 43)
    k_s (float): Thermal conductivity of steel (W/mK)
    rho_s (float): Density of steel (kg/m^3)
    c_s (float): Specific heat capacity of steel (J/kgK)
    rho_y (float): Density of aluminum (kg/m^3)
    c_y (float): Specific heat capacity of aluminum (J/kgK)
    k_rubber (float): Thermal conductivity of the rubber silicone lining (W/mK, default: 0.15)
    T_infinity (float): Initial temperature of the surrounding air heated by the convective heater (default: 100°C)
    T_initial (float): Initial temperature of both the steel ball and aluminum yoke (default: 30°C)
    t_end (float): Simulation end time (in seconds, default: 1000 seconds)
    w (list of float): relative weights assigned to the measurements used in PID error tracking signal

    Returns:
    solution: The solution of the differential equations (time and temperatures)
    control_effort: The control effort required by the heater blower
    """

    # Reset the PID controller
    pid_controller.reset()

    # Volume and surface area calculations for the steel ball
    V_s = (4 / 3) * np.pi * (R_s ** 3 - r_s ** 3)
    A_conv_s = 2 * np.pi * R_s ** 2
    A_cond = 2 * np.pi * R_s ** 2

    # Volume and surface area calculations for the aluminum yoke (bowl)
    V_y = (2 / 3) * np.pi * ((R_s + t) ** 3 - R_s ** 3)

    A_conv_y = 2 * np.pi * (R_s + t) ** 2

    # Thickness of the rubber silicone lining
    t_rubber = 0.005  # 5mm in meters

    # Time vector
    t_eval = np.linspace(0, t_end, 100)
    dt = t_eval[1] - t_eval[0]

    # Initialize control effort array
    control_effort = np.zeros_like(t_eval)

    # Differential equations
    def model(t, T):
        T_s, T_y = T
        T_avg = w[0] * T_s + w[1] * T_y

        # Update PID controller
        control_signal = pid_controller.update(T_avg)
        T_infinity = control_signal

        dT_s_dt = (h_s_initial * A_conv_s * (T_infinity - T_s) - k_rubber * A_cond * (T_s - T_y) / t_rubber) / (
                    rho_s * c_s * V_s)
        dT_y_dt = (k_rubber * A_cond * (T_s - T_y) / t_rubber + h_y_initial * A_conv_y * (T_infinity - T_y)) / (
                    rho_y * c_y * V_y)

        return [dT_s_dt, dT_y_dt]

    # Initial conditions
    T_initial = [T_initial, T_initial]

    # Solve the differential equations with control
    solution = solve_ivp(model, [0, t_end], T_initial, t_eval=t_eval, method='RK45')

    pid_controller.reset()

    # Calculate control effort at each time step
    for i, t in enumerate(t_eval):
        T_s, T_y = solution.y[:, i]
        T_avg = (T_s + T_y) / 2
        control_effort[i] = pid_controller.update(T_avg)

    return solution, control_effort

def simulate_closed_loop_with_pid():
    # Define the PID controller
    setpoint = 70  # Desired reference temperature
    pid = PIDController(Kp=10.0, Ki=0.45, Kd=0.01, setpoint=setpoint, dt=0.1)

    # Simulate the closed-loop response
    solution, control_effort = heat_transfer_simulation_with_pid(
        R_s=0.05,  # 50mm outer radius
        r_s=0.0,  # Solid steel ball
        t=0.01,  # 10mm radial thickness of yoke
        pid_controller=pid,
        t_end=1000,
        w=[0.5,0.5]
    )

    # Extract data for plotting
    t_eval = solution.t
    T_s = solution.y[0]
    T_y = solution.y[1]

    # Plot the results
    plt.figure(figsize=(10, 6))

    # Temperature responses
    plt.subplot(2, 1, 1)
    plt.plot(t_eval, T_s, label='Steel Ball Temperature (T_s)')
    plt.plot(t_eval, T_y, label='Aluminum Yoke Temperature (T_y)')
    plt.axhline(setpoint, color='k', linestyle='--', label='Setpoint')
    plt.xlabel('Time (s)')
    plt.ylabel('Temperature (°C)')
    plt.title('Closed-Loop Temperature Responses')
    plt.legend()
    plt.grid(True)

    # Control effort
    plt.subplot(2, 1, 2)
    plt.plot(t_eval, control_effort, label='Control Effort')
    plt.xlabel('Time (s)')
    plt.ylabel('Control Effort (T_infinity)')
    plt.title('Control Effort by the Heater Blower')
    plt.legend()
    plt.grid(True)

    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    # Run the closed loop simulation with PID
    simulate_closed_loop_with_pid()
