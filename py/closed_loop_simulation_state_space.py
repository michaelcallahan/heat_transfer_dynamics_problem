import numpy as np
import matplotlib.pyplot as plt
from scipy.linalg import solve_continuous_are
import control


class HeatTransferStateSpace:
    def __init__(self, R_s, r_s, t, h_s, h_y, k_s, rho_s, c_s, rho_y, c_y, k_rubber, T_ref):
        self.R_s = R_s
        self.r_s = r_s
        self.t = t
        self.h_s = h_s
        self.h_y = h_y
        self.k_s = k_s
        self.rho_s = rho_s
        self.c_s = c_s
        self.rho_y = rho_y
        self.c_y = c_y
        self.k_rubber = k_rubber
        self.T_ref = T_ref

        self.calculate_parameters()
        self.calculate_gains()

    def calculate_parameters(self):
        self.V_s = (4 / 3) * np.pi * (self.R_s ** 3 - self.r_s ** 3)
        self.A_s = 4 * np.pi * self.R_s ** 2
        self.A_he = np.pi * self.R_s ** 2  # Surface area of heating element (1/4 of the sphere's surface)
        self.A_conv_s = 1 * np.pi * self.R_s ** 2  # Remaining convective surface area (1/4 of the sphere's surface)
        self.A_cond = 2 * np.pi * self.R_s ** 2
        self.V_y = (2 / 3) * np.pi * ((self.R_s + self.t) ** 3 - self.R_s ** 3)
        self.A_conv_y = 2 * np.pi * (self.R_s + self.t) ** 2
        self.t_rubber = 0.005  # rubber lining thickness (m)

    def calculate_gains(self):
        self.A_matrix = np.array([
            [-(self.h_s * self.A_conv_s + self.h_s * self.A_he + self.k_rubber * self.A_cond / self.t_rubber) / (
                        self.rho_s * self.c_s * self.V_s),
             (self.k_rubber * self.A_cond / self.t_rubber) / (self.rho_s * self.c_s * self.V_s)],
            [(self.k_rubber * self.A_cond / self.t_rubber) / (self.rho_y * self.c_y * self.V_y),
             -(self.h_y * self.A_conv_y + self.k_rubber * self.A_cond / self.t_rubber) / (
                         self.rho_y * self.c_y * self.V_y)]
        ])

        self.B_matrix = np.array([
            [self.h_s * self.A_conv_s / (self.rho_s * self.c_s * self.V_s),
             self.h_s * self.A_he / (self.rho_s * self.c_s * self.V_s)],
            [self.h_y * self.A_conv_y / (self.rho_y * self.c_y * self.V_y), 0]
        ])

        self.C_matrix = np.eye(2)
        self.D_matrix = np.zeros((2, 2))

        Q = 50*np.eye(2)
        R = np.diag([5, 1])

        # Solve the CARE
        P = solve_continuous_are(self.A_matrix, self.B_matrix, Q, R)

        # Calculate the state feedback gain K
        self.K = np.linalg.inv(R) @ self.B_matrix.T @ P

        # Calculate the reference tracking gain K_r
        self.K_r = -np.linalg.inv(self.C_matrix @ np.linalg.inv(self.A_matrix - self.B_matrix @ self.K) @ self.B_matrix)

    def state_space_system(self):
        return control.ss(self.A_matrix - self.B_matrix @ self.K, self.B_matrix @ self.K_r, self.C_matrix,
                          self.D_matrix)


def simulate_closed_loop_with_ss():
    # Define Parameters
    R_s = 0.05  # 50mm outer radius
    r_s = 0.0  # Solid steel ball
    t = 0.01  # 10mm radial thickness of yoke
    h_s = 43  # Convective heat transfer coefficient for steel ball (W/m^2K)
    h_y = 43  # Convective heat transfer coefficient for aluminum yoke (W/m^2K)
    k_s = 50  # Thermal conductivity of steel (W/mK)
    rho_s = 7800  # Density of steel (kg/m^3)
    c_s = 500  # Specific heat capacity of steel (J/kgK)
    rho_y = 2700  # Density of aluminum (kg/m^3)
    c_y = 900  # Specific heat capacity of aluminum (J/kgK)
    k_rubber = 0.15  # Thermal conductivity of the rubber silicone lining (W/mK)
    T_ref = 70  # Desired reference temperature
    T_initial = 30  # Initial temperature
    t_end = 2000  # Simulation end time

    # Define State Space System
    state_space_system = HeatTransferStateSpace(R_s,
                                                r_s,
                                                t,
                                                h_s,
                                                h_y,
                                                k_s,
                                                rho_s,
                                                c_s,
                                                rho_y,
                                                c_y,
                                                k_rubber,
                                                T_ref)

    # Create the state space system
    sys_ss = state_space_system.state_space_system()

    # Initial conditions
    X0 = [T_initial, T_initial]

    # Time vector
    t_eval = np.linspace(0, t_end, 1000)

    # Define the reference input
    U = np.ones((len(t_eval), 2)).T * T_ref

    # Simulate the closed loop system
    T, yout, xout = control.forced_response(sys_ss, T=t_eval, U=U, X0=X0, return_x=True)

    # Extract the control efforts
    control_efforts = -state_space_system.K @ xout + state_space_system.K_r @ U
    T_infinity = control_efforts[0, :]
    T_he = control_efforts[1, :]

    print(control_efforts)

    # Plot the results
    plt.figure(figsize=(12, 8))

    # Temperature responses
    plt.subplot(3, 1, 1)
    plt.plot(T, yout[0], label='Steel Ball Temperature (T_s)')
    plt.plot(T, yout[1], label='Aluminum Yoke Temperature (T_y)')
    plt.axhline(T_ref, color='k', linestyle='--', label='Setpoint')
    plt.xlabel('Time (s)')
    plt.ylabel('Temperature (°C)')
    plt.title('Closed-Loop Temperature Responses')
    plt.legend()
    plt.grid(True)

    # Control effort for T_infinity
    plt.subplot(3, 1, 2)
    plt.plot(T, T_infinity, label='Control Effort (T_infinity)')
    plt.xlabel('Time (s)')
    plt.ylabel('Control Effort (°C)')
    plt.title('Control Effort by the Heater Blower (T_infinity)')
    plt.legend()
    plt.grid(True)

    print(T_infinity)

    # Control effort for T_he
    plt.subplot(3, 1, 3)
    plt.plot(T, T_he, label='Control Effort (T_he)')
    plt.xlabel('Time (s)')
    plt.ylabel('Control Effort (°C)')
    plt.title('Control Effort by the Heating Element (T_he)')
    plt.legend()
    plt.grid(True)

    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    # Run Closed Loop Simulation with State Space
    simulate_closed_loop_with_ss()
