import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt


def heat_transfer_simulation(R_s, r_s, t, h_s=43, h_y=43, k_s=50, rho_s=7800, c_s=500, rho_y=2700, c_y=900,
                             k_rubber=0.15, T_infinity=100, T_initial=30, t_end=3000, t_rubber=0.005):
    """
    Perform an open-loop simulation of heat transfer between a steel ball and an aluminum yoke with a rubber silicone lining and a step input.

    Parameters:
    R_s (float): Outer radius of the steel ball (in meters)
    r_s (float): Inner radius of the steel ball (in meters)
    t (float): Radial thickness of the aluminum yoke (in meters)
    h_s (float): Convective heat transfer coefficient for the steel ball (W/m^2K, default: 43)
    h_y (float): Convective heat transfer coefficient for the aluminum yoke (W/m^2K, default: 43)
    k_s (float): Thermal conductivity of steel (W/mK)
    rho_s (float): Density of steel (kg/m^3)
    c_s (float): Specific heat capacity of steel (J/kgK)
    rho_y (float): Density of aluminum (kg/m^3)
    c_y (float): Specific heat capacity of aluminum (J/kgK)
    k_rubber (float): Thermal conductivity of the rubber silicone lining (W/mK, default: 0.15)
    T_infinity (float): Temperature of the surrounding air heated by the convective heater (default: 100°C)
    T_initial (float): Initial temperature of both the steel ball and aluminum yoke (default: 30°C)
    t_end (float): Simulation end time (in seconds, default: 1000 seconds)

    Returns:
    None
    """

    # Volume and surface area calculations for the steel ball
    V_s = (4 / 3) * np.pi * (R_s ** 3 - r_s ** 3)
    A_conv_s = 4 * np.pi * R_s ** 2
    A_cond = 2 * np.pi * R_s ** 2

    # Volume and surface area calculations for the aluminum yoke (bowl)
    V_y = (2 / 3) * np.pi * ((R_s + t) ** 3 - R_s ** 3)

    A_conv_y = 2 * np.pi * (R_s + t) ** 2

    # Differential equations
    def model(t, T):
        T_s, T_y = T

        dT_s_dt = (h_s * A_conv_s * (T_infinity - T_s) - k_rubber * A_cond * (T_s - T_y) / t_rubber) / (
                    rho_s * c_s * V_s)
        dT_y_dt = (k_rubber * A_cond * (T_s - T_y) / t_rubber + h_y * A_conv_y * (T_infinity - T_y)) / (
                    rho_y * c_y * V_y)

        return [dT_s_dt, dT_y_dt]

    # Initial conditions
    T_initial = [T_initial, T_initial]

    # Time vector
    t_eval = np.linspace(0, t_end, 1000)

    # Solve the differential equations
    solution = solve_ivp(model, [0, t_end], T_initial, t_eval=t_eval, method='RK45')

    # Plot the results
    plt.figure(figsize=(10, 6))
    plt.plot(solution.t, solution.y[0], label='Steel Ball Temperature (T_s)')
    plt.plot(solution.t, solution.y[1], label='Aluminum Yoke Temperature (T_y)')
    plt.xlabel('Time (s)')
    plt.ylabel('Temperature (°C)')
    plt.title('Temperature Response with Step Input of Heater Blower at 100°C')
    plt.legend()
    plt.grid(True)
    plt.show()

    return solution


# Example usage:
heat_transfer_simulation(
    R_s=0.05,  # 50mm outer radius
    r_s=0.0,  # Solid steel ball
    t=0.01,  # 10mm radial thickness of yoke
)


def simulate_extreme_cases():
    # Define extreme geometrical conditions
    geometries = [
        {'R_s': 0.025, 't': 0.005},  # Minimum outer radius and minimum thickness
        {'R_s': 0.1, 't': 0.025},  # Maximum outer radius and maximum thickness
    ]

    # Simulate responses for extreme cases
    responses = []
    for geom in geometries:
        R_s = geom['R_s']
        t = geom['t']
        solution = heat_transfer_simulation(
            R_s=R_s,
            r_s=0.0,  # Solid steel ball
            t=t
        )
        responses.append(solution)

    # Extract data for plotting
    t_eval = responses[0].t
    T_s_min = responses[0].y[0]
    T_y_min = responses[0].y[1]
    T_s_max = responses[1].y[0]
    T_y_max = responses[1].y[1]

    # Plot the results
    plt.figure(figsize=(10, 6))

    # Steel Ball Temperature
    plt.fill_between(t_eval, T_s_min, T_s_max, color='skyblue', alpha=0.5, label='Steel Ball Temperature Range (T_s)')
    plt.plot(t_eval, T_s_min, 'b--', label='T_s (Min Geometry)')
    plt.plot(t_eval, T_s_max, 'b-', label='T_s (Max Geometry)')

    # Aluminum Yoke Temperature
    plt.fill_between(t_eval, T_y_min, T_y_max, color='lightcoral', alpha=0.5,
                     label='Aluminum Yoke Temperature Range (T_y)')
    plt.plot(t_eval, T_y_min, 'r--', label='T_y (Min Geometry)')
    plt.plot(t_eval, T_y_max, 'r-', label='T_y (Max Geometry)')

    plt.xlabel('Time (s)')
    plt.ylabel('Temperature (°C)')
    plt.title('Temperature Responses for Extreme Geometrical Conditions')
    plt.legend()
    plt.grid(True)
    plt.show()

if __name__ == '__main__':
    # Run the simulation for extreme cases
    simulate_extreme_cases()

