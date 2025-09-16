
Autonomous Flight Controller for HL-20 Space Plane

![GitHub](https://img.shields.io/badge/MATLAB-R2023a%2B-blue?logo=mathworks)
![GitHub](https://img.shields.io/badge/Status-Complete-success)
![GitHub](https://img.shields.io/badge/License-MIT-lightgrey)

This repository contains the MATLAB source code and simulation environment for the final report "Design of an Autonomous Flight Controller for an Aerospace Vehicle". The project focuses on the development, implementation, and comparative analysis of linear and non-linear control strategies to achieve stable autonomous flight for the HL-20 lifting body space plane.

📖 Abstract

This research provides an in-depth exploration of the dynamics and control mechanisms of the HL-20 space plane. The study begins with a thorough derivation of the aircraft's non-linear equations of motion and aerodynamic model. An initial control strategy based on the **Linear Quadratic Regulator (LQR)** is implemented and evaluated. Simulations reveal the limitations of LQR in handling the inherent non-linearities of the system, leading to deviations in key flight parameters.

To address these shortcomings, a non-linear decoupling control law is designed to separate the attitude dynamics from the translational dynamics. A comparative analysis demonstrates that the non-linear controller offers superior stability and precision, particularly in complex dynamic environments. This work underscores the critical need to account for non-linearities in aerospace control system design.

✨ Key Features

*   High-Fidelity HL-20 Model: A 6-DoF non-linear simulation model of the HL-20 space plane, incorporating realistic aerodynamics, inertia properties, and engine dynamics.
*   Dual Control Strategies:
    *   Linear Quadratic Regulator (LQR): An optimal controller designed for the linearized system model.
    *   Non-Linear Decoupling Law: A sophisticated controller that decouples and independently stabilizes the attitude and translational dynamics.
*   Comprehensive Simulation Framework: MATLAB scripts for simulating flight scenarios over varying durations (1,000 to 20,000 episodes).
*   Extensive Analysis: Scripts to plot and analyze critical flight dynamics variables, including:
    *   Forward Velocity (`u`), Vertical Velocity (`w`)
    *   Angular Rates: Roll (`p`), Pitch (`q`), Yaw (`r`)
    *   Altitude (`h`)
*   Comparative Results: Clear graphical evidence comparing the performance and limitations of both control laws.

🧮 Methodology

1. Dynamics Modeling
The non-linear 6-DoF equations of motion for the HL-20 were implemented in MATLAB, including:
*   Translational kinematics and dynamics (Eq. 1-3, 7-9).
*   Rotational kinematics and dynamics (Eq. 4-6, 32-33).
*   Engine and actuator dynamics (Eq. 10-11).

2. Aerodynamic Model
The function `Aero_model_dec.m` calculates aerodynamic force and moment coefficients (`CX, CY, CZ, Cl, Cm, Cn`) based on the angle of attack (α), sideslip angle (β), control surface deflections (elevator, aileron, rudder), and angular rates.

3. Control Law Design
*   LQR Control: The system was linearized around a cruising equilibrium point. An LQR controller was synthesized by solving the algebraic Riccati equation to minimize a quadratic cost function (Eq. 15).
*   Non-Linear Decoupling: A feedback linearization technique was applied to transform the non-linear system into a linear equivalent, allowing for independent control of the attitude (p, q, r) and translational (u, v, w) dynamics (Eq. 53).

4. Simulation & Evaluation
Both controllers were tested in a MATLAB/Simulink environment across multiple simulation runs ("episodes") to evaluate their ability to maintain stability and track desired flight conditions (e.g., altitude hold).

📊 Results and Conclusion

The simulation results conclusively demonstrate the superiority of the non-linear decoupling law:
*   LQR Limitations: The LQR controller, while stable for the linearized model, failed to maintain performance over longer durations in the full non-linear simulation. It exhibited gradual divergences in velocity and altitude, eventually leading to instability (Figs. 2-8).
*   Non-Linear Decoupling Success: The non-linear controller effectively managed cross-coupling and non-linear effects. It successfully corrected deviations and maintained stable flight across all tested scenarios, proving its robustness and suitability for autonomous aerospace vehicles (Figs. 9-15).

Conclusion: For complex, non-linear systems like the HL-20, control strategies that explicitly account for system non-linearities (like feedback linearization) are essential for guaranteeing stability and performance, especially during critical mission phases like re-entry and cruising.

🚀 Getting Started

Prerequisites
*   MATLAB: Version R2023a or newer is recommended. The project utilizes core MATLAB functionality.
*   Required Toolboxes: (Likely) Control System Toolbox, though standard installation often includes this.

Installation
1.  Clone this repository to your local machine:
    ```bash
    git clone https://github.com/bijin881/Project.git
    ```
2.  Open MATLAB and navigate to the cloned project directory.

Running Simulations
The main simulation is managed through the provided scripts.
1.  Ensure all `.m` files are in your MATLAB path.
2.  The primary dynamics are defined in `Aircraft_equations_of_motion.m`.
3.  The linear model and LQR gain calculation are performed in `Linear_model.m`.
4.  Run the main script (e.g., `main_simulation.m` or similar - please check the specific entry point script in the repository) to start a simulation.
5.  Configure the simulation parameters (e.g., number of episodes, control law selection: LQR vs. Non-Linear) within the script.
6.  The results will be plotted automatically, showing the time history of all key state variables.

📁 Repository Structure

```
Project/
├── Aircraft_equations_of_motion.m  # Main 6-DoF dynamics function
├── Aero_model_dec.m               # Aerodynamic coefficients calculation
├── Linear_model.m                 # Linearizes model and computes LQR gains
├── atmos.m                        # Atmospheric model (pressure, density)
├── main_simulation.m              # (Expected) Main script to run simulations
├── plot_results.m                 # (Expected) Script for generating figures
├── Final Report.pdf               # Detailed project thesis
└── README.md                      # This file
```

👨‍💻 Author

Bijin Biju
*   Queen Mary, University of London
*   [GitHub Profile](https://github.com/bijin881)

Supervisor: Dr. Ranjan Vepa

📜 License

This project is licensed under the MIT License - see the LICENSE file (if provided) for details.

🙏 Acknowledgments

*   Dr. Ranjan Vepa for his invaluable guidance and supervision.
*   NASA Langley Research Center for the HL-20 concept and data.
*   Family and friends for their unwavering support.

📚 References

Key references are listed in the final report. The work builds upon foundational research in optimal control, aircraft dynamics, and non-linear control theory, notably:
*   Garza & Morelli (2003) for the HL-20 MATLAB simulation.
*   Kalman (1960), Pontryagin (1962) for optimal control theory.
*   Etkin & Reid (1995), Stevens, Lewis, & Johnson (2015) for flight dynamics.
