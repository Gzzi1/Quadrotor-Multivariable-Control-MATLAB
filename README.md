# Robust Multivariable Control of the ANT-X Quadrotor Dynamics

This repository contains the MATLAB implementation for the synthesis and validation of a robust attitude controller for the ANT-X research quadrotor. The project was developed as part of the Aerospace Control Systems course at Politecnico di Milano (A.Y. 2025/2026).

The main objective is to design a reliable attitude control system operating on grey-box models linearized around the hovering condition, ensuring stability and performance despite parametric uncertainties and cross-axis coupling.

## Project Overview & Architecture

The control architecture features a cascade topology for both lateral and longitudinal axes:
* **Inner Loop (Rate):** Filtered PID controller (Rp and Rq) with a fixed derivative filter time constant of 0.02 s.
* **Outer Loop (Angle):** Pure proportional controller (R_phi and R_theta).

### Performance Requirements
* Minimum natural frequency (omega_n) >= 10 rad/s.
* Minimum damping ratio (xi) >= 0.8 (Relaxed from 0.9 to accommodate physical actuator constraints).
* Maximum command deflection |delta| <= 0.05 for a 10 deg doublet maneuver.

---

## Task Breakdown & Methodology

### Task 1: Nominal Modelling and H-infinity Design
* Analyzed the linearized lateral and longitudinal dynamics.
* The lateral plant is inherently unstable (roll pole at Re = +2.6478), while the longitudinal plant is stable. 
* Synthesized the controllers using mixed-sensitivity H-infinity optimization (`hinfstruct`) to satisfy both performance (Wp) and control effort (Wq) bounds.

### Task 2: Uncertain Modelling
* Modeled parameter uncertainties with a +/- 3 sigma bound (covering 99.7% of a Gaussian distribution) on 5 aerodynamic derivatives per axis.
* Built the uncertain linear families using the MATLAB Robust Control Toolbox (`ureal`, `uss`).

### Task 3: Robustness Analysis (mu-Analysis)
* Expressed the uncertain models in M-Delta form.
* Evaluated Robust Stability (RS) using the Structured Singular Value (mu) and compared it against the conservative Small Gain theorem. 
* **Result:** The mu upper bound remains strictly below 0 dB, guaranteeing robust stability for the given uncertainty family.

### Task 4: Robustness to Cross-Axis Coupling
* Introduced an uncertain kinematic coupling matrix R(psi) with a misalignment angle psi in [-15 deg, +15 deg].
* Conducted a full MIMO mu-analysis to verify stability.
* **Result:** The system remains robustly stable even with lateral-longitudinal coupling, despite the Small Gain theorem falsely predicting instability due to overestimation.

### Task 5: Monte Carlo Validation
* Performed a probabilistic validation drawing from 11 distinct sources of uncertainty (Gaussian for parameters, Uniform for coupling psi).
* Configured N = 4812 simulations based on the Chernoff bound to guarantee a +/- 2% accuracy with 95% confidence.
* **Result:** ~97.6% of the simulated quadrotor population met all aggressive performance and effort requirements.

---

## How to Run

1. Clone the repository.
2. Ensure you have MATLAB installed along with the **Robust Control Toolbox**.
3. Run the main script `ACS_Project.m`.
4. The script will sequentially compute nominal designs, uncertain models, robust stability margins, and finally execute the Monte Carlo simulation. 

## Project Documentation
* [Project Assignment and Specifications (PDF)](./docs/ACS_project_2526.pdf)
* [Final Presentation and Results (PDF)](./docs/ANT-X_ACS_Project_Presentation.pdf)

## Contributors
* Federico Gozzi
* Simone Leandri
* Matteo Graziani


*Politecnico di Milano - Department of Aerospace Science and Technology (DAER)*
