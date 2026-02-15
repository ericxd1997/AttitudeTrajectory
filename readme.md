# Attitude Trajectory Planning using ACADOS

This repository contains MATLAB simulations for attitude trajectory planning of a rigid body. The main script `closed_loop_LR.m` uses the **ACADOS** toolkit to perform nonlinear system simulation with embedded optimization.

## Overview

The program implements attitude trajectory planning for orientation representation using three methods:
- Euler angle method
- Quaternion method
- Quaternion decomposition method

It incorporates angle and angular velocity constraints, and simulates the nonlinear closed‑loop system via ACADOS, a software package for fast embedded optimization (particularly suited for model predictive control and nonlinear dynamic optimization).

## Dependencies

- **MATLAB** (tested with R2020b or later)
- **ACADOS** (C‑based toolkit with MATLAB interface)

### Installing ACADOS

Follow the official ACADOS installation guide: [https://github.com/acados/acados](https://github.com/acados/acados)

After installing ACADOS, you need to set up the MATLAB interface. This repository expects that you have compiled ACADOS and that the MATLAB interface files are available.

## Usage

1. **Add ACADOS to MATLAB path**  
   Before running the main simulation, you must run the ACADOS setup script to add the necessary folders to your MATLAB path.  
   From the MATLAB command window, navigate to your ACADOS installation and execute:
   ```matlab
   run('./acados/examples/acados_matlab_octave.m')
   ```
   (Adjust the path if your ACADOS installation is located elsewhere.)

2. **Run the main simulation**  
   Once the ACADOS path is set, simply run the main script:
   ```matlab
   closed_loop_LR
   ```
   The script will perform the attitude trajectory planning and simulation, and display / save the results.

## Files

- `closed_loop_LR.m` – Main simulation script.
- (Other supporting files may be present; refer to the script for details.)

## Program Description

The simulation performs attitude trajectory planning for orientation using three different parameterizations:
- Euler angles (with appropriate handling of singularities)
- Quaternions (unit quaternions for global, singularity‑free representation)
- Quaternion decomposition (a method that splits the quaternion into components for constraint handling)

Angle and angular velocity constraints are enforced during the trajectory generation, ensuring the planned motion respects physical limits. The nonlinear dynamics are simulated using the ACADOS toolkit, which efficiently solves the underlying optimal control problem in a closed‑loop manner.

## Notes

- Make sure that ACADOS is correctly installed and that the MATLAB interface is compiled before running the script.
- The script may generate plots or output data; check the console for progress messages.


## Contact

SENSORS:Attitude Controller for UAV First-Order Acceleration Coefficients Tracking
