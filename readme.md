# Attitude Trajectory Planning with ACADOS

This repository contains MATLAB code for attitude trajectory planning of a rigid body under angular motion constraints. The main simulation file `closed_loop_LR.m` implements a nonlinear closed‑loop controller using the **ACADOS** toolkit. Three attitude representations are considered: Euler angles, quaternions, and a quaternion decomposition method. Both angle and angular velocity constraints are incorporated into the planning.

## Dependencies

- **MATLAB** (R2020b or later recommended)
- **ACADOS** toolkit – a software package for fast optimization of optimal control problems.  
  A pre‑compiled version for macOS is provided in `acados.zip`.  
  For other operating systems, please compile ACADOS from source following the [official instructions](https://github.com/acados/acados).

## Installation

1. Clone or download this repository.
2. Unzip the provided ACADOS package:
   ```bash
   unzip acados.zip -d /path/to/your/workspace
   ```
   This will create a folder named `acados` containing the compiled binaries and MATLAB interface.
3. Set the environment variable `ACADOS_SOURCE_DIR` to the path of the extracted `acados` folder.  
   For a temporary session in MATLAB you can run:
   ```matlab
   setenv('ACADOS_SOURCE_DIR', '/path/to/your/workspace/acados')
   ```
   To make it permanent, add the following line to your shell startup file (e.g., `~/.zshrc` or `~/.bash_profile`):
   ```bash
   export ACADOS_SOURCE_DIR="/path/to/your/workspace/acados"
   ```
4. **Important**: Before running any simulation, execute the MATLAB script that adds ACADOS to the MATLAB path:
   ```matlab
   run('/path/to/your/workspace/acados/examples/acados_matlab_octave.m')
   ```
   This script configures the necessary paths and environment variables for the ACADOS MATLAB interface.

## Usage

1. Make sure ACADOS is properly set up (see Installation).
2. Open MATLAB and navigate to the root folder of this project.
3. Run the main simulation:
   ```matlab
   closed_loop_LR
   ```
   The script will perform attitude trajectory planning with angle and angular velocity constraints using the three representation methods and display the results.

## File Structure

- `closed_loop_LR.m` – Main simulation script.
- `acados.zip` – Pre‑compiled ACADOS package for macOS (includes MATLAB interface).
- Other supporting files (e.g., dynamics models, constraint functions) may be present in the repository.

## Notes

- The ACADOS package provided in `acados.zip` was built on macOS. If you are using Linux or Windows, you will need to compile ACADOS yourself and update the paths accordingly.
- The script `acados/examples/acados_matlab_octave.m` is part of the ACADOS distribution and must be run **every time** you start MATLAB, or you can add its contents to your `startup.m` file.
- For questions about ACADOS, refer to the [ACADOS documentation](https://docs.acados.org/).

## License

This project is for academic use only. Please cite the original authors if you use this code in your research.
