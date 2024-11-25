# Simulation Instructions

This repository contains all the files necessary to run a state estimation simulation. The current implementation uses an Extended Kalman Filter (EKF) in the `estimator_with_gps_smoothing` file. The task is to replace the EKF with a Particle Filter while keeping all other files unchanged.

## Prerequisites
- **MATLAB** (Recommended version: R2021a or later)
- Required MATLAB toolboxes for numerical computation and plotting.

## Instructions to Run the Simulation

1. **Download All Files**:
   - Download all the files from this repository.
   - Place all files in the **same folder**. This is necessary because the simulation relies on multiple files that need to access each other.

2. **Setup**:
   - Open MATLAB.
   - Navigate to the folder containing the downloaded files using MATLAB’s **Current Folder** browser or the `cd` command in the command window.

3. **Run the Simulation**:
   - Open the main simulation file `runhw9.m` in MATLAB.
   - Execute the script by clicking the **Run** button or typing `run('runhw9.m')` in the command window.

4. **Modify the Estimator**:
   - Locate the file `estimator_with_gps_smoothing.m`.
   - Replace the existing Extended Kalman Filter (EKF) implementation with a Particle Filter.
   - Save the file after modifying the code.
   - Re-run the simulation by executing `runhw9.m` to test the Particle Filter implementation.

5. **Output**:
   - The simulation will generate:
     - Plots comparing estimated states and ground truth.


## Notes
- Do not modify any files other than `estimator_with_gps_smoothing.m`. The other files are essential for the simulation and are pre-configured to work with both EKF and Particle Filter estimators.
- Ensure all downloaded files remain in the same folder throughout the process.
