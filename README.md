# Rendezvous-Proximity-Operation-GNC-Project

A MATLAB/Simulink project focused on developing a 6-DoF Guidance, Navigation, and Control (GNC) simulation for spacecraft rendezvous and proximity operations (RPO) in Low Earth Orbit.

## Project Overview

The simulation models an active chaser spacecraft approaching a passive target spacecraft, progressing from far-range rendezvous to close-proximity operations and docking.

The project covers orbital dynamics, trajectory guidance, state estimation, translational control, attitude determination and control, and actuator modelling.

## Current Implementation

- Nonlinear relative orbital dynamics in the LVLH frame
- Clohessy-Wiltshire (CW) linear relative motion model
- Validation of nonlinear dynamics against CW predictions
- Linear Quadratic Regulator (LQR) for translational control
- Discrete Kalman Filter for relative state estimation
- Preliminary thruster acceleration limits

## Planned Development

- Stateflow-based rendezvous guidance and mission sequencing
- Far-range CW targeting and manoeuvre planning
- Close-range approach and docking guidance
- Extended Kalman Filter (EKF) with range and bearing measurements
- Thruster allocation and actuator dynamics
- Integration with reaction-wheel-based Attitude Determination and Control (ADCS)
- Coupled 6-DoF simulation
- Robustness analysis, Monte Carlo testing, and trajectory visualization

## Tools

- MATLAB
- Simulink
- Stateflow
- Aerospace Toolbox

## Status

**Work in progress.** Nonlinear relative dynamics, linear model validation, LQR control, and initial Kalman filtering have been implemented. Far-range guidance development is underway.
