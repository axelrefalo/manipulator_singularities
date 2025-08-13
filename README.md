# Robotic Kinematic Singularities – ABB GoFa CRB15000

This repository provides a symbolic and numerical framework for analyzing kinematic singularities of open-chain manipulators. The ABB GoFa CRB15000 collaborative robot is used as an example, but the same methodology can be applied to other robots. The approach combines screw theory and exterior algebra (via wedge products) to identify singular configurations.

## Overview

Given a DH table and a robot definition (URDF), the script uses MATLAB's Robotics System Toolbox for visualization and computes all possible linear dependencies between screw axes to detect singularities.

## File Structure

├── main.m # Launches singularity search
├── analysis.mlx # Analysis of found singularities
├── function/ # Project functions
│ ├── checkSelfCollision.m
│ ├── displayRobot.m # Robot visualization
│ ├── loadModel.m
│ ├── optimize.m # Wedge product optimization
│ ├── searchSingularities.m # Core search engine
│ ├── wedgeProduct.m
├── robot/ # URDF files and meshes

## Requirements

- MATLAB R2021a or later
- Symbolic Math Toolbox
- Robotics System Toolbox (for URDF parsing and visualization)

## Usage

1. Edit the `main.m` file to match your local directory path.
2. Run:
   ```matlab
   >> main




