# Human-Robot Cooperative Heavy Payload Manipulation


![Build Status](https://img.shields.io/badge/build-passing-brightgreen)
![ROS Version](https://img.shields.io/badge/ROS-humble-blue)
![Simulation](https://img.shields.io/badge/MuJoCo-Physics-orange)

## 📖 Introduction

The project validates the HRC Transportation simulation (MuJoCo physics simulation validation) across three different Mobile Robot platforms:
1.  **Moying Mobile Manipulator** 
2.  **PR2 Robot** 
3.  **Moying Dual-Arm Robot**

## 📂 Repository Structure

This repository is organized into three distinct workspaces:

```text
iros/
├── moying_mcr_ws/      # Workspace for Moying mobile manipulator
│   ├── src/            # base control packages
│   └── ...
│   └── README.md
├── pr2_ws/             # Workspace for PR2 arm manipulation
│   ├── src/            # base control packages
│   └── ...
│   └── README.md
├── unitree_mujoco/     # Workspace for  simulation
│   ├── model/          # XML models for MuJoCo physics engine
│   └── src/            # Simulation control interfaces (C++/Python)
│   └── ...
│   └── README.md
└── README.md
>>>>>>> e111063cd64b345cc9c6b47b36807dede1efcd9e
