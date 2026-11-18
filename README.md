# AMR Factory

Autonomous Mobile Robot simulation inside a realistic cable / wiring factory.

Built with **Gazebo Harmonic** + pure Python navigator (no full ROS required).

## Features

- Autonomous navigation using A* path planning
- LiDAR-based dynamic obstacle avoidance and re-planning
- QR code verification at every workstation
- Multiple factory stations (assembly, cable cutter, crimping, ultrasonic welder, QC, packing, etc.)
- One-click installation and launch on Windows (via WSL)
- Interactive menu or full automatic patrol mode

## Quick Start (Windows)

1. Extract the project
2. Double-click `1_INSTALLER.bat` (first time only)
3. Double-click `2_LANCER_SIMULATEUR.bat`
4. When the factory appears in Gazebo, double-click `3_LANCER_NAVIGATEUR.bat`

You can also run a full patrol with `4_PATROUILLE_TOUS_LES_POSTES.bat`.

## Requirements

- Windows 10/11 with virtualization enabled
- At least 8 GB RAM
- Internet connection for the first installation

## Project Structure

- `navigate.py` — Main navigation logic (A* + LiDAR + QR reading)
- `factory_standalone.sdf` — Gazebo world of the factory
- `install.sh` / `start_sim.sh` / `start_nav.sh` — Linux scripts
- `*.bat` — Windows launchers

## License

This project is open source. Feel free to use, modify and share it.
