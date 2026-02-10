# PAC-MAN: Robotics and Control 2 Project

This repository contains the project implementation for the **Robotics & Control 2 (R&C2)** course at the **University of Padova**, Department of Information Engineering. 

The project was developed for the Master's Degree in Control Systems Engineering (A.Y. 2025-26) under the guidance of Prof. **Angelo Cenedese**.

## 🕹️ Project Overview: The "PAC-MAN Challenge"

The 2025 challenge involves controlling a unicycle mobile robot—modeled after the classic arcade character PAC-MAN—to navigate a maze environment. The robot must track a specific path and successfully perform a parking maneuver into a central docking box.

### Project Phases
The implementation is divided into four distinct phases:
* **Phase 1**: Definition of the scenario, path characterization (trajectory to be tracked), and positioning of the docking box.
* **Phase 2**: Definition of simplified scenarios (circular and square paths) to perform controller tuning.
* **Phase 3**: Execution of the tracking task along a closed trajectory within the maze.
* **Phase 4**: Regulation task to transition from the end of the trajectory into the final parking box.

## 🛠️ Control Architecture

The project implements advanced control strategies for non-holonomic unicycle robots:

### Path Tracking (Phase 3)

The tracking system utilizes **Differential Flatness** to follow the desired trajectory $q_d$. A PD feedback loop is integrated to minimize tracking errors.

### Regulation and Parking (Phase 4)

For the parking maneuver, a regulation task is performed to bring the robot from the trajectory endpoint to the docking station ($q_d = 0$ in the local frame). This involves state transformation (TF) and specific gain tuning for stability.

## 💻 Implementation Notes

The following MATLAB function demonstrates the core logic used to calculate the control inputs based on the desired velocities ($v_d, w_d$) and the damping parameters:


https://github.com/user-attachments/assets/3db8e75e-e49a-477e-84ed-f3a04f916301

