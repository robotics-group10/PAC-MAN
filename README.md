# PAC-MAN: Robotics and Control 2 Project

<p align="center">
  <img width="300" height="300" alt="logo_pacman" src="https://github.com/user-attachments/assets/9e5a2b53-99ee-4cc5-824d-422de4e2803c" />
</p>

This repository contains the project implementation for the **Robotics & Control 2 (R&C2)** course at the **University of Padova**, Department of Information Engineering. 
For more information you can read our presentation `./presentation.pdf`.

## Output Example

https://github.com/user-attachments/assets/3db8e75e-e49a-477e-84ed-f3a04f916301

## Project Overview: The "PAC-MAN Challenge"

The challenge involves controlling a unicycle mobile robot—modeled after the classic arcade character PAC-MAN—to navigate a maze environment. 
The robot must track a specific path and successfully perform a parking maneuver into a central docking box.

### Project Phases
The implementation is divided into four distinct phases:
* **Phase 1**: Definition of the scenario, path characterization (trajectory to be tracked), and positioning of the docking box.
* **Phase 2**: Definition of simplified scenarios to perform controller tuning.
* **Phase 3**: Execution of the tracking task along a closed trajectory within the maze.
* **Phase 4**: Regulation task to transition from the end of the trajectory into the final parking box.

## Control Architecture

In this project we've tested 3 different controllers for trajectory tracking and
2 different controllers for regulation.

<div align="center">

| Control Scheme | Architecture Diagram |
| :--- | :--- |
| **State-error linearization control scheme** | <img src="https://github.com/user-attachments/assets/a1efb702-d883-41a0-8b9a-b96f0ccc6606" width="600" /> |
| **State-error nonlinear control scheme** | <img src="https://github.com/user-attachments/assets/936abfe0-83e1-49c7-bc56-f9cffcc4ec5e" width="600" /> |
| **Output-error feedback control scheme** | <img src="https://github.com/user-attachments/assets/ab9a7bec-61e1-4cfa-a8cb-c0c568a95b02" width="600" /> |

| Cartesian Control | Posture Control |
|-----------------------|---------------------|
| <img src="https://github.com/user-attachments/assets/7ca03bd4-148b-433d-a672-12e9c9be3e34" width="100%"> | <img src="https://github.com/user-attachments/assets/6232cd77-529d-47b7-b540-9ac6ae733a16" width="100%"> |


</div>

## Run the code

Tune a controller:

```matlab
run Scripts/tuning.m
```

Plots are stored in `./Figures/` folder.

Simulate inside PAC-MAN maze:

```matlab
run Scripts/maze_sim.m
```

Plots are stored in `./Scripts/Figures/` folder, the video is stored in `./Scripts/Video/`.
