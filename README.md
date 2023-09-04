# ROB498 EngSci Capstone Project

## Contributors
- Savanna Blade
- Kohava Mendelson
- Nicole Amenta

## Project Description

Welcome to the ROB498 EngSci Capstone Project! In this project, our team was tasked with constructing a drone and endowing it with a suite of intelligent capabilities, including stationkeeping and waypoint navigation. This README file provides an overview of the project and its components.

## Project Files

This project includes two main Python files:

1. `challenge_2.py`: This script accomplishes the following task:
   
   For this task, teams must provide their drone with the ability to perform autonomous stationkeeping. In the context of inspection drones, stationkeeping is a task that requires the drone to maintain a stable hover at a fixed altitude. This challenge goes beyond ordinary stationkeeping and demands that the drone also maintain its pose (position and orientation) relative to a fixed external frame (e.g., MY580). The pose that the drone should maintain is determined by its starting position and a prescribed altitude of 150 cm. Since stationkeeping is often required for prolonged observation, the drone must demonstrate a stable hover at this pose for a test duration of 30 seconds.

2. `challenge_3.py`: This script accomplishes the following task:
   
   For this task, teams must enable their drones to navigate through several waypoints. Waypoint navigation allows the drone to visit several variable or pre-programmed sites accurately and quickly. As in Challenge #2, the task begins with a 150 cm hover; upon reaching the hover, a set of waypoint coordinates are sent to the drone (represented in the global Vicon frame). The waypoints are defined as spherical volumes around the given coordinates with a radius of 35 cm. There is no constraint regarding how long the drone must remain in the waypoint; it is sufficient to simply enter the bounding area. Additionally, this is a timed challenge, with a limit of 60 seconds for a full traversal of all given waypoints.

## Usage

Each script (`challenge_2.py` and `challenge_3.py`) can be executed independently to demonstrate the respective capabilities of the drone (stationkeeping and waypoint navigation). Make sure to review the specific instructions within each script's comments or documentation for proper usage and configuration.

## Contributions

We welcome contributions to this project! If you'd like to contribute, please follow these guidelines:

1. Fork the repository on GitHub.
2. Clone your forked repository to your local machine.
3. Create a new branch for your feature or bug fix.
4. Make your changes and test thoroughly.
5. Commit your changes and push them to your GitHub repository.
6. Create a pull request against the main branch of the original repository.

## License

This project is licensed under the MIT License. You are free to use, modify, and distribute this software as long as you adhere to the terms of the license.

---

Thank you for your interest in the ROB498 EngSci Capstone Project! If you have any questions or need further assistance, please feel free to contact us. We look forward to your contributions and hope this project will be a valuable learning experience for all involved.

