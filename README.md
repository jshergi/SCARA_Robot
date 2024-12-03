#SCARA Robot Control Software (ROBSIM)

##Overview

ROBSIM is a comprehensive software package designed for the simulation and control of a SCARA robot. 
It includes essential components for matrix computations, kinematics, and trajectory planning, enabling efficient and accurate robot motion. 
The project adheres to modular development principles, ensuring flexibility and ease of debugging.

##Features

    Matrix Computation
        Implements basic matrix routines with 3-tuple position vectors (x, y, z) and 3x3 rotation matrices (restricted to Z-axis rotations).
        Avoids external libraries to ensure compatibility with the robot lab's development environment.

    Forward and Inverse Kinematics
        Derives symbolic forward and inverse kinematics using Denavit-Hartenberg (D-H) parameters, substituting numerical values where appropriate.
        Implements robust kinematics procedures: KIN(), WHERE(), INVKIN(), and SOLVE().
        Considers joint limits and selects the most appropriate solution in cases of multiple solutions.
        Includes a graphic display (Display_Configuration()) for debugging purposes.

    Trajectory Planning
        Implements a joint-space trajectory planner for collision-free motion.
        Supports cubic splines for trajectory generation, ensuring velocity continuity.
        Debugging aid: Plots x-y components of the trajectory on a 2D plane for visualization.

##Debugging and Testing

    Matrix Computations: Test basic matrix operations independently.
    Kinematics: Validate kinematic routines against known configurations and joint limits.
    Trajectory Planning: Visualize planned paths on the x-y plane for better understanding.

