# Gazelle_NMPC_controller

## Installation
### Prerequisites
1. MATLAB - R2020a or later
    * Toolbox
      - Control System Toolbox
      - Robotics System Toolbox
2. CasADi
    * casadi-windows-matlabR2016a-v3.5.5 (Checked) 
    * Download the software from the official website and extract it into the same directory as the 'main.m' file.
 
### File tree
* Make sure all the folders and files are organized in the following structure:
    - Gazelle_NMPC_controller
      - qpSWIFT
      - Kinematics
      - casadi-windows-matlabR2016a-v3.5.5
      - PARA.m
      - main.m
      - ...
* If you download the project (not clone), make sure to also download qpSWIFT.

## How to use
To run the program, execute the 'main.m' file.

## User inputs
1. main.m
  * Step information
      * number_of_step: Number of steps
      * step_length: Step stride
      * step_width: Step width
      * step_time: Step period
      * L_or_R: First swing foot: 1: Left foot / -1: Right foot

  * Disturbance information: Apply a disturbance with magnitude [Impact_force_x; Impact_force_y] [N] and duration [Impact_duration] [s] at [Impact_timing] [s] of the [Impact_step_number] [th step].
      * Impact_force_x: X-direction impact force
      * Impact_force_y: Y-direction impact force
      * Impact_duration: Duration of impact
      * Impact_timing: Timing of impact
      * Impact_step_number: Step number when impact occurs
          
  * Flags
      * flag_HORIZON_CHANGED: Set to 1 if the number of MPC horizon is changed
      * flag_VISUALIZATION: Set to 1 enable graphics
      * flag_VISUALIZATION_ROBOT: Set to 1 to show the robot
      * flag_PLOT: Set to 1 to show plots

2. PARA.m
      * H: Number of MPC horizon
      * dt_MPC: Sampling time of MPC     






