# Gazelle_NMPC_controller

## Installation
### Prerequisites
1. MATLAB - R2020a or later
    * Toolbox
      - Control System Toolbox
      - Robotics System Toolbox
2. CasADi
    * casadi-windows-matlabR2016a-v3.5.5 (Checked) 
    * Please download the software from the official website and extract it into the same directory as the 'main.m' file
 
### File tree
* Make sure all these folders and files are organized in the same structure. 
* Your file tree should look like this:
    - Gazelle_NMPC_controller
      - qpSWIFT
      - Kinematics
      - casadi-windows-matlabR2016a-v3.5.5
      - PARA.m
      - main.m
      - ...

## How to use
Run 'main.m' file

## User inputs
1. main.m
  * Step information
      * number_of_step: Number of steps
      * step_length: Step stride
      * step_width: Step width
      * step_time: Step period
      * L_or_R: First swing foot: 1: Left foot / -1: Right foot

  * Disturbance information: Disturbance with [Impact_force_x; Impact_force_y] [N] magnitude and [Impact_duration] [s] duration is applied at [Impact_timing] [s] of [Impact_step_number] [th step]. 
      * Impact_force_x: x-dir impact force
      * Impact_force_y: y-dir impact force
      * Impact_duration: Impact duration
      * Impact_timing: Timing of impact
      * Impact_step_number
          
  * Flags
      * flag_HORIZON_CHANGED: Set to 1 if the number of MPC horizon is changed
      * flag_VISUALIZATION: Set to 1 for graphic ON
      * flag_VISUALIZATION_ROBOT: Set to 1 to show robot
      * flag_PLOT: Set to 1 to show plots

2. PARA.m
      * H: Number of MPC horizon
      * dt_MPC: sampling time of MPC     






