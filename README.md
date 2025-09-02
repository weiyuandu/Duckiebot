# Autonomous Driving System

---
## About
This project focuses on developing a self-driving vehicle system that combines lane-keeping, and road environment detection to mimic core autonomous driving behaviors.
An opencv-based lane detection module keeps the vehicle on course using PID control.
Road sign recognition adds to the autonomous capabilities of the vehicle, allowing it
to respond appropriately to road signage. The perception modules feed into a decision layer that keeps track of various states of the vehicle to plan safe and efficient
trajectories. Finally, a control layer translates high-level decisions into low-level motor commands for steering and speed. The autonomous driving system is written in
python, and tested in the gym-duckietown simulator, ensuring maximum similarities
with a real Duckietown environment. This project provides hands-on experience with
computer vision and cyber-physical systems, building a foundation for more advanced
autonomous vehicle research.

## Setup For Duckietown GYM
One-time setup:
Windows OS should install WSL, install docker inside WSL   
Run：
```bash
docker pull duckietown/gym-duckietown
```
Create 
```bash
C:\duckiesim on Windows host
```
Run these comands:  
```bash
docker run -ti --net=host --ipc=host -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v /mnt/c/duckiesim:/gym_duckietown--env="QT_X11_NO_MITSHM=1" --name duckie duckietown/gym-duckietown bash  
apt update && apt install -y fontconfig libglib2.0-0  
pip install stable_baselines3
```
If you want to exit the container, run:
```bash
exit
```

If you want to save changes, run: 
```bash
docker commit duckie updated_duckietown_image
```
You can start the simulator with：
```bash
docker start -ai duckie
```

Test your setup with python `manual_control.py`

## Setup for Duckiebot-ros
If you want to run the code from the `duckiebot-ros` module, you will need:  
- A properly installed and calibrated Duckiebot **DB21J/M**  
- A local environment with **Duckietown Shell** configured  

For detailed instructions, please refer to the [official Duckietown documentation](https://docs.duckietown.com/daffy/opmanual-duckiebot/intro.html).  

---
If the above conditions are met, you can proceed with the following steps:

1. Clone this repository:
```bash
git clone "git@github.com:weiyuandu/Duckiebot.git"
```
2.	Navigate into the duckiebot-ros folder:
```bash
  cd duckiebot-ros
```
3. Build the Docker image using Duckietown Shell:
```bash
dts devel build -f
```
4.Run the desired node on your Duckiebot:
```bash
dts devel run -R <duckiebot_name> -L <node_name>
```
Replace `<duckiebot_name>` with the name of your Duckiebot and `<node_name>` with the name of the node you want to run.

5.For more details on dts devel run usage, you can check the help manual by running:
```bahs
dts devel run --help
```

## Project Structure
```bash

Duckiebot/
├── README.md                          
├── duckie-setup.md                    # Hardware setup guide for gym
│
├── autonomous_driving/                # Simulation & algorithms
│   ├── main.py                      
│   ├── perception.py                 # Lane detection & computer vision
│   ├── controller.py                 # PID control logic
│   └── utils.py                      # Helper functions
│
├── duckiebot-ros/                     
│   ├── Dockerfile                    
│   ├── launchers/                     # Startup scripts
│   └── packages/my_package/src/       
│       ├── camera_reader_node.py      # Vision & LED control
│       ├── odometry_*.py              # Position estimation
│       ├── square_controller_*.py     # Movement controllers
│       └── wheel_*.py                 # Motor control
│
├── false light detection/             # Traffic light detection
├── Reinforce learning/                # ML approaches
└── assets/                          
```
## Overview of the functionalities of different modules

### autonomous_driving/main.py
Primary Function: Main entry point and orchestration layer

### autonomous_driving/perception.py
Primary Function: Computer vision and environmental sensing

### autonomous_driving/controller.py
Primary Function: Decision making and control logic

### autonomous_driving/utils.py
Primary Function: Utility functions and helper operations  

### autonomous_driving/pid_control_standalone.py
Primary function: An alternate approach using full PID-based control and turn-awareness modifications.

## Project Materials

Presentation link:    https://github.com/weiyuandu/Duckiebot/blob/main/Presentation.pdf

Demo link:    https://github.com/weiyuandu/Duckiebot/blob/main/Project%20Video%20Demo.mp4

Report link:  https://github.com/weiyuandu/Duckiebot/blob/main/CPS_Group_V_Report.pdf


