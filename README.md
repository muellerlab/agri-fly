# Flight Simulator in Agricultural Environment
![agri_fly_pic](https://user-images.githubusercontent.com/39609430/152670561-484cbd9d-8def-4851-97e0-e5c4ddbf6a1d.png)



This project is a simulator for autonomous flight in agricultural environment. 
It aims to help test different strategies of autonomous flights in simulated environments containing high-fidelity argricultral features. 

We created this project with Ubuntu environments. We recommend you to run it on Ubuntu 18.04 as the current AirSim version is not fully tested on future ubuntu distributions.

# Installation Guide

## 1. Install Prerequisite Libraries
The project is based on following common libraries. Make sure you have them installed on your computer. 
1. [Boost Library](https://www.boost.org/)
2. [Open CV](https://opencv.org/) (The oldest version we have tested is 3.2. We suggest using later version.) 
3. [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page) 

## 2. Install Unity
Unity editor is needed to run the simulated environment. 
An installation guide for Unity on Linux environment can be found [here](https://docs.unity3d.com/2020.1/Documentation/Manual/GettingStartedInstallingHub.html).

## 3. Clone Repository and Setup AirSim
Go to the directory where you want to install the code.
`git clone https://github.com/muellerlab/agri-fly` 

`cd AIFS_AirSim`

`./setup.sh`

`./build.sh`

Then navigate to the folder containing Unity wrapper code. 

`cd AirSim/Unity`

`./build.sh`

A detailed instruction for setting up AirSim project can also be found: 

`https://microsoft.github.io/AirSim/build_linux/`

`https://microsoft.github.io/AirSim/Unity/`


## 4. Setup Simulator Code Compilation
We recommend using cmake with graphic interface if you are not familiar with compiler setup.

* to get it: run `sudo apt install cmake-qt-gui`

* then open the app:

* where is the source code: $PATH TO REPOSITORY$

* where to build the binaries: $PATH TO REPOSITORY$/Build (this will ask to make new directory and say yes)

* Click Configure, then: 

* Select "Unix makefiles" and "Use default native compilers" 

* Click configure until the red items disappear 

* Click Generate 

Now the code is ready to be compiled.

# Quick Startup Guide For Single-thread Simulation

Notice that the single-thread simulation is extremely slow and we recommend to use it only as a tool to check if code installation is successful. We recommend to use ROS Simulator as basis for developement. 

* Start Unity Hub, import the argricultral world via selecting the folder 'AIFS_AirSim\Unity\UnityDemo', and then hit the OK button.

* Click on the new project which showed up in the Unity Hub menu to open it in Unity.

* In the bottom pane, click on Projects->Assets->Scenes. Then, double-click on SimModeSelector. Choose the Drone-Demo.

* Hit the play button to start the simulation (and hit play again to stop the simulation. 

* Then run the compiled Rappids_Simulator executive. You should see the vehicle taking off and trying to autonomously fly through the almond orchard.


# Quick Startup Guide For ROS Simulation
See readme.md inside the AIFS_ROS folder.


# Structure Overview
![image](https://user-images.githubusercontent.com/39609430/147999038-ac5ea2ee-9f68-4a50-ad56-389cf1e7840f.png)


1. The flight simulator code, including physics and example estimator and controller are based on work of [HiPeRLab](https://hiperlab.berkeley.edu/). The depth-camera based [RAPPIDS controller](https://hiperlab.berkeley.edu/wp-content/uploads/2020/11/2020_RectangularPyramid.pdf) is used in the onboard path planning example. 

2. The high-fidelity argricultral models used in the project comes from [Helios](https://baileylab.ucdavis.edu/software/helios/) developed by Bailey Lab of UC Davis.

3. Rendering of the argricultral world is implemented with [Unity](https://unity.com/). The tool is not open source and not free to use, but Unity provides personal/educator license that can be easily obtained without charge. 

4. The visualization and sensor fabrication is built upon the [AirSim](https://microsoft.github.io/AirSim/) and its pilot Unity-based simulation demo. 


# Acknowledgement
The project is supported by USDA-NIFA/NSF AI Institute for Next Generation Food Systems [AIFS](https://aifs.ucdavis.edu/)
![image](https://user-images.githubusercontent.com/39609430/147998908-35fdf682-6ddc-4d3a-9b42-f0877b539d82.png)

The high fidelity tree models in the simulator is generated with the [Helios](https://baileylab.ucdavis.edu/software/helios/) developed by Bailey lab of UC Davis.
![Helios_logo_small](https://user-images.githubusercontent.com/39609430/153678415-4192ebe7-d9f6-4d66-9a28-ed2dbb2a6443.png)


