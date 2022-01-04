# Flight Simulator in Agricultural Environment
This project is a simulator for autonomous flight in agricultural environment. 
It can be used to test different strategies of autonomous flights in simulated environments containing high-fidelity argricultral features. 

We expect this project to run on Ubuntu environments. We have tested it on 18.04 and it should in theory run on future versions as well (Though it has not been tested).

# Installation Guide

## Install Prerequisite Libraries
The project is based on following common libraries. Make sure you have them installed on your computer. 
1. [Boost Library](https://www.boost.org/)
2. [Open CV 3.2](https://opencv.org/opencv-3-2/)
3. [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page)

## Install Unity
Unity editor is needed to run the simulated environment. 
An installation guide for Unity on Linux environment can be found [here](https://docs.unity3d.com/Manual/GettingStartedInstallingHub.html).


# Structure Overview
![image](https://user-images.githubusercontent.com/39609430/147999038-ac5ea2ee-9f68-4a50-ad56-389cf1e7840f.png)


1. The flight simulator code, including physics and example estimator and controller are based on work of [HiPeRLab](https://hiperlab.berkeley.edu/). The depth-camera based [RAPPIDS controller](https://hiperlab.berkeley.edu/wp-content/uploads/2020/11/2020_RectangularPyramid.pdf) is used in the onboard path planning example. 

2. The high-fidelity argricultral models used in the project comes from [Helios](https://baileylab.ucdavis.edu/software/helios/) developed by Bailey Lab of UC Davis.

3. Rendering of the argricultral world is implemented with [Unity](https://unity.com/). The tool is not open source and not free to use, but Unity provides personal/educator license that can be easily obtained without charge. 

4. The visualization and sensor fabrication is built upon the [AirSim](https://microsoft.github.io/AirSim/) and its pilot Unity-based simulation demo. 






# Acknowledgement
The project is supported by USDA-NIFA/NSF AI Institute for Next Generation Food Systems (AIFS)(https://aifs.ucdavis.edu/)
![image](https://user-images.githubusercontent.com/39609430/147998908-35fdf682-6ddc-4d3a-9b42-f0877b539d82.png)

