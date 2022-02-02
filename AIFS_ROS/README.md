# 1- Installing ROS

If you haven't already installed ROS-Kinetic, follow the instructions listed here:
* https://wiki.ros.org/melodic/Installation/Ubuntu
Note: if you are using Ubuntu 20.04 LTS, [ROS-noetic](http://wiki.ros.org/noetic/Installation/Ubuntu) is official supported version

# 2-Setting up your workspace
We suggest use python catkin tools to build ROS projects. Make sure you have the python catkin tools installed:
  * `sudo apt install python-catkin-tools`
  
For more information you can see: http://catkin-tools.readthedocs.io/en/latest/installing.html

Now, create a workspace folder:
  * `$ mkdir -p ~/catkin_ws/src`  
  * `$ cd ~/catkin_ws/`
  
We suggest you use a fresh work space for this simulator to prevent possible cross-contamination. Instructions on using multiple work spaces can be found here:
  * https://roboticsbackend.com/ros-multiple-catkin-workspaces/

Then, just build an empty project by:
  * `$ catkin build`

This is just to see you can build projects using python catkin tools. NOTE: If you used `catkin_make` before you cannot build, first do `catkin_make clean`, then `catkin build`.

# 3-Setup symlinks
Now `cd` into your ROS ws folder, and make the needed symlinks between the simulator and the work space

* `ln -s {Path to your project folder}/AIFS_ROS/hiperlab_common/ src/`
* `ln -s {Path to your project folder}/AIFS_ROS/hiperlab_components/ src/`
* `ln -s {Path to your project folder}/AIFS_ROS/hiperlab_rostools/ src/`
* `ln -s {Path to your project folder}/AIFS_ROS/hiperlab_hardware/ src/`
* `ln -s {Path to your project folder}/AIFS_ROS/makeProjects_AIFS.sh `

Then we need to link the source files for common & components:
* `ln -s {Path to your project folder}/Common/ src/hiperlab_common/src`
* `ln -s {Path to your project folder}/Components/ src/hiperlab_components/src`

# 4-Setup AirSim Root in buildfile
In AIFS_ROS/hiperlab_rostools.CMakeLists.txt,
Set the AIRSIM_ROOT to the folder you store airsim on your computer. 
i.e.:
* set(AIRSIM_ROOT {Path to your project folder}/AIFS_AirSim)  


# 5-Build the workspace
Finally, run `sh makeProjects_AIFS.sh`

Before continuing source your new setup.*sh file:
  * `source devel/setup.bash`

To make sure your workspace is properly overlayed by the setup script, make sure ROS_PACKAGE_PATH environment variable includes the directory you're in.
  * `$ echo $ROS_PACKAGE_PATH`

By echo you have to see something like:
`/home/your_username/catkin_ws/src/hiperlab_common:/home/your_username/catkin_ws/src/hiperlab_components:/home/your_username/catkin_ws/src/hiperlab_rostools:/home/your_username/catkin_ws/src/hiperlab_hardware:/opt/ros/kinetic/share`



--------------------------------------------------------------------------------------------
# Quick startup guide

# 1- Start the simulator in Unity
* Start Unity Hub, import the argricultral world via selecting the folder 'AIFS_AirSim\Unity\UnityDemo', and then hit the OK button.

* Click on the new project which showed up in the Unity Hub menu to open it in Unity.

* In the bottom pane, click on Projects->Assets->Scenes. Then, double-click on SimModeSelector. Choose the Drone-Demo.

* Hit the play button to start.

# 2- Run the ROS simulator
* Start ROS by running `roscore`

* Start the AirSim Bridge node by running `rosrun hiperlab_rostools air_sim_bridge`. The AirSim Bridge receives synthesized camera images from Unity and convert it to ROS compatible messages.

* Start the simulator node by running `rosrun hiperlab_rostools simulator 1` change `1` to other vehicle IDs if you are not running with the default vehicle. 

* Start the RAPPIDS planner and controller node `rosrun hiperlab_rostools quad_rappids_planner_controller 1`. Again, change `1` to your vehicle IDs if you are running with a different vehicle.   

* Following the instruction given by the RAPPIDS node and hit 's' button on the keyboard multiple times to start the flight controller.

* To end the simulation, kill the ROS nodes first before you stop the Unity simulated world.

# 3- Record bag files
* Copy and paste `rosbag_record_airsim.sh` to places you want to record data at.
* Run 'rosbag_record_airsim.sh' to record ROS bag files.


