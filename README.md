# bebop2_autonav

Autonomous control of the Bebop 2 drone to showcase the possibilty of automated material delivery on the shop floor.
**Authors:** Aakash Yadav, Aditya Yalamanchili and Ravi Srivatsa

### Environment setup
Install the following operating system and other packages. It is highly recommended to perform the setup in given order and to test Python/OpenCV before installing ROS
  - Ubuntu 16.04 LTS Xenial
  - Python 3+
  - OpenCV 2
  - ROS Kinetic Kame

<!--### New Features-->

<!--  - Import a HTML file and watch it magically convert to Markdown-->
<!--  - Drag and drop images (requires your Dropbox account be linked)-->
  
### Installation
##### Setting up environment. 
Visit the official bebop page [bebop-autonomy](https://bebop-autonomy.readthedocs.io/en/latest/installation.html)
```sh
# Install the required packages
$ sudo apt-get install build-essential python-rosdep python-catkin-tools
# Create and initialize the workspace
$ mkdir -p ~/bebop_ws/src && cd ~/bebop_ws
$ catkin init
$ git clone https://github.com/AutonomyLab/bebop_autonomy.git src/bebop_autonomy
# Update rosdep database and install dependencies (including parrot_arsdk)
$ rosdep update
$ rosdep install --from-paths src -i
# Build the workspace
$ catkin build
```
The build should complete without any errors
##### Takeoff and Landing using ROS
Caution : Before performing the take off please follow all the safety requirements.

```sh
# launching the bebop driver
$ roslaunch bebop_driver bebop_node.launch
```
Open a new terminal
```sh
# takeoff drone
$ rostopic pub --once bebop/takeoff std_msgs/Empty
# land drone
$ rostopic pub --once [namespace]/land std_msgs/Empty
```
##### Creating the package (1st time only)
```sh
$ cd ~bebop2_autonav/src/bebop_autonomy
$ catkin_create_pkg motion_plan2 rospy roscpp std_msgs geometry_msgs sensor_msgs
$ cd motion_plan2
$ mkdir scripts
```
##### Running a python script 
```sh
# place your ROS python scripts at cd ~bebop2_autonav/src/bebop_autonomy/scripts
# install dos2unix to resolve any character mismatch conflict
$ sudo apt-get install dos2unix
$ dos2unix my_file_name.py
# make the python file executible (not required for cpp)
$ cd ~bebop2_autonav/src/bebop_autonomy/scripts
$ chmod +x my_file_name.py
# run the script (launch the bebop driver first)
$ rosrun motion_plan2 my_file_name.py

```

### TODO
> The documentation needs to be further improved




