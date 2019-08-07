# bebop2_autonav
***

Autonomous control of the Bebop 2 drone to showcase the possibilty of automated material delivery on the shop floor.
**Authors:** Aakash Yadav, Aditya Yalamanchili and Ravi Srivatsa

### Environment setup
Install the following operating system and other packages. It is highly recommended to perform the setup in given order and to test Python/OpenCV before installing ROS
  - Ubuntu 16.04 LTS Xenial
  - Python 3+
  - OpenCV 2
  - ROS Kinetic Kame

### New Features

  - Import a HTML file and watch it magically convert to Markdown
  - Drag and drop images (requires your Dropbox account be linked)
  
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

$ sudo apt-get install dos2unix
$ dos2unix file.py

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

##### Fetching the live image frames from drone using custom subscriber

```sh
$ npm install --production
$ NODE_ENV=production node app
```

### Plugins
Markdown is a lightweight markup language based on the formatting conventions that people naturally use in email.  As [John Gruber] writes on the [Markdown site][df1]

> The overriding design goal for Markdown's
> formatting syntax is to make it as readable
> as possible. The idea is that a
> Markdown-formatted document should be



