# cepheus simulator
ros-gazebo project for [Space Robotics Team](http://csl-ep.mech.ntua.gr/index.php/research/robotics-for-extreme-environments/space-robotics)@[Control Systems Lab, NTUA](http://csl-ep.mech.ntua.gr/ "Lab website")


Control Panel UI
----------------
```bash
docker-compose up
```
Then visit http://localhost

![alt text](src/cepheus_interface/docs/screen-01.png)

Simulation run:
![alt text](src/cepheus_interface/docs/screen-02.png)

Simulation Requirements
-----------------------
ROS version: melodic, and:
```bash
sudo apt install ros-melodic-ros-controllers
sudo apt install ros-melodic-rosbridge-server
sudo apt install ros-melodic-web-video-server # reason not upgrading to noetic
```

Build
-----
```bash
cd catkin_simulator
catkin_make
source {PATH}/devel/setup.bash
```

Spawn Model
-----------
```bash
roslaunch cepheus_gazebo cepheus_world.launch
```
