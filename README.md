# cepheus simulator
ros-gazebo project for [Space Robotics Team](http://csl-ep.mech.ntua.gr/index.php/research/robotics-for-extreme-environments/space-robotics)@[Control Systems Lab, NTUA](http://csl-ep.mech.ntua.gr/ "Lab website")


Requirements
------------
ROS version: melodic, and:
```bash
sudo apt install ros-melodic-ros-controllers
sudo apt install ros-melodic-rosbridge-server
sudo apt install ros-melodic-web-video-server # reason not upgrading to noetic
```

Use Control Panel UI
--------------------
```bash
cd {PATH}/src/cepheus_interface/server
npm install
npm run dev
# New terminal/tab
cd {PATH}/src/cepheus_interface/web_ui
npm install
npm start
```
Then visit http://localhost:3000/

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
