# cepheus simulator
ros-gazebo project for [Space Robotics Team](http://csl-ep.mech.ntua.gr/index.php/research/robotics-for-extreme-environments/space-robotics)@[Control Systems Lab, NTUA](http://csl-ep.mech.ntua.gr/ "Lab website")


ROS version: melodic

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
