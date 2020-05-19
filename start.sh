#!/bin/bash
RED='\033[0;31m'
LB='\033[1;34m'
LG='\033[0;33m'
NC='\033[0m' # No Color
printf "${LB}[Step 1]${NC} Remove ${LG}build${NC} and ${LG}devel${NC} directories... "
rm -rf ../../build ../../devel
printf "Done.\n"
printf "${LB}[Step 2]${NC} Build ${LG}cepheus${NC}... "
catkin_make
printf "Done.\n"
printf "${LB}[Step 3]${NC} Source ${LG}setup.bash${NC}... "
source ./devel/setup.bash
printf "Done.\n"
printf "${LB}[Step 4]${NC} Launch ${LG}cepheus model${NC}... "
gnome-terminal -- roslaunch cepheus_gazebo cepheus_world.launch
printf "Done.\n"
