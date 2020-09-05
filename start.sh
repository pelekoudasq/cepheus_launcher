#!/bin/bash
RED='\033[0;31m'
LB='\033[1;34m'
LG='\033[0;33m'
NC='\033[0m' # No Color

# first argument
CONTROLLER=$1
# echo $CONTROLLER
# printf "${LB}[1/4]${NC} Remove ${LG}build${NC} and ${LG}devel${NC} directories... "
# rm -rf ./build ./devel
# printf "Done.\n"
printf "[1/3] Build cepheus... "
catkin_make
printf "Done.\n"
printf "[2/3] Source setup.bash... "
source ./devel/setup.bash
printf "Done.\n"
printf "[3/3] Launch cepheus model... "
gnome-terminal -- roslaunch cepheus_gazebo cepheus_world.launch controller:=$CONTROLLER
printf "Done.\n"
