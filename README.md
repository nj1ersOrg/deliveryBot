# "Delivery" Code


### ROS package dependencies
The morph and delivery stack uses the Turtlebot stack and ros_control. Follow [installation instructions for the Turtlebot stack](http://wiki.ros.org/turtlebot/Tutorials/indigo/Turtlebot%20Installation).
Other dependencies are linked as submodule from the morph repository.
```
sudo apt-get install ros-kinetic-ros-control ros-kinetic-ros-controllers
```

### Create a catkin_workspace
```
mkdir -p delivery/src
cd delivery
catkin_make
```

### Checkout and initialize and update submodules
```
cd delivery/src
git clone https://github.com/nisalj/deliveryBot
cd deliveryBot
git submodule init
git submodule update
```
### Compile code
```
cd delivery
catkin_make
```
