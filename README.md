# ARIAC 2020 Group 1

## Build Instructions

- Download the ARIAC 2020 package from the ARIAC github repo in the source folder of your workspace.
```
mkdir -p ~/ariac_ws/src
cd ~/ariac_ws/src/
git clone https://github.com/usnistgov/ARIAC.git
cd ~/ariac_ws/src/ARIAC/
git checkout ariac2020
cd ~/ariac_ws/src/
git clone https://github.com/osrf/ariac-gazebo_ros_pkgs -b ariac-network-melodic
```

- Download this repo in the source directory
Run the following commands in a terminal

```
cd ~/ariac_ws/src/
git clone https://github.com/sandeep-kota/ARIAC_Group_1.git
catkin build
```


## Run Instructions
 - The launch directory of the package `/rwa2_group_1` has a launch file `rwa2.launch`. All the necessary nodes will be launched within this node.

 ```
source ~/ariac_ws/devel/setup.bash
roslaunch rwa2_group_1 rwa2.launch
 ```

 ## Output
 - The terminal iteratively outputs the information of the parts detected by the `logical cameras` placed in the ARIAC environment. The structure of the output is as follows:

 ```
 [ ROS_INFO] [TIME_STAMP]: LOGICAL CAMERA DETECTED PARTS


*****  PART (# parts)  *****
===================
COLOR PART (# parts)
===================
# of type of part
Part ID: part + colo r+ #
Sensor: logical camera that detects the part
Type: {disk, pulley, gasket, piston, gear}
Color: {red, blue, green}
Position:
	x: 
	y: 
	z: 
`````
Orientation:
	x: 
	y: 
	z: 
	w: 

	roll: 
	pitch: 
	yaw: 
````
-------------------------
             .
             .
             .
------------!!!----------            