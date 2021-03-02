## Output
 - The terminal iteratively outputs the information of the parts detected by the `logical cameras` placed in the ARIAC environment. The structure of the output is as follows:

 ```
 [ ROS_INFO] [TIME_STAMP]: LOGICAL CAMERA DETECTED PARTS


*****  PART (# parts)  *****
===================
COLOR PART (# parts)
===================
# of type of part
Part ID: part + color + #
Sensor: logical camera that detects the part
Type: {disk, pulley, gasket, piston, gear}
Color: {red, blue, green}
Position: (position in world coordinates)
	x: 
	y: 
	z: 

Orientation: (orientation in world coordinates)
	x: 
	y: 
	z: 
	w: 

	roll: 
	pitch: 
	yaw: 

-------------------------
             .
             .
             .
------------!!!----------