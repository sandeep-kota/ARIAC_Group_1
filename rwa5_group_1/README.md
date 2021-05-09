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

git checkout final

catkin build
```

---
## Run Instructions
 - The launch directory of the package `/rwa5_group_1` has a launch file `rwa5.launch`. All the necessary nodes will be launched within this node.

 ```
source ~/ariac_ws/devel/setup.bash

roslaunch rwa5_group_1 rwa5.launch load_moveit:=true
 ```

 - Run the rwa5 node 
 ```
source ~/ariac_ws/devel/setup.bash

rosrun rwa5_group_1 rwa5_node
 ``` 


---
 ## [Output Video](https://drive.google.com/file/d/1501NE050lnaqLuepaXyWLED4M8BpvkMU/view?usp=sharing)

 [![alt text](./docs/output_score/video_thumbnail.PNG?raw=true "Final Output Video")](https://drive.google.com/file/d/1501NE050lnaqLuepaXyWLED4M8BpvkMU/view?usp=sharing)


 ---
 ## Output Score for final run (config file: [final_ariac_2021.yaml](./config/final_ariac_2021.yaml))

 ![alt text](./docs/output_score/final_score1.png?raw=true "Final Score")

 ![alt text](./docs/output_score/final_score2.png?raw=true "Final Score")

 ![alt text](./docs/output_score/final_score3.png?raw=true "Final Score")




