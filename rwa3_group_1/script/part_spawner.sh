#!/bin/bash

# spawn a part in the tray on agv1
rosrun gazebo_ros spawn_model -sdf -x -0.1 -y -0.2 -z 0.05 -R 0 -P 0 -Y 0.78539816339 -file `rospack find nist_gear`/models/pulley_part_red_ariac/model.sdf -reference_frame agv2::kit_tray_2::kit_tray_2::tray -model red_pulley_20
