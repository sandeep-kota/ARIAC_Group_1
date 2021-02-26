#!/bin/bash

rosrun gazebo_ros spawn_model -sdf -x 0.0 -y -0.2 -z 0.05 -R 3.14159 -P 0 -Y 0 -file `rospack find nist_gear`/models/pulley_part_red_ariac/model.sdf -reference_frame agv2::kit_tray_2::kit_tray_2::tray -model pulley_part_red_20

rosrun gazebo_ros spawn_model -sdf -x 0.0 -y 0 -z 0.05 -R 3.14159 -P 0 -Y 0 -file `rospack find nist_gear`/models/pulley_part_blue_ariac/model.sdf -reference_frame agv1::kit_tray_1::kit_tray_1::tray -model pulley_part_blue_10