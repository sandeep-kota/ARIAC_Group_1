#!/bin/bash

# used in sample_sensor_blackout.yaml
# rosrun gazebo_ros spawn_model -sdf -x 0.0 -y -0.15 -z 0.05 -R 0 -P 0 -Y 0.78539816339 -file `rospack find nist_gear`/models/pulley_part_red_ariac/model.sdf -reference_frame agv1::kit_tray_1::kit_tray_1::tray -model red_pulley_20
# rosrun gazebo_ros spawn_model -sdf -x 0.0 -y 0.15 -z 0.05 -R 0 -P 0 -Y 0.78539816339 -file `rospack find nist_gear`/models/pulley_part_red_ariac/model.sdf -reference_frame agv1::kit_tray_1::kit_tray_1::tray -model red_pulley_21

# used in sample_interruption.yaml
# unwanted_products = 1
# rosrun gazebo_ros spawn_model -sdf -x 0.0 -y 0.15 -z 0.05 -R 0 -P 0 -Y 0.78539816339 -file `rospack find nist_gear`/models/gear_part_red_ariac/model.sdf -reference_frame agv1::kit_tray_1::kit_tray_1::tray -model gear_part_red_21
# wanted_products = 2
# rosrun gazebo_ros spawn_model -sdf -x 0.0 -y -0.15 -z 0.05 -R 0 -P 0 -Y 0.78539816339 -file `rospack find nist_gear`/models/gear_part_green_ariac/model.sdf -reference_frame agv1::kit_tray_1::kit_tray_1::tray -model gear_part_green_20
# rosrun gazebo_ros spawn_model -sdf -x 0.0 -y 0.15 -z 0.05 -R 0 -P 0 -Y 0.78539816339 -file `rospack find nist_gear`/models/gear_part_green_ariac/model.sdf -reference_frame agv1::kit_tray_1::kit_tray_1::tray -model gear_part_green_21

