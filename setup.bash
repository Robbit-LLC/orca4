#!/usr/bin/env bash

# Modify this for your environment

# Add results of ArduSub build
export PATH=$HOME/ardupilot/build/sitl/bin:$PATH

# Optional: add autotest to the PATH, helpful for running sim_vehicle.py
export PATH=$HOME/ardupilot/Tools/autotest:$PATH

# Add results of colcon build
source $HOME/colcon_ws/install/setup.bash

# Add ardupilot_gazebo plugin
export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:$Z_SIM_SYSTEM_PLUGIN_PATH

# Optional: add ardupilot_gazebo models and worlds
export GZ_SIM_RESOURCE_PATH=$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:$GZ_SIM_RESOURCE_PATH

# Add bluerov2_ignition models and worlds
export GZ_SIM_RESOURCE_PATH=$HOME/colcon_ws/src/bluerov2_ignition/models:$HOME/colcon_ws/src/bluerov2_ignition/worlds:$GZ_SIM_RESOURCE_PATH

# Add orca4 models and worlds
export GZ_SIM_RESOURCE_PATH=$HOME/colcon_ws/src/orca4/orca_description/models:$HOME/colcon_ws/src/orca4/orca_description/worlds:$GZ_SIM_RESOURCE_PATH

# Build ros_gz on the humble branch for Gazebo Garden
export GZ_VERSION=garden

# for future use - to support multiple Gazebo versions
export GZ_VERSION=garden

# not usually required as should default to localhost address
export GZ_IP=127.0.0.1

# ensure the model and world files are found
export GZ_SIM_RESOURCE_PATH=\
$GZ_SIM_RESOURCE_PATH:\
$HOME/gz_ws/src/asv_wave_sim/gz-waves-models/models:\
$HOME/gz_ws/src/asv_wave_sim/gz-waves-models/world_models:\
$HOME/gz_ws/src/asv_wave_sim/gz-waves-models/worlds

# ensure the system plugins are found
export GZ_SIM_SYSTEM_PLUGIN_PATH=\
$GZ_SIM_SYSTEM_PLUGIN_PATH:\
$HOME/gz_ws/src/install/lib

# ensure the gui plugin is found
export GZ_GUI_PLUGIN_PATH=\
$GZ_GUI_PLUGIN_PATH:\
$HOME/gz_ws/src/asv_wave_sim/gz-waves/src/gui/plugins/waves_control/build
