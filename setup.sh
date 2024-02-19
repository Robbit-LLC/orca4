#!/usr/bin/env bash

# Modify this for your environment

export PROJECT_PATH=$(pwd)/..

# Add results of ArduSub build
export PATH=$PROJECT_PATH/ardupilot/build/sitl/bin:$PATH

# Optional: add autotest to the PATH, helpful for running sim_vehicle.py
export PATH=$PROJECT_PATH/ardupilot/Tools/autotest:$PATH

# Add results of colcon build
source $PROJECT_PATH/colcon_ws/install/setup.sh

# Add ardupilot_gazebo plugin
export GZ_SIM_SYSTEM_PLUGIN_PATH=$PROJECT_PATH/ardupilot_gazebo/build:$Z_SIM_SYSTEM_PLUGIN_PATH

# Optional: add ardupilot_gazebo models and worlds
export GZ_SIM_RESOURCE_PATH=$PROJECT_PATH/ardupilot_gazebo/models:$PROJECT_PATH/ardupilot_gazebo/worlds:$GZ_SIM_RESOURCE_PATH

# Add bluerov2_ignition models and worlds
export GZ_SIM_RESOURCE_PATH=$PROJECT_PATH/colcon_ws/src/bluerov2_ignition/models:$PROJECT_PATH/colcon_ws/src/bluerov2_ignition/worlds:$GZ_SIM_RESOURCE_PATH

# Add orca4 models and worlds
export GZ_SIM_RESOURCE_PATH=$PROJECT_PATH/colcon_ws/src/orca4/orca_description/gazebo/models:$PROJECT_PATH/colcon_ws/src/orca4/orca_description/gazebo/worlds:$GZ_SIM_RESOURCE_PATH

export GZ_SIM_SYSTEM_PLUGIN_PATH=$PROJECT_PATH/colcon_ws/install/orca_description/lib/plugins:$GZ_SIM_SYSTEM_PLUGIN_PATH

# Build ros_gz on the humble branch for Gazebo Garden
export GZ_VERSION=garden
