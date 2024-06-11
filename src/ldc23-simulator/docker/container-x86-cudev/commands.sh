#!/usr/bin/env bash

# Project-specific shell functions and commands.
#
# Roberto Masocco <robmasocco@gmail.com>
# Intelligent Systems Lab <isl.torvergata@gmail.com>
#
# April 4, 2023

# Add yours, some convenient ones are provided below.
# You can also source other files from sub-units included by this project.

# Routine to convert an angle in degrees [-180° +180°] to radians [-PI +PI]
function degrad {
  if [[ $# -ne 1 ]] || [[ $1 -lt -180 ]] || [[ $1 -gt 180 ]]; then
    echo >&2 "Usage:"
    echo >&2 "    degrad ANGLE"
    echo >&2 "ANGLE must be in degrees and in [-180° +180°]"
    return 1
  fi
  local OP
  local FIRST
  local SECOND
  local RES
  OP="scale=6;$1*3.14159265359/180.0"
  RES="$(bc <<<"$OP")"
  FIRST="${RES:0:1}"
  SECOND="${RES:1:1}"
  if [[ $FIRST == "." ]]; then
    RES="0${RES}"
  fi
  if [[ $FIRST == "-" ]] && [[ $SECOND == "." ]]; then
    RES="-0.${RES:2:6}"
  fi
  echo "$RES"
}

# Routine to configure Gazebo Classic environment to find PX4 plugins and models
function gazeboenv {
  export SITL_GAZEBO_PATH=$HOME/workspace/src/px4_sitl_gazebo
  export GAZEBO_MODEL_DATABASE_URI=""
  export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:$SITL_GAZEBO_PATH/models
  export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:$HOME/workspace/build/px4_sitl_gazebo
}

function make_sdf {
  if [[ $# -ne 1 ]]; then
    echo >&2 "Usage:"
    echo >&2 "    make_sdf MODEL"
    echo >&2 "MODEL must be one of: axis_camera, seppia, stanis, with relative configurations (i.e. the name of the urdf.xacro file)"
    return 1
  fi

  local MODEL_NAME
  # Check if the model contains the substring "stanis"
  if [[ $1 == *"stanis"* ]]; then
    MODEL_NAME="stanis"
  elif [[ $1 == *"seppia"* ]]; then
    MODEL_NAME="seppia"
  elif [[ $1 == *"axis_camera"* ]]; then
    MODEL_NAME="axis_camera"
  else
    echo >&2 "ERROR: Invalid model"
    return 1
  fi

  # Generate the URDF file from the xacro file
  ros2 run xacro xacro \
    /home/neo/workspace/src/ldc23-robot-description/src/${MODEL_NAME}_description/urdf/"$1".urdf.xacro \
      -o /home/neo/workspace/src/ldc23-robot-description/src/${MODEL_NAME}_description/urdf/${MODEL_NAME}.urdf \
  || return 1

  # Generate the SDF file from the URDF file
  ign sdf -p \
    /home/neo/workspace/src/ldc23-robot-description/src/${MODEL_NAME}_description/urdf/${MODEL_NAME}.urdf \
    > /home/neo/workspace/config/gazebo/models/${MODEL_NAME}/model.sdf

  rm /home/neo/workspace/src/ldc23-robot-description/src/${MODEL_NAME}_description/urdf/${MODEL_NAME}.urdf
}
