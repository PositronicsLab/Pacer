#!/bin/bash

export GAZEBO_MODEL_PATH=$PWD/..:$GAZEBO_MODEL_PATH
export GAZEBO_PLUGIN_PATH=$PWD/../../control-gazebo:$GAZEBO_PLUGIN_PATH

screen -m -d gzserver model.world
gzclient
