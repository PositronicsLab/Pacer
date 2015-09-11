#!/bin/bash

read -p "enter directory that contains 'moby-driver': " -e path

${path}/moby-driver -y=osg -v=10 -s=0.001 -p=${PACER_INTERFACE_PATH}/libPacerMobyPlugin.so model.xml > cout.log 2> cerr.log
