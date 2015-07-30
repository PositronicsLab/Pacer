#!/bin/bash

for var in "$@"
do
      pushd .
      cd "$var"
      ../../parse_data.sh 
      popd
done
