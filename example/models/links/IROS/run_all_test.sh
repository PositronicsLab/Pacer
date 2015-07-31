#!/bin/bash
DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
for var in "$@"
do
      pushd .
      cd "$var"
      $DIR/clean_data.sh .
      screen -d -m ./RUN.sh 
      popd
done
