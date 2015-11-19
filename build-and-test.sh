#!/bin/bash
# Local Build
mkdir $1; cd $1;
cmake -DBUILD_EXAMPLES=ON -DBUILD_INTERFACES=ON -DBUILD_PLUGINS=ON -DBUILD_REGRESSION_TESTS=ON -DBUILD_UNIT_TESTS=OFF -DBUILD_TESTS=ON -DUSE_OSG_DISPLAY=ON -DCMAKE_BUILD_TYPE=$1 ..;
make -j 4;
# Regression Test
export GTEST_OUTPUT="xml:./";
source setup.sh;
( make regression-test ; exit 0);
