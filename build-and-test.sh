#!/bin/bash
# Local Build
mkdir $1; cd $1;
cmake -DBUILD_EXAMPLES=ON -DBUILD_INTERFACES=ON -DBUILD_PLUGINS=ON -DBUILD_REGRESSION_TESTS=ON -DBUILD_UNIT_TESTS=OFF -DBUILD_TESTS=ON -DUSE_OSG_DISPLAY=ON -DCMAKE_BUILD_TYPE=$1 ..;
make -j 4;
# Regression Test
export GTEST_OUTPUT="xml:./$1/";
source $1/setup.sh;
#convert @@EXPORTED_VALUE@@ tokens to value: $(echo ${EXPORTED_VALUE})
./test/RegressionTests/setup-tests.sh ./test/RegressionTests/*/*.in
( make regression-test ; exit 0);
