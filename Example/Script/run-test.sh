#!/bin/bash
DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
# Local Build
# Regression Test
#convert @@EXPORTED_VALUE@@ tokens to value: $(echo ${EXPORTED_VALUE})
git clean -dfx ${DIR}/test/
${DIR}/test/RegressionTests/setup-tests.sh ${DIR}/test/RegressionTests/*/*.in
( make regression-test ; exit 0);
