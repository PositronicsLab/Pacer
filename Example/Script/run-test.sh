#!/bin/bash
DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
# Local Build
# Regression Test
#convert @@EXPORTED_VALUE@@ tokens to value: $(echo ${EXPORTED_VALUE})
git clean -dfx ${PACER_HOME}
${PACER_SCRIPT_PATH}/setup-tests.sh ${PACER_HOME}/Example/Demo/*/*.in
( make regression-test ; exit 0);
