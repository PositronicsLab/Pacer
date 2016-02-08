#!/bin/bash
git clean -dfx ${PACER_HOME}/Example/Demo/*
cd ${PACER_BINARY_PATH}
source setup.sh
${PACER_SCRIPT_PATH}/setup-tests.sh ${PACER_HOME}/Example/Demo/*/*.in
( make regression-test ; exit 0);
