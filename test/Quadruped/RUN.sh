#!/bin/bash
DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )

ls -d ./*/ | screen -m -d ./$DIR/../run_all_test_sequential.sh