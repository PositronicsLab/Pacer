#!/bin/bash
pushd .
DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
cd $DIR
tar -czvf ~/Dropbox/data.tar.gz */*/*.mat
popd
