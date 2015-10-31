#!/bin/bash
#rpl "/links/model.sdf" "${PACER_MODEL_PATH}/links/model.sdf" model.xml
DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
for var in "$@"
do
  pushd .
  cd "$var"
  sed -E "s|<SDF filename=\".*\"|<SDF filename=\"${PACER_MODEL_PATH}/${TESTING_ROBOT}/model.sdf\"|" model.xml > model.xml.bak && mv model.xml.bak model.xml 
  sed -E "s|<init-file type=\"file\">.*</init-file>|<init-file type=\"file\">${PACER_MODEL_PATH}/${TESTING_ROBOT}/init.xml</init-file>|" vars.xml > vars.xml.bak && mv vars.xml.bak vars.xml 
  popd
done