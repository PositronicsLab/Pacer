#!/bin/bash
DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
for var in "$@"
do
# remove .in from name
  newvar=$(echo $var | sed 's#\(.*\)\.in#\1#' )
# create new file
  cp -f $var $newvar
# replace environment vars
  sed -E -i.bak "s#\@\@PACER_MODEL_PATH\@\@#${PACER_MODEL_PATH}#g" $newvar
  sed -E -i.bak "s#\@\@TESTING_ROBOT\@\@#${TESTING_ROBOT}#g" $newvar
#newvar="${var}.in"
#cp -f $var $newvar
#sed -E -i.bak "s#/links/#/\@\@TESTING_ROBOT\@\@/#g" $newvar
#sed -E -i.bak "s#../../../example/models/#\@\@PACER_MODEL_PATH\@\@/#g" $newvar
done
