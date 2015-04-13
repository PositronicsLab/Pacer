#!/bin/bash

# more sophisticated script for rendering a movie that takes in dynamic
# camera positions and targets 

  # commands like this can be used to modify the osg files
  #rpl -q "ColorMode DIFFUSE" "ColorMode AMBIENT_AND_DIFFUSE" $1/*.osg

  # renders the osg files to images in order expected by ffmpeg
  a=1
  for i in $1/driver.out-*.osg; do
    new=$(printf "img%04d.png" ${a});
    let a=a+1;
    POS=($(awk 'NR == n' n=$(($2 * ${a})) $1/x.mat));

    #moby-render $i -p 5 3 1  -t $(awk 'NR == n' n=$(($2 * ${a})) $1/com.mat) $1/${new};
    ~/Projects/Moby/build/moby-render $i -s scene.osg -p $(awk "BEGIN {print ${POS[0]}+1.0}") -0.5 0.5  -t ${POS[*]} $1/${new};
  done

  # render at 25fps 
  ffmpeg -r 25 -i $1/img%04d.png -f mp4 -q:v 0 -vcodec mpeg4 $1/movie.mp4
