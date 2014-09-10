#!/bin/bash

/usr/local/bin/moby-driver -y=osg -v=10 -s=0.001 -mt=65 -p=../../libLinksPlugin.so links.xml > out.log
