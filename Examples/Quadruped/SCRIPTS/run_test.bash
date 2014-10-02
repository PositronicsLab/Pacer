#!/bin/bash

/usr/local/bin/moby-driver -y=osg -v=10 -s=0.00101 -mt=10 -p=../../libLinksPlugin.so links1.xml > out.log 2>err.log
