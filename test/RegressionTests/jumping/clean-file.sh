#!/bin/bash
sed -i.bak '/[a-z]/d' $1 
sed -i.bak 's/\[//g' $1
sed -i.bak 's/\]//g' $1
sed -i.bak 's/,//g' $1

