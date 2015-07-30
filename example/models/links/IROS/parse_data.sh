#!/bin/bash

grep "idyn_NC = " out.log > nc.mat
rpl -q "idyn_NC = " "" nc.mat

for i in `seq 0 1`;
do
  grep "$i, Sum normal force: " out.log > n${i}.mat
  grep "cf_$i = " out.log > cf${i}.mat
  grep "uff_$i = " out.log > uff${i}.mat

  rpl -q "$i, Sum normal force: " "" n${i}.mat
  rpl -q "cf_$i = [" "" cf${i}.mat
  rpl -q "uff_$i = [" "" uff${i}.mat
done

rpl -q " ]';" "" *.mat
rpl -q "]';" "" *.mat
rpl -q ";" "" *.mat
