#!/bin/bash

grep "contacts = " out.log > c.mat
for i in `seq 0 3`;
do
grep "true_impulse_${i} = " out.log > i${i}t.mat
grep "true_point_${i} = " out.log > p${i}t.mat
grep "true_normal_${i} = " out.log > n${i}t.mat
grep "obs_impulse_${i} = " out.log > i${i}o.mat
grep "obs_point_${i} = " out.log > p${i}o.mat
grep "obs_normal_${i} = " out.log > n${i}o.mat
rpl -q "true_impulse_${i} = [" "" i${i}t.mat
rpl -q "true_point_${i} = [" "" p${i}t.mat
rpl -q "true_normal_${i} = [" "" n${i}t.mat
rpl -q "obs_impulse_${i} = [" "" i${i}o.mat
rpl -q "obs_point_${i} = [" "" p${i}o.mat
rpl -q "obs_normal_${i} = [" "" n${i}o.mat
done
rpl -q "contacts = [" "" c.mat
rpl -q "] ;" "" *.mat
