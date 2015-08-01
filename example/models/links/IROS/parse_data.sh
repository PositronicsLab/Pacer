#!/bin/bash

grep "Final Energy r" err.log > eR.mat
grep "Final Energy e" err.log > eA.mat
grep "w/ h" err.log > h.mat
grep "w/h" err.log > h2.mat
grep "Final Energy -- 1 steps (LINKS) " err.log > e1.mat
grep "Final Energy -- 2 steps (LINKS) " err.log > e2.mat
rpl -q "Final Energy error (LINKS) " "" eA.mat 
rpl -q "Final Energy relative error (LINKS) " "" eR.mat 
rpl -q "Safe integration ended w/ h = " "" h.mat 
rpl -q "Position integration ended w/h = " "" h2.mat 
rpl -q "Final Energy -- 2 steps (LINKS) " "" e2.mat
rpl -q "Final Energy -- 1 steps (LINKS) " "" e1.mat
