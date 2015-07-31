#!/bin/bash

grep "Final Energy r" err.log > eR.mat
grep "Final Energy e" err.log > eA.mat
grep "w/ h" err.log > h.mat
