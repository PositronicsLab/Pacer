#!/bin/bash

grep "u = " out.log > u.mat
grep "uff = " out.log > uff.mat
grep "ufb = " out.log > ufb.mat
grep "qdd = " out.log > qdd.mat
grep "qd = " out.log > qd.mat
grep "q = " out.log > q.mat
grep "qdd_des = " out.log > qdd_des.mat
grep "qd_des = " out.log > qd_des.mat
grep "q_des = " out.log > q_des.mat
grep "CoM_x = " out.log > com.mat
grep ".._FOOT_x = \[" out.log > x.mat
grep ".._FOOT_x_des = \[" out.log > x_des.mat
grep ".._FOOT_x_err = \[" out.log > x_err.mat
grep ".._FOOT_xd = \[" out.log > xd.mat
grep ".._FOOT_xd_des = \[" out.log > xd_des.mat
grep ".._FOOT_xd_err = \[" out.log > xd_err.mat

rpl -q "qdd = [" "" qdd.mat
rpl -q "qd = [" "" qd.mat
rpl -q "q = [" "" q.mat
rpl -q "qdd_des = [" "" qdd_des.mat
rpl -q "qd_des = [" "" qd_des.mat
rpl -q "q_des = [" "" q_des.mat
rpl -q "CoM_x = [" "" com.mat
rpl -q "LF_FOOT_x = [" "" x.mat
rpl -q "LF_FOOT_x_des = [" "" x_des.mat
rpl -q "LF_FOOT_x_err = [" "" x_err.mat
rpl -q "LF_FOOT_xd = [" "" xd.mat
rpl -q "LF_FOOT_xd_des = [" "" xd_des.mat
rpl -q "LF_FOOT_xd_err = [" "" xd_err.mat
rpl -q "RF_FOOT_x = [" "" x.mat
rpl -q "RF_FOOT_x_des = [" "" x_des.mat
rpl -q "RF_FOOT_x_err = [" "" x_err.mat
rpl -q "RF_FOOT_xd = [" "" xd.mat
rpl -q "RF_FOOT_xd_des = [" "" xd_des.mat
rpl -q "RF_FOOT_xd_err = [" "" xd_err.mat
rpl -q "LH_FOOT_x = [" "" x.mat
rpl -q "LH_FOOT_x_des = [" "" x_des.mat
rpl -q "LH_FOOT_x_err = [" "" x_err.mat
rpl -q "LH_FOOT_xd = [" "" xd.mat
rpl -q "LH_FOOT_xd_des = [" "" xd_des.mat
rpl -q "LH_FOOT_xd_err = [" "" xd_err.mat
rpl -q "RH_FOOT_x = [" "" x.mat
rpl -q "RH_FOOT_x_des = [" "" x_des.mat
rpl -q "RH_FOOT_x_err = [" "" x_err.mat
rpl -q "RH_FOOT_xd = [" "" xd.mat
rpl -q "RH_FOOT_xd_des = [" "" xd_des.mat
rpl -q "RH_FOOT_xd_err = [" "" xd_err.mat
rpl -q "ufb = [" "" ufb.mat
rpl -q "uff = [" "" uff.mat

rpl -q "]';" "" *.mat
