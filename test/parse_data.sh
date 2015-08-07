#!/bin/bash

grep "u = " out.log > u.mat
grep "uff = " out.log > uff.mat
grep "ufb = " out.log > ufb.mat
grep "qdd = " out.log > qdd.mat
grep "generalized_qd = " out.log > qd.mat
grep "q = " out.log > q.mat
grep "qdd_des = " out.log > qdd_des.mat
grep "qd_des = " out.log > qd_des.mat
grep "q_des = " out.log > q_des.mat
grep "CoM_x = " out.log > com.mat
grep "CoM_xd = " out.log > comxd.mat
grep ".._FOOT_x = \[" out.log > x.mat
grep ".._FOOT_x_des = \[" out.log > x_des.mat
grep ".._FOOT_x_err = \[" out.log > x_err.mat
grep ".._FOOT_xd = \[" out.log > xd.mat
grep ".._FOOT_xd_des = \[" out.log > xd_des.mat
grep ".._FOOT_xd_err = \[" out.log > xd_err.mat
grep "roll_pitch_yaw = " out.log > rpy.mat
grep "idyn_timing = " out.log > t_idyn.mat
grep "num_contacts = " out.log > nc.mat
grep "cf_moby = " out.log > cf_moby.mat
grep "cf_id = " out.log > cf_id.mat

rpl -q "qdd = [" "" qdd.mat
rpl -q "generalized_qd = [" "" qd.mat
rpl -q "q = [" "" q.mat
rpl -q "qdd_des = [" "" qdd_des.mat
rpl -q "qd_des = [" "" qd_des.mat
rpl -q "q_des = [" "" q_des.mat
rpl -q "CoM_x = [" "" com.mat
rpl -q "CoM_xd = [" "" comxd.mat
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
rpl -q "u = [" "" u.mat
rpl -q "roll_pitch_yaw = [" "" rpy.mat
rpl -q "idyn_timing = " "" t_idyn.mat
rpl -q "num_contacts = " "" nc.mat
rpl -q "cf_moby = [" "" cf_moby.mat
rpl -q "cf_id = [" "" cf_id.mat

rpl -q " ]';" "" *.mat
rpl -q " ]';" "" *.mat
rpl -q "]';" "" *.mat
rpl -q ";" "" *.mat
