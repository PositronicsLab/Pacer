# Perception, Planning and Control Plugins
plugin-src |	DESCRIPTION
------------|-------------------------------------------------------------
drive-robot |	 drives the robot to waypoints using SE2 base positon differential commands
stand-trajectory | tells the robot to maintain foot and joint positions at initial state
ik-feet | calcualtes inverse kinematics for foot trajectory commands in the frame relative to the base-link
stabilization | reactive controller -- applies joint forces tot he robot to maintain a desired base position and velocity
gait-planner | follows an SE2 base position differental goal by planning footsteps and foot trajectoies for the robot to walk/trot
joint-PID-controller | PID error-feedback controller for joint trajectory
eef-PID-controller | PID error-feedback controller for end effector trajectory
inverse-dynamics | Inverse dynamics controller with friction -- accepts a desired joint acceleration
