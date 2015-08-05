### IROS 2015 Experiments Branch of Pacer

If there are any issues reproducing experimental data please submit a bug report to this branch so we can correct any issues, Thanks!

Requires: (specific versions of dependencies)

- [Moby]
branch: stable-int
commit: [85a526c19efb0f62e52f95e8459cdad694b65293]

[85a526c19efb0f62e52f95e8459cdad694b65293]: https://github.com/PositronicsLab/Moby/tree/85a526c19efb0f62e52f95e8459cdad694b65293

- [Ravelin] 
branch: accel-update
commit: [930f9f826f76d825f7fe554afb4d6b561bac025d]
[930f9f826f76d825f7fe554afb4d6b561bac025d]: https://github.com/PositronicsLab/Ravelin/tree/930f9f826f76d825f7fe554afb4d6b561bac025d
[Moby]: https://github.com/PositronicsLab/Moby
[Ravelin]: https://github.com/PositronicsLab/Ravelin

#Building:
In the Pacer base directory:
```
$ mkdir build
$ cd build
$ ccmake .. # configure cmake project
$ make all
```
#Running
```
$ source setup.sh
$ cd $PACER_HOME/example/models/links/IROS
```
This directory is organized as:
```
IROS/<experiment>/<sample>
```
each `<sample>` directory contains a `RUN.sh` script that will start Moby and collect data for the experiment. 
After the sample has completed execution, running the script `IROS/parse_all_data.sh <sample directory paths>` will generate MATLAB readable data files:


In directory `${PACER_HOME}/example/models/links/IROS`, run `./run_all_tests.sh */*` to run all tests then `./parse_all_data.sh */*` to generate all data for the experiments.

Moby must be installed on the system for the `RUN.sh` scripts to function properly.


 EXPERIMENT | DESCRIPTION
----------- | ------------------------
stepsize    | Measure how quickly larger step sizes destabilize simulationi for a walking quadruped (Section IV-A: [IROS '15 paper])
euler    | Measure how much explicity versus semi-implicit first-order Euler integration affects simulation stability for a walking quadruped (Section IV-C: [IROS '15 paper])
adaptive    | Measure the largest step size for a walking and standing quadruped (Section IV-D: [IROS '15 paper])
exponential | Measure how much exponential dissipation stabilizes simulation for a walking robot (Section IV-E: [IROS '15 paper])
rayleigh | Measure how much rayleigh dissipation stabilizes simulation for a walking robot
baumgarte | Measure how much baumgarte stabilization (error-correction) destabilizes simulation for a walking robot

octave/MATLAB scripts `plot_data.m` and `plot_data2.m` are provided to assist the user in parsing and plotting experimental data.

[IROS '15 paper]: http://robotics.gwu.edu/positronics/wp-content/uploads/2014/08/IROS2015-Zapolsky.pdf

###Pacer Project Repository
by Sam Zapolsky @ the [Positronics Lab]

[Positronics Lab]: http://robotics.gwu.edu/positronics/

This repository hosts code for [Pacer] an open source robot locomotion library for simulated and physical legged robots.

[Pacer]: https://github.com/PositronicsLab/Pacer

## Installation/building

[Pacer] currently must be built from source. Debian binaries will be available soon.

In the project base directory:
```
$ mkdir build
$ cd build
$ ccmake .. # configure cmake project
$ make all
```
Make sure that all Plugins and Interfaces build successfully

##Running Pacer (Example)

In the project base directory (bash terminal):
```
$ source setup.sh
$ cd $PACER_HOME/example/models/links
```
in Gazebo:
```
$ gazebo links.world
```
in Moby:
```
$ moby-driver -p=$PACER_HOME/build/example/interfaces/libPacerMobyPlugin.so model.xml
```
###Dependencies 

##Required 

- [Ravelin] 
- [Moby] 
- Boost 

##Optional 

- OpenSceneGraph
- [DXL]
- SDL (permits joystick control)

[Ravelin]: https://github.com/PositronicsLab/Ravelin
[Moby]: https://github.com/PositronicsLab/Moby
[DXL]: https://github.com/samzapo/DynamixelDriver

###Usage

Access the library by using the examples found in the examples directory:

 EXAMPLE       |  DESCRIPTION
--------------|---------------------------------------------------------
 interfaces     |  Moby & Gazebo controller plugins communicte with Pacer
 models        |  Pre-implemented robot models and vars files (see models/README.md)
 plugins   |    Discrete control modules for the robot.  Add capabilites to Pacer by adding plugins here.

## Exporting Data to MATLAB

 Logs are either output to terminal or written to the file: "out.log" in the run directory.  Running Pacer will overwrite the current instance of "./out.log"

 Vectors and Matrices output is readable in Matlab.  The examples in [scripts] demonstrate exporting output from Pacer's logs to MATLAB readable space delimited files.

[scripts]: https://github.com/PositronicsLab/Pacer/tree/master/examples/test-scripts

### File Structure

 FILE/DIRECTORY  |  DESCRIPTION
---------------- | ---------------------------------------------------------
 CMakeLists.txt  |  CMake build file for Pacer
 example         |  Data files and examples for Pacer
 include         |  Header files for calling functions in Pacer
 src             |  Pacer source code
 setup.sh        |  source this file to configure Pacer and Gazebo environment variables
