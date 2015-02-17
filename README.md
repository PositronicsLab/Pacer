#Pacer Project Repository
by Sam Zapolsky @ the [Positronics Lab]

[Positronics Lab]: http://robotics.gwu.edu/positronics/

This repository hosts code for [Pacer] an open source robot locomotion library for simulation and physical robots.

[Pacer]: https://github.com/PositronicsLab/Pacer

###Dependencies 

##Required 

- [Ravelin] 
- [Moby] 
- Boost 
- [CVars] 

##Optional 

- Open Scene Graph
- [GLConsole] 
- [DXL]

[Ravelin]: https://github.com/PositronicsLab/Ravelin
[Moby]: https://github.com/PositronicsLab/Moby
[CVars]: https://github.com/arpg/GLConsole
[GLConsole]: https://github.com/arpg/GLConsole
[DXL]: https://github.com/samzapo/DynamixelDriver

###Usage

access the library by using the examples found in the [examples] directory:

 EXAMPLE       | LOCATION |  DESCRIPTION
-------------- | -------- | ---------------------------------------------------------
 control-moby  |          |  Moby controller plugin that communictes with Pacer
 control-gazebo|          |  Gazebo Model Plugin for an SDF model that communicates with Pacer
 driver.cpp    |          |  Stand-alone Pacer with no simulator, example for use with hardware
 models        |          |  Pre-implemented robot models and vars files (see [models/README])
 drive-robot   |          |  Simple library for driving the locomotion system of Pacer (called by the simulation plugins and driver.cpp).

 [models/README]: https://github.com/PositronicsLab/Pacer/tree/master/examples/models/README

## Exporting Data to MATLAB
 
 Vectors and Matrices are output in a syntax readable by Matlab for copy and pasting.  The examples in [scripts] demonstrate exporting output from pacer logs to MATLAB readable delimited files.

[scripts]: https://github.com/PositronicsLab/Pacer/tree/master/examples/test-scripts

### File Structure

 FILE/DIRECTORY  |  DESCRIPTION
---------------- | ---------------------------------------------------------
 CMakeLists.txt  |  CMake build file for Pacer
 example         |  Data files and examples for Pacer
 include         |  Header files for calling functions in Pacer
 src             |  Pacer source code
