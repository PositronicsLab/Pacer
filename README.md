###Pacer Project Repository
by Sam Zapolsky @ the [Positronics Lab]

[Positronics Lab]: http://robotics.gwu.edu/positronics/

This repository hosts code for [Pacer] an open source robot locomotion library for simulation and physical robots.

[Pacer]: https://github.com/PositronicsLab/Pacer

##Building

In the project base directory:
    $ mkdir build
    $ cd build
    $ ccmake .. # configure cmake project
    $ make all

Make sure that all Plugins and Interfaces build successfully

##Running Pacer (Example)

In the project base directory (bash terminal):
    $ source setup.sh
    $ cd $PACER_HOME/example/models/links

in Gazebo:
    $ gazebo links.world

in Moby:
    $ moby-driver -p=$PACER_HOME/build/example/interfaces/libPacerMobyPlugin.so model.xml

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

access the library by using the examples found in the examples directory:

 EXAMPLE       |  DESCRIPTION
--------------|---------------------------------------------------------
 inerfaces     |  Moby & Gazebo controller plugins communicte with Pacer
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
