#Pacer Project Repository
by Sam Zapolsky @ the [Positronics Lab]

[Positronics Lab]: http://robotics.gwu.edu/positronics/

This repository hosts code for [Pacer] an open source robot locomotion library for simulated and physical legged robots.

[Pacer]: https://github.com/PositronicsLab/Pacer

**For more information, check out the [wiki].**

[wiki]: https://github.com/PositronicsLab/Pacer/wiki

##Dependencies 

####Required 

- [Ravelin] 
- [Moby] 
- Boost 

####Optional 

- OpenSceneGraph
- [DXL]
- SDL (permits joystick control)

[Ravelin]: https://github.com/PositronicsLab/Ravelin
[Moby]: https://github.com/PositronicsLab/Moby
[DXL]: https://github.com/samzapo/DynamixelDriver

## Quick start to using Pacer
### Building
```
mkdir release; cd release;
```
If [Ravelin](http://positronicslab.github.io/Ravelin/) and [Moby](http://positronicslab.github.io/Moby/) are *installed* on your system:
```
cmake ..
```
otherwise
```
cmake -DMOBY_INCLUDE_DIR="/path/to/moby/include" -DMOBY_LIBRARY="/path/to/moby/lib/libMoby.so" -DMOBY_LIBRBRY_DIR="/path/to/moby/lib" -DMOBY_LIBRBRY_DIRS="/path/to/moby/lib" -DRAVELIN_INCLUDE_DIR="/path/to/ravelin/include" -DRAVELIN_LIBRARY="/path/to/ravelin/lib/libRavelin.so" -DRAVELIN_LIBRBRY_DIR="/path/to/ravelin/lib" -DRAVELIN_LIBRBRY_DIRS="/path/to/ravelin/lib" -DUSE_OSG_DISPLAY=ON -DCMAKE_BUILD_TYPE=RELEASE ..;
```
and then build:
```
make -j <cores> all
```
### Setup

in the project base directory, run:

`source setup.sh`

This will point `Gazebo` and `Pacer` to the models and build directories.  `setup.sh` sets environment vars:
*     `PACER_MODEL_PATH` points to model directories where `Pacer` will import the robot kinematic and inertial information (`Pacer` does NOT get model information from the simulator)
  *     `PACER_COMPONENT_PATH` points to the directory where the perception, planning, and control (Component) plugins have been built.
  *     `PACER_SIMULATOR_PATH` points to the directory where the simulator (interface) plugins have been built.
  *     `GAZEBO_MODEL_PATH` is a `:` delimited list of paths to where the gazebo models `sdf` and their config files reside.
  *     `GAZEBO_PLUGIN_PATH` is a `:` delimited list of paths to where the gazebo model plugins have been built, including our gazebo plugin: `libPacerGazeboPlugin.so` has been built

### R. Links (Quadruped) Demo

  remember to run:
  ```
  source /build/dir/setup.sh
  ```
  then navigate to the demo working directory for a trotting gait:
  ```
  cd ${PACER_HOME}/Example/Demo/Trot
  ```
  Write the environment paths into the demo XML files: (*NOTE:* this command writes `filename.xml.in` to `filename.xml`)
  ```
  ${PACER_SCRIPT_PATH}/setup-tests.sh *.in
  ```
#### starting in Moby:

  then start *Moby* while pointing at the *Pacer* Moby simulator plugin:
  ```
  moby-driver -r -s=0.001 -p=${PACER_SIMULATOR_PATH}/libPacerMobyPlugin.so model.xml
  ```
  This will start the robot `R. Links` 'trotting' on a plane in the *Moby* simulator with visualization turned on.

#### starting in Gazebo:

  then start *Gazebo* while pointing at the links world file:
  ```
  gazebo ${PACER_MODEL_PATH}/links/model.world
  ```
  Gazebo will see that the model file `${PACER_MODEL_PATH}/links/model.sdf` uses to controller `libPacerGazeboPlugin.so`, it will find this controller in directory `${GAZEBO_PLUGIN_PATH}/` which was exported by running `setup.sh`.

  *  NOTE: This could be built as `libPacerGazeboPlugin.dylib` on an OSX system be sure to change the name of the plugin in `${PACER_MODEL_PATH}/links/model.sdf` if Gazebo can't find any model plugin.

  This will start the robot `R. Links` 'trotting' on a plane in the *Gazebo* simulator.

  *NOTE:* `Pacer` will read the file `vars.xml` in the working directory on startup.  This file sets parameters in all controller plugins, as well as telling `Pacer` which plugins to load. 

##Usage

Access the library by using the examples found in the examples directory:

 EXAMPLE       |  DESCRIPTION
--------------|---------------------------------------------------------
 simulator plugins     |  Moby & Gazebo controller plugins communicte with Pacer
 models        |  Pre-implemented robot models and vars files (see models/README.md)
 component plugins   |    Discrete control modules for the robot.  Add capabilites to Pacer by adding plugins here.

#### Exporting Data to MATLAB

 Logs are either output to terminal or written to the file: "out.log" in the run directory.  Running Pacer will overwrite the current instance of "./out.log"

 Vectors and Matrices output is readable in Matlab.  The examples in [scripts] demonstrate exporting output from Pacer's logs to MATLAB readable space delimited files.

[scripts]: https://github.com/PositronicsLab/Pacer/tree/master/Example/Script

#### File Structure

 FILE/DIRECTORY  |  DESCRIPTION
---------------- | ---------------------------------------------------------
 CMakeLists.txt  |  CMake build file for Pacer
 Example         |  Use cases and helpful files for using Pacer
 src             |  Pacer core source code and, Header files for calling functions in Pacer (does not do much without *components*)
 Plugin          |  Everything needed to actually use Pacer: *Components* plug into Pacer and serve as perception, planning, control, or data collection modules;  Plugins built in *Simulator* plug Pacer into a simulator, currently [Gazebo] and [Moby] are supported.  

 [Moby]: https://github.com/PositronicsLab/Moby
 [Gazebo]: https://gazebosim.org
