#Pacer Project Repository
by Sam Zapolsky

This repository hosts code for [Pacer] an open source robot locomotion library for simulation and physical robots.

[Pacer]: https://github.com/PositronicsLab/Pacer

Originally developed on a 64 bit Ubuntu 14.04. 

###Usage

After running, from the root directory of the rlinks library,

```
mkdir build && cd build
```
```
cmake ..
sudo make install
```

you can access the library by using the examples found in the [test] directory.

[test]: https://github.com/gwsd2015/rlinks/tree/master/test

###Dependencies (* required) 

- [Ravelin] *
- [Moby] *
- Boost *
- Open Scene Graph
- [CVars] *
- [GLConsole] 
- [DXL]

[Ravelin]: https://github.com/PositronicsLab/Ravelin
[Moby]: https://github.com/PositronicsLab/Moby
[CVars]: https://github.com/arpg/GLConsole
[GLConsole]: https://github.com/arpg/GLConsole
[DXL]: https://github.com/samzapo/DynamixelDriver

### File Structure

 FILE/DIRECTORY  |  DESCRIPTION
---------------- | ---------------------------------------------------------
 CMakeLists.txt  |  CMake build file for Pacer
 example         |  Data files and examples for Pacer
 include         |  Header files for calling functions in Pacer
 src             |  Pacer source code
