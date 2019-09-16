# Navilation
is an experimental project for applying and testing a simplified vehicle control. The main purpose is purely educational. Its features currently comprise 
* a graphical visualisation of a 2D map and a vehicle
* position control either via manual input or  model predictive controller (MPC)
* path specification: defining a spline that is used as a trajectory

## Structure
The project consists of three subprojects. __gui__ provides a graphical interface and a control panel. Furthermore
it contains the main class function. Graphical output is realized by manipulating directly the image matrix using _openCV_. For a better UI experience, a _QT_ based control panel is implemented in order to use the main features described above.

__model__ provides a simplified kinematic and dynamic model of a vehicle. Besides, it's also where the MPC for each
model and its integration into feedback control system is defined.

All methods that may be useful in other projects are for now collected in __Util__.

## Build
This project has several dependencies on other libraries. These dependencies can be resolved using
[Conan](https://www.conan.io). However, as not all libraries used are found in the conan repository, it is neccessary to
download and install them manually into '''./lib/grampc'''.
_this section needs more information. Build process needs a refinement_

## Requirements
This project uses C++ v17. Thus, a compatible compiler is required.
