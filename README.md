# resolution
Conflict Resolution classes developed by David Alejo during the course of his phD Thesis
"Planning and Coordination on Systems of multiple UAVs"

This repository contains all libraries which are necessary to run the tests performed during 
my Thesis in chapters 3 and 4.

The contents are divided into several libraries, which are listed here in the same order that need to be installed 
in a computer. The installation steps for each library are the usual steps in CMake based developments in Linux:

1- Create a build directory inside the library and execute cmake inside this folder:

  > mkdir build
  > cd build
  > cmake ..
  
2- Compile the source code

  > make
  
3- Install

  > sudo make install
  
This procedure has to be done for each library in the following order:

1- functions --> general purpose functions

2- sparser--> parsing library initially developed by Pablo Soriano

3- simulator --> basic simulation library for a number of UAVs

4- graph --> basic graph library

5- particle_swarm --> basic PSO optimization library

6- resolution --> conflict detection and resolution library based on Evolutionary Optimization

7- UAVFlightPlan --> Function used to automatically generate flight plans for UAVs in some Autopilots format


For detailed instructions which could be necessary in order to install each library, please refer to the folders
of these libraries.

