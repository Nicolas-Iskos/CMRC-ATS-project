## Apogee Targeting System

This repository contains the source code for the Carnegie Mellon Rocket Command
Apogee Targeting System (ATS). The ATS is an active prediction and control algorithm
coupled to an electrically operated air-braking system used to achieve a
precise apogee (maximum altitude) with a High Power Rocket. It includes the
following contents.


#### ATS library
The ATS library allows for prediction and control on apogee given raw
sensor data. It can be used with any launch vehicle design and any
flight computer. It includes the following files.
  - ATS_int.h
  - LVD.h
  - LVD.cpp
  - pi_controller.h
  - pi_controller.cpp
  - sensor.h
  - sensor.cpp


#### Testing framework
The testing framework allows for testing of the ATS library by simulating
the flight of the launch vehicle from the beginning of ATS activation to apogee.
It includes the following files.
  - simulation_routine.cpp
  - run_simulation.m


#### Arduino application
The included Arduino application of the ATS library runs the ATS
on an Arduino micocontroller. It consists of a single file.
 - mainloop.ino
