## Apogee Targeting System

This repository contains the source code for the Carnegie Mellon Rocket Command
Apogee Targeting System (ATS), an active air-braking system used to achieve a
precise apogee (maximum altitude) with a High Power Rocket.

### Contents

#### ATS library
The ATS library allows for prediction and control on apogee given raw
sensor data. It includes the following files. It can be used with any launch
vehicle design and any flight computer.
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
  - simulation_routine.cpp
  - run_simulation.m


#### Arduino application
The included Arduino application of the ATS library runs the ATS
on an Arduino micocontroller.
 - mainloop.ino
