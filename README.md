This repository contains the source code for the Carnegie Mellon Rocket Command
Apogee Targeting System (ATS), an active air-braking system used to achieve a
precise apogee with a High Power Rocket.

The ATS source code is designed to run on an Arduino microcontroller. The C++
files in the directory form an Arduino library, which can then be used in
the prediction and control routine contained in mainloop.ino.

A rough breakdown of the file structure is as follows:

sensor.cpp<br/>
Defines the sensor class, which is used to process raw sensor data
and turn it into useable state data

LVD.cpp:<br/>
Defines the LVD class, which is used to make predictions of apogee
based on vehicle state data and the current control

pi\_controller.cpp:<br/>
Defines the pi\_controller class, which is used to determine an
appropriate control on the air-brakes given instantaneous and
integral error values on the predicted apogee compared to the
target apogee

mainloop.ino:<br/>
An Arduino sketch that uses objects of the three classes described
above to perform real-time prediction and control on the apogee
of the launch vehicle
