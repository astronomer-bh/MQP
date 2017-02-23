# PluMQP
This project involved the design, analysis, and implementation of a gas-sensing mobile robot capable of autonomously mapping the gas concentration gradient being generated from a plume of carbon dioxide. Autonomy of the device is achieved through a micro-controller embedded in the robot in conjunction with several sensors for guidance, gas concentration and wind speed. This experiment intends to help further research in gas dynamics as well as gas sensing methodology, either in a terrestrial or aeronautic environments. This system seeks to differentiate itself from other experiments of the same type because the robot not only localizes the gas source, but also plots out the surrounding gas concentration to create a dynamic gas analysis model.

## Breakdown
### Robot (Python)
Buit with [enter library name here] which is used for the basic comunication protocal and basic commands for the iRobot Create. The robot is tasked with navigating around the experiment area and determining the concentration of gas (C02) throughout. The robot is also in charge of self-localization via a Kalman Filter which uses both encoders and an accelerometer to get measurements. The robot then sends it location as well as the readings from the C02 sensors via TCP/IP to the base station or base station as it is denoted in this document. The base station then sends back x and y velocities which are transformed into a direction (angle) and a velocity magnitude. The robot then travels in that direction until the base station sends new instructions.

### Base Station (Python)
The base station takes the location and concentration data from the robot and then directs the robot where to go in terms of x and y velocities based upon the data recieved. This is determined by a Fortran???? code held by Prof. Demetriou.

### Field (C/C++)
This code utilizes AprilTags and an overhead camera to determine the physical location of the robot for verification of the validity of the Kalman Filter in use by the Robot. This program is also a stepping stone to using a quadcopter for future localization of a multi-robot system.

### Copter (C/C++)
This code is the further implementation of the field code. Added a few classes for interrobot communication. Incomplete.

## Setup
All code in this repository was built to be run and debugged on Ubuntu (Debian Linux). I'm sure it works with other OSs as well but all guides and setup are geared towards Ubuntu, specifically 16.04.

### Ubuntu
I'm not going to go too much in depth about how to install Ubuntu, other guides do it better than me. Grab the LTS version of [Ubuntu 16.04](http://releases.ubuntu.com/16.04/) if you are going to install on a Desktop. If you are looking for RasPi installations grab [Ubuntu Mate](https://ubuntu-mate.org/download) and then [Win32DiskImager](https://sourceforge.net/projects/win32diskimager/) for instalation. Works well and much easier than me explaning how to install it on an SD card in Linux. Not worth either of our times.

### GitHub
By now you are probably wondering, hey when do I get to actually run this code? Its now and gosh darn is it simple. Two more commands to install git and then get the repo.
~~~~
sudo apt-get install git
cd && git clone https://github.com/rmwiesenberg/MQP/
~~~~
And thats it! the MQP should be in your home directory for your perusal. READMEs can be opened with a text editor or just use the browser because they look prettier here, pictures and stuff.
