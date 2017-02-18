# Robot
The robot is run on a RaspberryPi with an iRobot Create, IMU, and Teensy attached to retrieve values from the gas sensors. The KTeam folder contains the past iteration of the project's code when the robot was a small unstable Khepera IV from KTeam. This folder will not be discussed but is available for reference. This code is all writted in Python for ease of use of Aero students.

## Setup
The field requires several packages to run and must be placed in the proper directories. If you do not know how to operate linux please refer to my linux instructions in the readme of the top MQP folder. Running this code on something that is not a RaspberryPi will result in failures, for instructions on setting up a RaspberryPi for this project please refer to the instructions in the readme of the top MQP folder.

### Required Packages
#### Dependencies
Please copy and paste the following command into your RaspberryPi's terminal. Make sure that all other packages have been updated prior to install by running:
~~~~
sudo apt-get update && sudo apt-get install upstart
sudo apt-get dist-upgrade
~~~~
You must also add your user to dialout in order to use and communicate with all of the external components (iRobot, IMU, Teensy). Then reload the terminal you are working in
~~~~
sudo adduser [yourusername] dialout
~~~~
If updates were installed and user was not added to dialout previously, please also run to restart your computer and avoid misconfigurations:
~~~~
sudo shutdown now -r
~~~~
The following command will install python and its dependencies and sub packages:
~~~~
sudo apt-get install -y build-essential python-dev python-smbus python-pip python3-pip
sudo -H python3 -m pip install --upgrade pip 
sudo -H python3 -m pip install pyserial sympy Adafruit-GPIO numpy
~~~~
If this does not work or a specific dependency fails you may have to look up thier new installation paths. The list of dependencies are as follows:
- Python-Dev (for Python 3)
- Python-Pip (Python package installer)
- PySerial
- SymPy
- NumPy
- Adafruit-GPIO

## Usage
In this directory there are two essential python codes, Create1 and Create2. Create1.py is the code used for the original iRobot Create (the white one) and as such may not work entirely with the updates made for the Create2. Its code base is from the pycreate module but may not work as is with changed made to the project. Create2.py is the current setup for the project which uses an iRobot Create2 in conjunction with the framework of breezycreate2, a separate github repository which has been modified to fix some errors and add a few more functions to the API.

Before running Create2.py, the base station's code should already be running, otherwise the robot will spit back a connection error associated with not seeing the base. See the base directory for those instructions. To run the code on the Create2 simply run the following line of code in this directory:
~~~~
python3 Create2.py 0
~~~~
The terminating 0 signifies the robot's ID number and can be changed if running multiple robots simultaneously. This ID should be set to match the ID of the AprilTag on the robot as to not confuse the future potential quadcopter. This code will report back the current desired velocities and send to the base station the robot's ID, current position, orientation, and sensor data.

### Runtime Commands
-id: Sets the robot's ID for communicating with the base station properly

## Walkthrough

## Libraries
### Adafruit_BNO055
![alt text](https://github.com/rmwiesenberg/MQP/blob/master/robot/libs/Adafruit_BNO055/sensors_raspberry_pi_bb.png "BNO055 Hookup")
### Teensy
### breezycreate2
### pycreate
### TCP (custom_libs)
### EKF (custom_libs)
