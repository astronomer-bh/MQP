# Robot
The robot is run on a RaspberryPi with an iRobot Create, IMU, and Teensy attached to retrieve values from the gas sensors. The KTeam folder contains the past iteration of the project's code when the robot was a small unstable Khepera IV from KTeam. This folder will not be discussed but is available for reference. This code is all writted in Python for ease of use of Aero students.

## Setup
The field requires several packages to run and must be placed in the proper directories. If you do not know how to operate linux please refer to my linux instructions in the readme of the top MQP folder. Running this code on something that is not a RaspberryPi will result in failures, for instructions on setting up a RaspberryPi for this project please refer to the instructions in the readme of the top MQP folder.

### Required Packages
#### Dependencies
Please copy and paste the following command into your RaspberryPi's terminal. Make sure that all other packages have been updated prior to install by running:
~~~~
sudo apt-get update
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
sudo apt-get install -y build-essential python-dev python-smbus python-pip python3-pip upstart
sudo -H python3 -m pip install --upgrade pip 
sudo -H python3 -m pip install pyserial sympy Adafruit-GPIO
~~~~
If this does not work or a specific dependency fails you may have to look up thier new installation paths. The list of dependencies are as follows:
- Python-Dev (for Python 3)
- Python-Pip (Python package installer)
- PySerial
- SymPy
- Adafruit-GPIO

## Usage
In this directory there are two essential python codes, Create1 and Create.
