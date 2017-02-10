# Field
The field MUST be run on an Ubuntu (debian) workstation or it will not have all the packages and setup necessary. The computer also needs to have a webcam to run. The webcam can either be integrated or a USB camera, it will behave similarly.

## Setup
The field requires several packages to run and must be placed in the proper directories. If you do not know how to operate linux please refer to my linux instructions in the readme of the top MQP folder.

### Required Packages

#### Dependencies
Please copy and paste the following command into your terminal. Make sure that all other packages have been updated prior to install by running:
~~~~
sudo apt-get update
sudo apt-get dist-upgrade
~~~~
If updates were installed please also run to restart your computer and avoid misconfigurations:
~~~~
sudo shutdown now -r
~~~~
The following command will install all required dependencies for the field code:
~~~~
sudo apt-get install subversion cmake libopencv-dev libeigen3-dev libv4l-dev
~~~~
If this does not work or a specific dependency fails you may have to look up their new install paths. The list of all dependencies are as follows:
- Subversion
- Cmake
- OpenCV (C/C++)
- Eigen (or Eigen3)
- libv4l (Ubuntu Video4Linux)

### Apriltags


### Building the File

## Runtime Options
TODO:
