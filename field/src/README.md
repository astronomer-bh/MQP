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
sudo apt-get install subversion cmake libopencv-dev libeigen3-dev libv4l-dev c++11
~~~~
If this does not work or a specific dependency fails you may have to look up their new install paths. The list of all dependencies are as follows:
- Subversion
- Cmake
- OpenCV (C/C++)
- Eigen (or Eigen3)
- libv4l (Ubuntu Video4Linux)

### Apriltags
To install the AprilTags library move to the lib directory and then pull it from MIT's C++ repository:
~~~~
cd /usr/include
sudo svn co https://svn.csail.mit.edu/apriltags
~~~~
for more information on this repository please visit [here](http://people.csail.mit.edu/kaess/apriltags/)

The way this library is installed allows for it to be used across the field and copter files. Unfortunately, this is not how it was originally designed. To remedy this just rename all instances of #include AprilTags/{filename.h} in the AprilTags library to just #include {filename.h}.  (Instances are found in: Edge.h, TagDetector.h, and TagFamily.h)

### Building the File

## Runtime Options
TODO:
