#include "Field.hpp"
#include "mqpif.h"

#include <stdexcept>
// Includes below this line added to support socket -BH
#include <iostream>
#include <getopt.h>
#include <stdlib.h>
#include <errno.h>
#include <sstream>
#include <cstring>

//inialize field
Field::Field(int argc, char* argv[])
: m_cam(argc, argv)
{
  parseOptions(argc, argv);  // Note that camera-related options are parsed in Cam.cpp
}

// Opens the socket to send info to the Python code
void Field::startSocket(MQPIf mqpif){
  std::cout << "starting socket" << std::endl;

  extern char *optarg;  // this is the argument pointer thingie
  char host[256];
  int port = 9999;
  std::strcpy(host, "localhost");  //strcpy == string copy -BH
  try{
    mqpif.connect(host, port);
  }
  catch (const char *n_err) {
    std::cout << *n_err << std::endl;  // This might not work (supposed to print the error message taken from mqpif.cpp) -BH
  }
  catch (...) {
    std::cout << "internal error" << std::endl;
  }
}


// parse command line options to change default behavior
// TODO: FANCY CAMERA THINGS
// eg. change camera size, set max robots?
void Field::parseOptions(int argc, char* argv[]) {

}

// checks cam for apriltag locations
// updates robots with id corresponding to current apriltag IDs
// creates robots if robot list does not have robot with ID needed
void Field::updateRobots(MQPIf mqpif){
  for (unsigned int i = 0; i < m_cam.getTags().size(); i++){
    AprilTags::TagDetection curTag = m_cam.getTags()[i];
    int tagID = curTag.id;

    Eigen::Vector3d translation;
    Eigen::Matrix3d rotation;

    // Works off of Relative Camera Coordinates
    curTag.getRelativeTranslationRotation(m_cam.tagSize(),
    m_cam.fx(), m_cam.fy(), m_cam.px(), m_cam.py(),
    translation, rotation);

    try{
      m_robots.at(tagID).setPose(translation(2), -translation(1), translation(0),
                                rotation(0), rotation(1), rotation(2));
    } catch (std::out_of_range&) {
      Robot curRobot(tagID, translation(2), -translation(1), translation(0),
                            rotation(0), rotation(1), rotation(2));
      m_robots.insert(std::pair<int,Robot>(tagID, curRobot));
    }

    try {
      std::stringstream ss;
      ss << m_robots.at(tagID).getPose().x() << ',';
      ss << m_robots.at(tagID).getPose().y() << ',';
      ss << m_robots.at(tagID).getPose().z() << ',';
      ss << m_robots.at(tagID).getPose().yaw() << ',';
      ss << m_robots.at(tagID).getPose().ptc() << ',';
      ss << m_robots.at(tagID).getPose().rol();
      std::string str = ss.str();

      mqpif.sendOne(str.c_str(), str.length());
      //m_robots.at(tagID).printID();  // These printed the data to the console -BH
      //m_robots.at(tagID).printPose();
    } catch (std::out_of_range&) {
      std::cout << "Robot " << tagID << " was not correctly added to list" << endl;
    } catch (...) {
      std::cout << "Error sending Robot" << tagID << std::endl;
    }
  }
}

// updates camera and apriltag locations
// then updates robot positions based on the new info
void Field::loop(MQPIf mqpif){
  m_cam.loop();
  updateRobots(mqpif);
}

// here is were everything begins
// creates field with optional args
// loops through field
int main(int argc, char* argv[]) {
  static MQPIf mqpif;
  Field field(argc, argv);
  field.startSocket(mqpif);  // TODO: is there a better way to handle this mqpif object? -BH
  while(true){
    // the actual processing loop where tags are detected and visualized
    field.loop(mqpif);
    // BH-TODO: Add the communication code here??
  }

  return 0;
}

/**
* Normalize angle to be within the interval [-pi,pi].
*/
inline double standardRad(double t) {
  if (t >= 0.) {
    t = fmod(t+PI, TWOPI) - PI;
  } else {
    t = fmod(t-PI, -TWOPI) + PI;
  }
  return t;
}

/**
* Convert rotation matrix to Euler angles
*/
void wRo_to_euler(const Eigen::Matrix3d& wRo, double& yaw, double& pitch, double& roll) {
  yaw = standardRad(atan2(wRo(1,0), wRo(0,0)));
  double c = cos(yaw);
  double s = sin(yaw);
  pitch = standardRad(atan2(-wRo(2,0), wRo(0,0)*c + wRo(1,0)*s));
  roll  = standardRad(atan2(wRo(0,2)*s - wRo(1,2)*c, -wRo(0,1)*s + wRo(1,1)*c));
}
