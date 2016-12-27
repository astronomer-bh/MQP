#include "Field.hpp"

//inialize field
Field::Field(int argc, char* argv[])
: m_cam(argc, argv)
{
  parseOptions(argc, argv);

  for (int i = 0; i < NUM_THREADS; i++){
    threads[i] = std::thread(&Field::loop, this);
  }

  for (int i = 0; i < NUM_THREADS; i++){
    threads[i].join();
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
void Field::updateRobots(){
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
      m_robots.at(tagID).printID();
      m_robots.at(tagID).printPose();
    } catch (std::out_of_range&) {
      std::cout << "Robot " << tagID << " was not correctly added to list" << endl;
    }
  }
}

// updates camera and apriltag locations
// then updates robot positions based on the new info
void Field::loop(){
  m_cam.loop();
  updateRobots();
}

// here is were everything begins
// creates field with optional args
// loops through field
int main(int argc, char* argv[]) {
  Field field(argc, argv);

  while(true){
    // the actual processing loop where tags are detected and visualized
    field.loop();
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
