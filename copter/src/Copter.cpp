#include "Copter.hpp"

  Copter::Copter(int argc, char* argv[])
  : m_field(argc, argv), m_homeID(0), m_pose()
  {

  }

  // Finds Home Robot Location
  Pose Copter::posHome(){
    return m_field.getRobotCurPose(m_homeID);
  }

  // Goes Home (Location ONLY)
  void Copter::goHome(){

  }

  // Flies the Copter
  void Copter::loop(){
    m_field.loop();
  }

  // kills any necessary processes before closing
  void Copter::terminate(){
    m_field.terminate();
  }


// Main Function Bits

bool keepRunning = 1;

void intHandler(int dummy) {
  keepRunning = false;
}

// here is were everything happens
// creates field with optional args
// loops through field
int main(int argc, char* argv[]) {
  signal(SIGINT, intHandler);
  Copter copter(argc, argv);
  while(keepRunning){
    copter.loop();
  }
  copter.terminate();

  return 0;
}
