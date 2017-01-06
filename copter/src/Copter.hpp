#ifndef Copter_hpp

#include <signal.h>
#include "Field.hpp"

class Copter {
private:
  Field m_field;
  int m_homeID;
  Pose m_pose;

  Pose posHome();
  void goHome();

  void matchHome();
  void matchLoc();
  void matchYaw();
  void matchPtc();
  void matchRol();
  void matchOrt();
public:
  Copter(int argc, char* argv[]);
  void loop();
  void terminate();

  void getPose();
};

#endif
