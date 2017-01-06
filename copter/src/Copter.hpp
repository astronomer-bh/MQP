#ifndef Copter_hpp

#include <signal.h>
#include "Field.hpp"

class Copter : private Robot{
private:
  Field m_field;

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
