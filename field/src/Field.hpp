#ifndef Field_hpp

#include "Cam.hpp"
#include "Robot.hpp"
#include "mqpif.h"

#include <algorithm>

class Field{
private:
  Cam m_cam;
  std::map<int,Robot> m_robots;

  void parseOptions(int argc, char* argv[]);
  void updateRobots(MQPIf mqpif);
public:
  void startSocket(MQPIf mqpif);
  Field(int argc, char* argv[]);
  void loop(MQPIf mqpif);
};

#endif
