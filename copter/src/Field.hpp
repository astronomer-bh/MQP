#ifndef Field_hpp

#include "Cam.hpp"
#include "Robot.hpp"

#include <algorithm>

using namespace std;

class Field{
private:
  Cam m_cam;
  map<int,Robot> m_robots;

  int keepRunning;

  void parseOptions(int argc, char* argv[]);
  void updateRobots();
public:
  Field(int argc, char* argv[], int keepRunning);
  void loop();
  void terminate();
};

#endif
