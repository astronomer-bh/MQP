#ifndef Field_hpp

#include "Cam.hpp"
#include "Robot.hpp"

#include <algorithm>
#include <thread>

#define NUM_THREADS 3

class Field{
private:
  Cam m_cam;
  std::map<int,Robot> m_robots;

  void parseOptions(int argc, char* argv[]);
  void updateRobots();

  std::thread threads[NUM_THREADS];
public:
  Field(int argc, char* argv[]);
  void init();
  void loop();
};

#endif
