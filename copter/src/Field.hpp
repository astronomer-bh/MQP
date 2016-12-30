#ifndef Field_hpp

#include "Cam.hpp"
#include "Robot.hpp"

#include <algorithm>
#include <thread>
#include <signal.h>

#define NUM_THREADS 3

using namespace std;

class Field{
private:
  Cam m_cam;
  map<int,Robot> m_robots;

  int keepRunning;

  void parseOptions(int argc, char* argv[]);
  void updateRobots();

  thread threads[NUM_THREADS];
public:
  Field(int argc, char* argv[], int keepRunning);
  void loop(int i);
  void runThreads();
  void terminate();
};

#endif
