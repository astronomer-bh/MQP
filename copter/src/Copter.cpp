#include "Copter.hpp"

static volatile int keepRunning = 1;

void intHandler(int dummy) {
  keepRunning = 0;
}

// here is were everything begins
// creates field with optional args
// loops through field
int main(int argc, char* argv[]) {
  // signal(SIGINT, intHandler);
  Field field(argc, argv, keepRunning);
  field.loop();
  field.terminate();

  return 0;
}
