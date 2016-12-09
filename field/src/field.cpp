#include "CamLoc.cpp"

// parse command line options to change default behavior
// TODO: FANCY CAMERA THINGS
void parseOptions(int argc, char* argv[]) {
}

// here is were everything begins
int main(int argc, char* argv[]) {
  CamLoc cam;

  // process command line options
  parseOptions(argc, argv);

  cam.setup();

  if (cam.isVideo()) {
    cout << "Processing video" << endl;

    // setup image source, window for drawing, serial port...
    cam.setupVideo();

    // the actual processing loop where tags are detected and visualized
    cam.loop();

  } else {
    cout << "Processing image" << endl;

    // process single image
    cam.loadImages();

  }

  return 0;
}
