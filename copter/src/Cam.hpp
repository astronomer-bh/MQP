#ifndef Cam_hpp
#include <iostream>
#include <cstring>
#include <vector>
#include <list>
#include <sys/time.h>
#include <cmath>
#include <thread>
#include <mutex>

#ifndef __APPLE__
#define EXPOSURE_CONTROL // only works in Linux
#endif

#ifdef __arm__
#include "raspicam"
#endif

// OpenCV library for easy access to USB camera and drawing of images
// on screen
#include "opencv2/opencv.hpp"


// April tags detector and various families that can be selected by command line option
// located in /usr/include
#include "apriltags/AprilTags/TagDetector.h"
#include "apriltags/AprilTags/Tag16h5.h"
#include "apriltags/AprilTags/Tag25h7.h"
#include "apriltags/AprilTags/Tag25h9.h"
#include "apriltags/AprilTags/Tag36h9.h"
#include "apriltags/AprilTags/Tag36h11.h"

// Needed for getopt / command line options processing
#include <unistd.h>
extern int optind;
extern char *optarg;


#define NUM_THREADS 1
using namespace std;


#ifndef PI
const double PI = 3.14159265358979323846;
#endif
const double TWOPI = 2.0*PI;
#endif

class Cam{
private:
  bool keepRunning;

  AprilTags::TagDetector* m_tagDetector;
  AprilTags::TagCodes m_tagCodes;
  vector<AprilTags::TagDetection> m_detections; // AprilTag Detections

  cv::Mat m_image;      // cur image
  mutex m_image_mutex;
  bool m_image_proc;

  cv::Mat m_image_gray; // cur grayscale image
  mutex m_image_gray_mutex;

  bool m_draw; // draw image and April tag detections?
  bool m_timing; // print timing information for each tag extraction call
  bool m_picam; //picamera is being used;

  int m_width; // image size in pixels
  int m_height;
  double m_tagSize; // April tag side length in meters of square black frame
  double m_fx; // camera focal length in pixels
  double m_fy;
  double m_px; // camera principal point
  double m_py;

  list<string> m_imgNames;

  cv::VideoCapture m_cap;

  thread threads[NUM_THREADS];

  bool hasDetections;

  int m_deviceId; // camera id (in case of multiple cameras)
  const char* m_windowName;

  void setTagCodes(string S);
  void setupVideo();
  void createThreads();
  void startThreads();
  void processImage();
  void pullImage();
  void drawImage();
  void parseOptions(int argc, char* argv[]);
public:
  Cam(int argc, char* argv[]);
  void loop();
  void terminate();

  vector<AprilTags::TagDetection> getTags();
  double tagSize();
  double fx();
  double fy();
  double px();
  double py();
};

double tic();
