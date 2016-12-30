#ifndef Cam_hpp
#include <iostream>
#include <cstring>
#include <vector>
#include <list>
#include <sys/time.h>
#include <cmath>

#ifndef __APPLE__
#define EXPOSURE_CONTROL // only works in Linux
#endif

#ifdef EXPOSURE_CONTROL
#include <libv4l2.h>
#include <linux/videodev2.h>
#include <fcntl.h>
#include <errno.h>
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

extern const char* windowName;

#ifndef PI
const double PI = 3.14159265358979323846;
#endif
const double TWOPI = 2.0*PI;
#endif

class Cam {
private:
  AprilTags::TagDetector* m_tagDetector;
  AprilTags::TagCodes m_tagCodes;
  vector<AprilTags::TagDetection> m_detections; // AprilTag Detections

  cv::Mat m_image;      // cur image
  cv::Mat m_image_gray; // cur grayscale image

  bool m_draw; // draw image and April tag detections?
  bool m_timing; // print timing information for each tag extraction call

  int m_width; // image size in pixels
  int m_height;
  double m_tagSize; // April tag side length in meters of square black frame
  double m_fx; // camera focal length in pixels
  double m_fy;
  double m_px; // camera principal point
  double m_py;

  int m_deviceId; // camera id (in case of multiple cameras)

  list<string> m_imgNames;

  cv::VideoCapture m_cap;

  int m_exposure;
  int m_gain;
  int m_brightness;

  bool hasDetections;

  void setTagCodes(string S);
  void setupVideo();
  void processImage();
  void parseOptions(int argc, char* argv[]);
public:
  double tagSize();
  double fx();
  double fy();
  double px();
  double py();

  Cam(int argc, char* argv[]);
  void loop();
  vector<AprilTags::TagDetection> getTags();
};
