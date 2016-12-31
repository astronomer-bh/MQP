using namespace std;

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

const char* windowName = "FieldCam";

#ifndef PI
const double PI = 3.14159265358979323846;
#endif
const double TWOPI = 2.0*PI;

// utility function to provide current system time (used below in
// determining frame rate at which images are being processed)
double tic() {
  struct timeval t;
  gettimeofday(&t, NULL);
  return ((double)t.tv_sec + ((double)t.tv_usec)/1000000.);
}

/**
* Normalize angle to be within the interval [-pi,pi].
*/
inline double standardRad(double t) {
  if (t >= 0.) {
    t = fmod(t+PI, TWOPI) - PI;
  } else {
    t = fmod(t-PI, -TWOPI) + PI;
  }
  return t;
}

/**
* Convert rotation matrix to Euler angles
*/
void wRo_to_euler(const Eigen::Matrix3d& wRo, double& yaw, double& pitch, double& roll) {
  yaw = standardRad(atan2(wRo(1,0), wRo(0,0)));
  double c = cos(yaw);
  double s = sin(yaw);
  pitch = standardRad(atan2(-wRo(2,0), wRo(0,0)*c + wRo(1,0)*s));
  roll  = standardRad(atan2(wRo(0,2)*s - wRo(1,2)*c, -wRo(0,1)*s + wRo(1,1)*c));
}

class CamLoc {
  AprilTags::TagDetector* m_tagDetector;
  AprilTags::TagCodes m_tagCodes;

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

public:

  CamLoc() :
    // default settings, most can be modified through command line options (see below)
    m_tagDetector(NULL),
    m_tagCodes(AprilTags::tagCodes36h11),

    m_draw(true),
    m_timing(false),

    m_width(640),
    m_height(480),
    m_tagSize(0.166),
    m_fx(600),
    m_fy(600),
    m_px(m_width/2),
    m_py(m_height/2),

    m_exposure(-1),
    m_gain(-1),
    m_brightness(-1),

    hasDetections(false),

    m_deviceId(0)
  {}

    // changing the tag family
    void setTagCodes(string s) {
      if (s=="16h5") {
        m_tagCodes = AprilTags::tagCodes16h5;
      } else if (s=="25h7") {
        m_tagCodes = AprilTags::tagCodes25h7;
      } else if (s=="25h9") {
        m_tagCodes = AprilTags::tagCodes25h9;
      } else if (s=="36h9") {
        m_tagCodes = AprilTags::tagCodes36h9;
      } else if (s=="36h11") {
        m_tagCodes = AprilTags::tagCodes36h11;
      } else {
        cout << "Invalid tag family specified" << endl;
        exit(1);
      }
    }



    void setup() {
      m_tagDetector = new AprilTags::TagDetector(m_tagCodes);

      // open window for camera drawing (Optional)
      if (m_draw) {
        cv::namedWindow(windowName, 1);
      }
    }

    void setupVideo() {

      // This is from the AprilTags source code
      // Unfortunately don't know enought about exposure control
      // to describe the going ons of this section
  #ifdef EXPOSURE_CONTROL
      // manually setting camera exposure settings; OpenCV/v4l1 doesn't
      // support exposure control; so here we manually use v4l2 before
      // opening the device via OpenCV; confirmed to work with Logitech
      // C270; try exposure=20, gain=100, brightness=150

      string video_str = "/dev/video0";
      video_str[10] = '0' + m_deviceId;
      int device = v4l2_open(video_str.c_str(), O_RDWR | O_NONBLOCK);

      if (m_exposure >= 0) {
        // not sure why, but v4l2_set_control() does not work for
        // V4L2_CID_EXPOSURE_AUTO...
        struct v4l2_control c;
        c.id = V4L2_CID_EXPOSURE_AUTO;
        c.value = 1; // 1=manual, 3=auto; V4L2_EXPOSURE_AUTO fails...
        if (v4l2_ioctl(device, VIDIOC_S_CTRL, &c) != 0) {
          cout << "Failed to set... " << strerror(errno) << endl;
        }
        cout << "exposure: " << m_exposure << endl;
        v4l2_set_control(device, V4L2_CID_EXPOSURE_ABSOLUTE, m_exposure*6);
      }
      if (m_gain >= 0) {
        cout << "gain: " << m_gain << endl;
        v4l2_set_control(device, V4L2_CID_GAIN, m_gain*256);
      }
      if (m_brightness >= 0) {
        cout << "brightness: " << m_brightness << endl;
        v4l2_set_control(device, V4L2_CID_BRIGHTNESS, m_brightness*256);
      }
      v4l2_close(device);
  #endif

      // find and open a USB camera (built in laptop camera, web cam etc)
      m_cap = cv::VideoCapture(m_deviceId);
          if(!m_cap.isOpened()) {
        cerr << "ERROR: Can't find video device " << m_deviceId << "\n";
        exit(1);
      }
      m_cap.set(CV_CAP_PROP_FRAME_WIDTH, m_width);
      m_cap.set(CV_CAP_PROP_FRAME_HEIGHT, m_height);
      cout << "Camera successfully opened (ignore error messages above...)" << endl;
      cout << "Actual resolution: "
           << m_cap.get(CV_CAP_PROP_FRAME_WIDTH) << "x"
           << m_cap.get(CV_CAP_PROP_FRAME_HEIGHT) << endl;

    }

    // the printing of relative location
    void print_detection(AprilTags::TagDetection& detection) const {
      cout << "  Id: " << detection.id
           << " (Hamming: " << detection.hammingDistance << ")";

      // recovering the relative pose of a tag:

      // NOTE: for this to be accurate, it is necessary to use the
      // actual camera parameters here as well as the actual tag size
      // (m_fx, m_fy, m_px, m_py, m_tagSize)

      Eigen::Vector3d translation;
      Eigen::Matrix3d rotation;
      detection.getRelativeTranslationRotation(m_tagSize, m_fx, m_fy, m_px, m_py,
                                               translation, rotation);

      Eigen::Matrix3d F;
      F <<
        1, 0,  0,
        0,  -1,  0,
        0,  0,  1;
      Eigen::Matrix3d fixed_rot = F*rotation;
      double yaw, pitch, roll;
      wRo_to_euler(fixed_rot, yaw, pitch, roll);

      cout << "  distance=" << translation.norm()
           << "m, x=" << translation(0)
           << ", y=" << translation(1)
           << ", z=" << translation(2)
           << ", yaw=" << yaw
           << ", pitch=" << pitch
           << ", roll=" << roll
           << endl;

      // Also note that for SLAM/multi-view application it is better to
      // use reprojection error of corner points, because the noise in
      // this relative pose is very non-Gaussian; see iSAM source code
      // for suitable factors.
    }

    void processImage(cv::Mat& image, cv::Mat& image_gray) {
      // alternative way is to grab, then retrieve; allows for
      // multiple grab when processing below frame rate - v4l keeps a
      // number of frames buffered, which can lead to significant lag
      //      m_cap.grab();
      //      m_cap.retrieve(image);

      // detect April tags (requires a gray scale image)
      cv::cvtColor(image, image_gray, CV_BGR2GRAY);

      double t0;
      if (m_timing) {
        t0 = tic();
      }

      vector<AprilTags::TagDetection> detections = m_tagDetector->extractTags(image_gray);

      if (m_timing) {
        double dt = tic()-t0;
        cout << "Extracting tags took " << dt << " seconds." << endl;
      }

      // print out each detection
      cout << detections.size() << " tags detected:" << endl;
      for (unsigned int i=0; i<detections.size(); i++) {
        print_detection(detections[i]);
      }

      // show the current image including any detections
      if (m_draw) {
        for (unsigned int i=0; i<detections.size(); i++) {
          // also highlight in the image
          detections[i].draw(image);
        }
        imshow(windowName, image); // OpenCV call
      }


    }

    // Load and process a single image
    void loadImages() {
      cv::Mat image;
      cv::Mat image_gray;

      for (list<string>::iterator it=m_imgNames.begin(); it!=m_imgNames.end(); it++) {
        image = cv::imread(*it); // load image with opencv
        processImage(image, image_gray);
        while (cv::waitKey(100) == -1) {}
      }
    }

    // Video or image processing?
    bool isVideo() {
      return m_imgNames.empty();
    }

    // The processing loop where images are retrieved, tags detected,
    // and information about detections generated
    void loop() {

      cv::Mat image;
      cv::Mat image_gray;

      int frame = 0;
      double last_t = tic();
      while (true) {

        // capture frame
        m_cap >> image;

        processImage(image, image_gray);

        // print out the frame rate at which image frames are being processed
        frame++;
        if (frame % 10 == 0) {
          double t = tic();
          cout << "  " << 10./(t-last_t) << " fps" << endl;
          last_t = t;
        }

        // exit if any key is pressed
        if (cv::waitKey(1) >= 0) break;
      }
    }
  }; // CamLoc
