using namespace std;

#include "Cam.hpp"

const char* windowName = "Field";

// utility function to provide current system time (used below in
// determining frame rate at which images are being processed)
double tic() {
  struct timeval t;
  gettimeofday(&t, NULL);
  return ((double)t.tv_sec + ((double)t.tv_usec)/1000000.);
}

Cam::Cam(int argc, char* argv[]) :
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
{
  parseOptions(argc, argv);
  m_tagDetector = new AprilTags::TagDetector(m_tagCodes);
  setupVideo();

  // open window for camera drawing (Optional)
  if (m_draw) {
    cv::namedWindow(windowName, 1);
  }
}

// set fancy camera things
void Cam::parseOptions(int argc, char* argv[]){

}

// changing the tag family
void Cam::setTagCodes(string s) {
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


void Cam::setupVideo() {
  // This is from the AprilTags source code
  // Unfortunately don't know enought about exposure control
  // to describe the going ons of this section
  // #ifdef EXPOSURE_CONTROL
  // // manually setting camera exposure settings; OpenCV/v4l1 doesn't
  // // support exposure control; so here we manually use v4l2 before
  // // opening the device via OpenCV; confirmed to work with Logitech
  // // C270; try exposure=20, gain=100, brightness=150
  //
  // string video_str = "/dev/video0";
  // video_str[10] = '0' + m_deviceId;
  // int device = v4l2_open(video_str.c_str(), O_RDWR | O_NONBLOCK);
  //
  // if (m_exposure >= 0) {
  //   // not sure why, but v4l2_set_control() does not work for
  //   // V4L2_CID_EXPOSURE_AUTO...
  //   struct v4l2_control c;
  //   c.id = V4L2_CID_EXPOSURE_AUTO;
  //   c.value = 1; // 1=manual, 3=auto; V4L2_EXPOSURE_AUTO fails...
  //   if (v4l2_ioctl(device, VIDIOC_S_CTRL, &c) != 0) {
  //     cout << "Failed to set... " << strerror(errno) << endl;
  //   }
  //   cout << "exposure: " << m_exposure << endl;
  //   v4l2_set_control(device, V4L2_CID_EXPOSURE_ABSOLUTE, m_exposure*6);
  // }
  // if (m_gain >= 0) {
  //   cout << "gain: " << m_gain << endl;
  //   v4l2_set_control(device, V4L2_CID_GAIN, m_gain*256);
  // }
  // if (m_brightness >= 0) {
  //   cout << "brightness: " << m_brightness << endl;
  //   v4l2_set_control(device, V4L2_CID_BRIGHTNESS, m_brightness*256);
  // }
  // v4l2_close(device);
  // #endif

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

void Cam::processImage() {
  // alternative way is to grab, then retrieve; allows for
  // multiple grab when processing below frame rate - v4l keeps a
  // number of frames buffered, which can lead to significant lag
  //      m_cap.grab();
  //      m_cap.retrieve(image);

  // detect April tags (requires a gray scale image)
  cv::cvtColor(m_image, m_image_gray, CV_BGR2GRAY);

  double t0;
  if (m_timing) {
    t0 = tic();
  }

  m_detections = m_tagDetector->extractTags(m_image_gray);

  if (m_timing) {
    double dt = tic()-t0;
    cout << "Extracting tags took " << dt << " seconds." << endl;
  }

  // show the current image including any detections
  if (m_draw) {
    for (unsigned int i=0; i<m_detections.size(); i++) {
      // also highlight in the image
      m_detections[i].draw(m_image);
    }
    imshow(windowName, m_image); // OpenCV call
    cv::waitKey(1);
  }
}

// The processing loop where images are retrieved, tags detected,
// and information about detections generated
void Cam::loop() {
  // capture frame
  m_cap >> m_image;

  processImage();
}

//
// Getters and Setters
// ony define needed ones as to not overclutte
// or overwrite
//
vector<AprilTags::TagDetection> Cam::getTags(){
  return m_detections;
}

double Cam::tagSize(){
  return m_tagSize;
}

double Cam::fx(){
  return m_fx;
}

double Cam::fy(){
  return m_fy;
}

double Cam::px(){
  return m_px;
}

double Cam::py(){
  return m_py;
}
