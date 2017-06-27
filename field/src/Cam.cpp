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
  int c;
  while ((c = getopt(argc, argv, "c:hv")) != -1) {
    switch (c) {
      case 'c' : m_deviceId = atoi(optarg);    break;
      case 'v' : m_draw = false;    break;
      case 'h' : std::cout << std::endl
                           <<"This is the slightly helpful help file!" << std::endl
                           << "Flags:" << std::endl
                           << "-c <int> set camera device id" << std::endl
                           << "-h help (but you probably figured that out)" << std::endl
                           << "-v turns off the video" << std::endl
                           << std::endl
                           ;std::exit(0);
      default : std::cout << "error from flags" << std::endl;   break;
    }
  }
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

// Finds open video device and then starts capture
// Throws an error if it cannot open
void Cam::setupVideo() {
  m_cap = cv::VideoCapture(m_deviceId);
  if(!m_cap.isOpened()) {
    cerr << "ERROR: Can't find video device " << m_deviceId << "\n";
    exit(1);
  }
  m_cap.set(CV_CAP_PROP_FRAME_WIDTH, m_width);
  m_cap.set(CV_CAP_PROP_FRAME_HEIGHT, m_height);
  cout << "Camera successfully opened (ignore error messages above...)" << endl;
  cout << "Specified resolution: "
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
