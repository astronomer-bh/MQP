#include "Cam.hpp"

// utility function to provide current system time (used below in
// determining frame rate at which images are being processed)
double tic() {
  struct timeval t;
  gettimeofday(&t, NULL);
  return ((double)t.tv_sec + ((double)t.tv_usec)/1000000.);
}

Cam::Cam(int argc, char* argv[], int keepRunning) :
// default settings, most can be modified through command line options (see below)
keepRunning(keepRunning),
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
hasDetections(false),

m_deviceId(0),

m_windowName("Field")
{
  parseOptions(argc, argv);
  m_tagDetector = new AprilTags::TagDetector(m_tagCodes);
  setupVideo();

  // open window for camera drawing (Optional)
  if (m_draw) {
    cv::namedWindow(m_windowName, 1);
  }

  for (int i = 0; i < NUM_THREADS; i++){
    threads[i] = thread(&Cam::processImage, this);
  }

  startThreads();
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
  cout << "Actual resolution: "
  << m_cap.get(CV_CAP_PROP_FRAME_WIDTH) << "x"
  << m_cap.get(CV_CAP_PROP_FRAME_HEIGHT) << endl;

}

// decolors image and then extracts tags
void Cam::processImage() {
  while(keepRunning) {
    if(!m_image.empty()){
      // detect April tags (requires a gray scale image)
      cv::cvtColor(m_image, m_image_gray, CV_BGR2GRAY);

      m_detections = m_tagDetector->extractTags(m_image_gray);
    }
  }
}

// pulls images from the camera and puts them into the buffer
// processing threads take care of actual image processing
void Cam::pullImage(){
  // capture frame
  m_cap >> m_image;
}

// draws detections alongside current image
void Cam::drawImage(){
  // show the current image including any detections
  if (m_draw) {
    for (unsigned int i=0; i<m_detections.size(); i++) {
      // also highlight in the image
      m_detections[i].draw(m_image);
    }
    imshow(m_windowName, m_image); // OpenCV call
    cv::waitKey(1);
  }
}

// starts processing threads
void Cam::startThreads(){
  for (int i = 0; i < NUM_THREADS; i++){
    threads[i].join();
  }
}

// The processing loop where images are retrieved, tags detected,
// and information about detections generated
void Cam::loop() {
  pullImage();
  drawImage();
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
