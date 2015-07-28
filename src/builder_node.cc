#include <iostream>
#include <vector>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

#include "ORBextractor.h"

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>

#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include "std_msgs/String.h"
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

#include <DBoW2.h> // defines ORBVocabulary and ORBDatabase
#include <DUtils.h>
#include <DUtilsCV.h> // defines macros CVXX
#include <DVision.h>

using namespace DBoW2;
using namespace DUtils;
using namespace std;


int kbhit(void);
void imageCallback(const sensor_msgs::ImageConstPtr& msg);
void buildDatabase(std::string path);
void changeStructure(const cv::Mat &plain, vector<cv::Mat> &out, int L);

static int frame_interval = 1;
static int counter = 0;

string descriptorType;

int main(int argc, char ** argv) {
  ros::init(argc, argv, "orb2bowdb");
  ros::NodeHandle _nh("~");

  string listen_to = "/camera/image_raw";
  string output_path = "/tmp/orbbow_db.yml.gz";

  _nh.getParam("frameInterval", frame_interval);
  counter = frame_interval; //set to frame interval so that 1rst frame processed
  _nh.getParam("outputPath", output_path);
  _nh.getParam("listenTo", listen_to);

  _nh.getParam("featureType", descriptorType);

  image_transport::ImageTransport _it(_nh);
  image_transport::Subscriber sub = _it.subscribe(listen_to, 1, imageCallback);

  cout << "listening to: " << listen_to << endl;
  cout << "Capturing frames. Press the \'s\' key to stop Capturing and build the vocabulary database" << endl;

  //loop until node killed.
  while (ros::ok()) {
    //detect keyboard input
    if (kbhit()) {
      //if s pressed then save and quit
      int c = getchar();   //check if database need to be saved.
      if (c == 's') {
        ROS_INFO("Building database....");
        buildDatabase(output_path);
        cout << "Saving database to: " << output_path << endl;
        ROS_INFO("Database successfully generated.");
        ros::shutdown();
      } else {
        cout << "press the s key to save the database and quit the node." << endl;
      }
    }
    ros::spinOnce();
  }
  ROS_INFO("Shutdown requested....");
  return 0;
}

const int MAX_FEATURES = 1500;
const float SCALE = 1.2f;
const int NLAYERS = 8;
const int EDGE_THRESHOLD = 16;

void toDescriptorVector(const cv::Mat &plain, std::vector<cv::Mat> &out) {
  out.resize(plain.rows);
  for (unsigned int i = 0; i < plain.rows; i++) {
    out[i] = plain.row(i);
  }
}

void toSurfVector(const vector<float> &plain, vector<vector<float> > &out,
  int L)
{
  out.resize(plain.size() / L);

  unsigned int j = 0;
  for(unsigned int i = 0; i < plain.size(); i += L, ++j)
  {
    out[j].resize(L);
    std::copy(plain.begin() + i, plain.begin() + i + L, out[j].begin());
  }
}

void toSurfVector(const cv::Mat &plain, vector<vector<float> > &out)
{
  out.resize(plain.rows);
  int L = plain.cols;

  for (int j = 0; j < plain.rows; j++) {
    cv::Mat d  = plain.row(j);
    out[j].resize(L);
    for(int r=0;r<plain.cols;r++){
      out[j][r] = d.at<float>(r);
    }
  }
}

vector<vector<cv::Mat> > features;
vector<vector<vector<float> > > featuresSurf;

void extractDescriptors(const cv::Mat& img) {
  //Compute ORB descriptor + append to list of descriptors
  cv::Mat mask, descriptors;
  std::vector<cv::KeyPoint> keypoints;

  if (descriptorType == "orb") {
    //Extract features for current image.
    features.push_back(vector<cv::Mat>());
    static ORB_SLAM::ORBextractor orb(MAX_FEATURES, SCALE, NLAYERS,
                                      ORB_SLAM::ORBextractor::FAST_SCORE, EDGE_THRESHOLD);
    orb(img, mask, keypoints, descriptors);
    toDescriptorVector(descriptors, features.back());
  }
  if (descriptorType == "brisk") {
    //brisk matching
    cv::BRISK BRISKD(EDGE_THRESHOLD, NLAYERS, SCALE);
    BRISKD.create("brisk");

    BRISKD.detect(img, keypoints);
    BRISKD.compute(img, keypoints, descriptors);

    features.push_back(vector<cv::Mat>());
    toDescriptorVector(descriptors, features.back());
  }
  if (descriptorType == "surf") {
    static ORB_SLAM::ORBextractor orb(MAX_FEATURES, SCALE, NLAYERS,
                                      ORB_SLAM::ORBextractor::FAST_SCORE, EDGE_THRESHOLD);
    cv::Mat desc_orb;
    orb(img, mask, keypoints, desc_orb);
    cv::SurfDescriptorExtractor extractor;
    extractor.compute(img, keypoints, descriptors);

    featuresSurf.push_back(vector<vector<float> >());
    toSurfVector(descriptors, featuresSurf.back());
  }

}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  if (counter < frame_interval) {
    counter++;
    return;
  }

  cv::Mat img;
  try {
    //Get B&W image from bridge.
    img = cv_bridge::toCvShare(msg, "mono8")->image;
    extractDescriptors(img);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  } catch (exception& e) {
    ROS_ERROR("exception: %s", e.what());
  }
  counter = 0; //reset counter
}

void buildDatabase(std::string path)
{
  // branching factor and depth levels
  const int k = 10;
  const int L = 6;

  const WeightingType weight = TF_IDF;
  const ScoringType score = L1_NORM;

  cout << "building a: " << k << "^" << L << " vocabulary..." << endl;
  cout << "This can take a very long time..." << endl;

  if (descriptorType == "orb") {
    ORBVocabulary voc(k, L, weight, score);
    voc.create(features);
    cout << "Vocabulary information: " << endl
         << voc << endl << endl;
    voc.save(path);
  }
  if (descriptorType == "brisk") {
    BriskVocabulary voc(k, L, weight, score);
    voc.create(features);
    cout << "Vocabulary information: " << endl
         << voc << endl << endl;
    voc.save(path);
  }
  if (descriptorType == "surf") {
    Surf64Vocabulary voc(k, L, weight, score);
    voc.create(featuresSurf);
    cout << "Vocabulary information: " << endl
         << voc << endl << endl;
    voc.save(path);
  }
  cout << "... done!" << endl;
}

//Helper to detect if a key has been presses
int kbhit(void)
{
  struct termios oldt, newt;
  int ch;
  int oldf;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

  ch = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);

  if (ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }

  return 0;
}