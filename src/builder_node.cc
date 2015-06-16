#include <iostream>
#include <vector>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>

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

vector<vector<cv::Mat> > features;
static int frame_interval=1;
static int counter=0;

int main(int argc, char ** argv) {
  ros::init(argc, argv, "orb2bowdb");
  ros::NodeHandle _nh("~");

  string listen_to = "/camera/image_raw";
  string output_path = "/tmp/orbbow_db.yml.gz";

  _nh.getParam("frameInterval", frame_interval);
  counter=frame_interval; //set to frame interval so that 1rst frame processed
  _nh.getParam("outputPath", output_path);
  _nh.getParam("listenTo", listen_to);

  image_transport::ImageTransport _it(_nh);
  image_transport::Subscriber sub = _it.subscribe(listen_to, 1, imageCallback);

  cout << "listening to: " << listen_to;

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
      }else{
        cout << "press the s key to save the database and quit the node." << endl;
      }
    }
    ros::spinOnce();
  }
  ROS_INFO("Shutdown requested....");
  return 0;
}

const int MAX_FEATURES = 2000;
const float SCALE = 1.2f;
const int NLAYERS = 8;
const int PATCH_SIZE = 31;
const int EDGE_THRESHOLD = 16;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  if(counter<frame_interval){
    counter++;
    return;
  }

  cv::Mat img;
  try {
    //Get B&W image from bridge.
    img = cv_bridge::toCvShare(msg, "mono8")->image;
    //Compute ORB descriptor + append to list of descriptors
    cv::Mat mask;
    vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;

    //get orb features and descriptors in image.
    static cv::ORB orb(MAX_FEATURES, SCALE, NLAYERS, EDGE_THRESHOLD, 0, 2, cv::ORB::FAST_SCORE, PATCH_SIZE);
    orb(img, mask, keypoints, descriptors);

    //Add Results for this image
    features.push_back(vector<cv::Mat>());
    changeStructure(descriptors, features.back(), orb.descriptorSize());
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
  counter=0; //reset counter
}

void buildDatabase(std::string path)
{
  // branching factor and depth levels
  const int k = 10;
  const int L = 6;

  const WeightingType weight = TF_IDF;
  const ScoringType score = L1_NORM;

  ORBVocabulary voc(k, L, weight, score);

  cout << "building a: " << k << "^" << L << " vocabulary..." << endl;
  voc.create(features);
  cout << "... done!" << endl;

  cout << "Vocabulary information: " << endl
       << voc << endl << endl;

  // save the vocabulary to disk
  cout << endl << "Saving vocabulary..." << endl;
  voc.save(path);
  cout << "Done" << endl;
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
 
  if(ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }
 
  return 0;
}

void changeStructure(const cv::Mat &plain, vector<cv::Mat> &out, int L)
{
  out.resize(plain.rows * plain.cols / L);

  for (unsigned int i = 0; i < plain.rows; i++) {
    out[i] = plain.row(i);
  }
}