#include <ros/ros.h>
#include <string>
#include <iostream>
#include <cv_bridge/cv_bridge.h>
// #include <utils/timestamp.h>
#include <cassert>
#include <std_msgs/Int8.h>
#include <tf/transform_datatypes.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
#include "DBoW2.h"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <vector>

using namespace cv;
using namespace DBoW2;
using namespace std;

#define IMAGE_SAVE_PATH "./image"
#define CONST_IMAGE_SCALE_FACTOR_INV_DEFAULT     0.25f
#define MACRO_RGB_IMG_TOPIC                 "/head_camera/rgb/image_raw"
#define BEST_MATCH_INDEX_TOPIC              "/place_recognizor/bestmatch"
#define DATABASE_PATH                       "/home/yifan/DBoW2/build/small_db.yml.gz"

typedef DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB>
    ORBVocabulary;

/// ORB Database
typedef DBoW2::TemplatedDatabase<DBoW2::FORB::TDescriptor, DBoW2::FORB>
    ORBDatabase;


void wait()
{
  cout << endl << "Press enter to save image" << endl;
  getchar();
}

void changeStructure(const cv::Mat &plain, vector<cv::Mat> &out)
{
  out.resize(plain.rows);

  for(int i = 0; i < plain.rows; ++i)
  {
    out[i] = plain.row(i);
  }
}

vector<cv::Mat> extracting_feature(const cv::Mat &img) {
    // extract feature of the image
    vector<cv::Mat > feature;
    cv::Mat mask;
    vector<cv::KeyPoint> keypoints;
    cv::Ptr<cv::ORB> orb = cv::ORB::create();
    cv::Mat descriptors;

    orb->detectAndCompute(img, mask, keypoints, descriptors);
    changeStructure(descriptors, feature);
    cout << "sucessfully extract features of the image" << endl;
    // cout << feature[10] << endl;
    return feature;
}

int find_bestmatch(const vector<cv::Mat> &feature, const string &database_path){
    ORBDatabase db(database_path);
    QueryResults ret;

    BowVector v;
    const int k = 9;
    const int L = 3;
    const WeightingType weight = TF_IDF;
    const ScoringType scoring = L1_NORM;
    OrbVocabulary voc(k, L, weight, scoring);

    voc.transform(feature, v);
    // cout<<feature[10]<<endl;
    cout<<"Starting query"<<endl;
    if(feature.size()>0){
      db.query(feature, ret);

      cout<<"Best match is: "<<ret[0].Id<<endl;
    
    return ret[0].Id;
    }
    else {
      cout<<"NO FEATURES!!!"<<endl;
      return 1000;
    }

}

void saveImg(const cv::Mat pic, int image_num)
{
    ostringstream filepath;
    filepath<<"./image/image"<<image_num<<".png";
    imwrite(filepath.str(), pic);
    cout<<"save succeefully"<<endl;
}

class PlaceRecognizor
{
public:
ros::NodeHandle nh;
image_transport::Subscriber sub_image;
Mat scaledRGBImage;
ros::Publisher index_pub;
int im_n;



PlaceRecognizor()
{
    // if(!lcm.good())
    //     ros::shutdown();
    image_transport::ImageTransport it(nh);
    im_n = 0;
    sub_image = it.subscribe(MACRO_RGB_IMG_TOPIC, 1,  &PlaceRecognizor::ImageRGBCallback, this);
    index_pub = nh.advertise<std_msgs::Int8>(BEST_MATCH_INDEX_TOPIC,1000);
}

~PlaceRecognizor()
{

}
void ImageRGBCallback(const sensor_msgs::ImageConstPtr& pMsg);



};
void PlaceRecognizor::ImageRGBCallback(const sensor_msgs::ImageConstPtr& pMsg)
{
        Mat bgrimg;
        try
        {   
           bgrimg=cv_bridge::toCvShare(pMsg,"bgr8")->image;
           //for test
          //  bgrimg=imread("./image/image1.png",IMREAD_COLOR);
          // cout<<bgrimg[1]<<endl;
           resize(bgrimg,bgrimg,Size(640,480));
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("Could not convert from '%s' to 'bgr8'.", pMsg->encoding.c_str());
        }

        
        Mat greyimg;

        cvtColor(bgrimg,greyimg,CV_BGR2GRAY);

        // imwrite("./image/test.jpeg", greyimg);

        std_msgs::Int8 best_index;
        
        vector<Mat> feature = extracting_feature(greyimg);
        try {
          best_index.data = find_bestmatch(feature, DATABASE_PATH);

          index_pub.publish(best_index);
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_INFO("cannot get any features\n");
        }

        // wait();

        // saveImg(greyimg,im_n);

        // im_n += 1;
        
}






int main(int argc, char** argv)
{

  ros::init(argc, argv, "lcm_republisher");
  
  PlaceRecognizor roswrapper;
  ros::spin();
  
  return 0;
}
