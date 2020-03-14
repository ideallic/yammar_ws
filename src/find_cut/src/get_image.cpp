//ros
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv/cv.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
//boost
#include <boost/program_options.hpp>
//srvs
#include <dosuss_srvs/find_cut.h>
//C++
#include <iostream>
#include <fstream>
#include <cmath>
#include <string>
#include <unistd.h>
#include <sys/time.h>
#include <time.h>
#include <sstream>
#include <dirent.h>
#include <algorithm>

using namespace std;
using namespace cv;
//***if use commad line input please uncommit it***
namespace bpo = boost::program_options;

sensor_msgs::Image rgb_image_f;
sensor_msgs::Image rgb_image_r;
sensor_msgs::Image rgb_image_b;
sensor_msgs::Image rgb_image_l;
Mat img_f, img_r, img_b, img_l;
//内参及消畸相关
vector<Mat> intrinsic_matrix(4);
vector<Mat> distortion_coeffs(4);
Mat M1l,M2l,M1r,M2r,M1f,M2f,M1b,M2b;

void readInstric(const string& calib,
                 vector<Mat>& intrinsic_matrix,
                 vector<Mat>& distortion_coeffs)
{
  assert(intrinsic_matrix.size()==4 && distortion_coeffs.size()==4);
  ifstream in(calib);
  if (!in.good())
  {
    printf(" ...calib.txt not found. Cannot operate without calib.txt, shutting down.\n");
    exit(1);
  }
  double fx,fy,cx,cy,k1,k2,k3,r1,r2;
  for (int i = 0; i < 4; ++i) //data文件只有4行
  {
    in >> fx >> fy >> cx >> cy >> k1>> k2 >> k3 >> r1 >> r2;
    intrinsic_matrix[i] = Mat::zeros(Size(3,3),CV_64FC1);
    distortion_coeffs[i] = Mat::zeros(Size(1,5),CV_64FC1);
    intrinsic_matrix[i].at<double>(0, 0) = fx;
    intrinsic_matrix[i].at<double>(0, 2) = cx;
    intrinsic_matrix[i].at<double>(1, 1) = fy;
    intrinsic_matrix[i].at<double>(1, 2) = cy;
    intrinsic_matrix[i].at<double>(2, 2) = 1.0;
    distortion_coeffs[i].at<double>(0, 0) = k1;
    distortion_coeffs[i].at<double>(0, 1) = k2;
    distortion_coeffs[i].at<double>(0, 2) = r1;
    distortion_coeffs[i].at<double>(0, 3) = r2;
    distortion_coeffs[i].at<double>(0, 4) = k3;
    // cout << a << " " << b << endl;
  }
}

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_rgb_f;
  image_transport::Subscriber image_sub_rgb_r;
  image_transport::Subscriber image_sub_rgb_b;
  image_transport::Subscriber image_sub_rgb_l;
  image_transport::Publisher image_pub_rgb_f;
  image_transport::Publisher image_pub_rgb_r;
  image_transport::Publisher image_pub_rgb_b;
  image_transport::Publisher image_pub_rgb_l;
 // image_transport::Subscriber image_sub_rgb3;

public:

  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_rgb_f = it_.subscribe("/camera/image_1", 1, &ImageConverter::imageCb_rgb_f, this);
    image_sub_rgb_l = it_.subscribe("/camera/image_0", 1, &ImageConverter::imageCb_rgb_l, this);
    image_sub_rgb_b = it_.subscribe("/camera/image_2", 1, &ImageConverter::imageCb_rgb_b, this);
    image_sub_rgb_r = it_.subscribe("/camera/image_3", 1, &ImageConverter::imageCb_rgb_r, this);

    image_pub_rgb_f = it_.advertise("/camera/image_0_undistort", 1);
    image_pub_rgb_l = it_.advertise("/camera/image_1_undistort", 1);
    image_pub_rgb_b = it_.advertise("/camera/image_2_undistort", 1);
    image_pub_rgb_r = it_.advertise("/camera/image_3_undistort", 1);
  }

  ~ImageConverter()
  {
  }

  void imageCb_rgb_f(const sensor_msgs::ImageConstPtr& msg)
  {
     //rgb_image_f = *msg;
     cv_bridge::CvImagePtr cv_ptr;
     try
     {
       cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);

     }
     catch (cv_bridge::Exception& e)
     {
       ROS_ERROR("cv_bridge exception: %s",e.what());
       return;
     }
     img_f =Mat::zeros(960,1280,CV_8UC3);
     img_f = cv_ptr->image;
     //pub undistort
     cv::Mat img_f_undistort;
     cv::remap(img_f,img_f_undistort,M1f,M2f,cv::INTER_LINEAR);
     sensor_msgs::ImagePtr msg_pub = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_f_undistort).toImageMsg();
     image_pub_rgb_f.publish(msg_pub);
    // imshow("saf",img_f);
     waitKey(1);
  }
  void imageCb_rgb_r(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);

    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s",e.what());
      return;
    }
    img_r =Mat::zeros(960,1280,CV_8UC3);
    img_r = cv_ptr->image;
    //pub undistort
    cv::Mat img_r_undistort;
    cv::remap(img_r,img_r_undistort,M1r,M2r,cv::INTER_LINEAR);
    sensor_msgs::ImagePtr msg_pub = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_r_undistort).toImageMsg();
    image_pub_rgb_r.publish(msg_pub);
   // imshow("saf",img_f);
    waitKey(1);
  }
  void imageCb_rgb_b(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);

    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s",e.what());
      return;
    }
    img_b =Mat::zeros(960,1280,CV_8UC3);
    img_b = cv_ptr->image;
    //pub undistort
    cv::Mat img_b_undistort;
    cv::remap(img_b,img_b_undistort,M1b,M2b,cv::INTER_LINEAR);
    sensor_msgs::ImagePtr msg_pub = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_b_undistort).toImageMsg();
    image_pub_rgb_b.publish(msg_pub);
   // imshow("saf",img_f);
    waitKey(1);
  }
  void imageCb_rgb_l(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);

    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s",e.what());
      return;
    }
    img_l =Mat::zeros(960,1280,CV_8UC3);
    img_l = cv_ptr->image;
    //pub undistort
    cv::Mat img_l_undistort;
    cv::remap(img_l,img_l_undistort,M1l,M2l,cv::INTER_LINEAR);
    sensor_msgs::ImagePtr msg_pub = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_l_undistort).toImageMsg();
    image_pub_rgb_l.publish(msg_pub);
   // imshow("saf",img_f);
    waitKey(1);
  }

};

bool get_image_f(dosuss_srvs::find_cut::Request &req, dosuss_srvs::find_cut::Response &res)
{
    int row,col;
    if (req.start)
    {
        res.p_w = 1280;
        res.p_h = 960;
        for (int i=0; i<1280*960;i++)
      {
          row = i / res.p_w;
          col = i-(row*res.p_w);

            res.pixel_b.push_back(img_f.at<Vec3b>(row,col)[0]);
            res.pixel_g.push_back(img_f.at<Vec3b>(row,col)[1]);
            res.pixel_r.push_back(img_f.at<Vec3b>(row,col)[2]);
      }

    }
    ROS_INFO("success");
    return true;
}

bool get_image_b(dosuss_srvs::find_cut::Request &req, dosuss_srvs::find_cut::Response &res)
{
    int row,col;
    if (req.start)
    {
        res.p_w = 1280;
        res.p_h = 960;
        for (int i=0; i<1280*960;i++)
      {
          row = i / res.p_w;
          col = i-(row*res.p_w);

            res.pixel_b.push_back(img_b.at<Vec3b>(row,col)[0]);
            res.pixel_g.push_back(img_b.at<Vec3b>(row,col)[1]);
            res.pixel_r.push_back(img_b.at<Vec3b>(row,col)[2]);
      }

    }
    ROS_INFO("success");
    return true;
}

bool get_image_r(dosuss_srvs::find_cut::Request &req, dosuss_srvs::find_cut::Response &res)
{
    int row,col;
    if (req.start)
    {
        res.p_w = 1280;
        res.p_h = 960;
        for (int i=0; i<1280*960;i++)
      {
            row = i / res.p_w;
            col = i-(row*res.p_w);
            res.pixel_b.push_back(img_r.at<Vec3b>(row,col)[0]);
            res.pixel_g.push_back(img_r.at<Vec3b>(row,col)[1]);
            res.pixel_r.push_back(img_r.at<Vec3b>(row,col)[2]);
      }

    }
    ROS_INFO("success");
    return true;
}

bool get_image_l(dosuss_srvs::find_cut::Request &req, dosuss_srvs::find_cut::Response &res)
{
    int row,col;
    if (req.start)
    {
        res.p_w = 1280;
        res.p_h = 960;
        for (int i=0; i<1280*960;i++)
      {
          row = i / res.p_w;
          col = i-(row*res.p_w);

            res.pixel_b.push_back(img_l.at<Vec3b>(row,col)[0]);
            res.pixel_g.push_back(img_l.at<Vec3b>(row,col)[1]);
            res.pixel_r.push_back(img_l.at<Vec3b>(row,col)[2]);
      }

    }
    ROS_INFO("success");
    return true;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "get_image_node");

  string intri_src;
  bpo::options_description desc("Program options");
  desc.add_options()
    //Options
    ("intri_src", bpo::value<string>(&intri_src)
                  -> default_value("/home/dosu/dosuss_ws_test/src/birdView_calibration_v2/calib.txt"),
                  "camera intrinsic.xml path, Usage: --intri_src ~/xxx/xxx.xml");
  // Parse the command line
  bpo::variables_map vm;
  bpo::store(bpo::parse_command_line(argc, argv, desc), vm);
  // option help
  if (vm.count("help"))
  {
    cout << desc << "\n";
    return false;
  }
  // Process options.
  bpo::notify(vm);

  //read instric parameter;
  int cols =1280;
  int rows =960;
  Size imageSize =Size(cols,rows);
  readInstric(intri_src, intrinsic_matrix, distortion_coeffs);
  cv::initUndistortRectifyMap(intrinsic_matrix[0],distortion_coeffs[0],Mat(),
                    intrinsic_matrix[0],
                    imageSize, CV_32F, M1f, M2f);
  cv::initUndistortRectifyMap(intrinsic_matrix[1],distortion_coeffs[1],Mat(),
                    intrinsic_matrix[1],
                    imageSize, CV_32F, M1l, M2l);
  cv::initUndistortRectifyMap(intrinsic_matrix[2],distortion_coeffs[2],Mat(),
                    intrinsic_matrix[2],
                    imageSize, CV_32F, M1b, M2b);
  cv::initUndistortRectifyMap(intrinsic_matrix[3],distortion_coeffs[3],Mat(),
                    intrinsic_matrix[3],
                    imageSize, CV_32F, M1r, M2r);

  ImageConverter ic;
  ros::NodeHandle n;
  ros::ServiceServer service_f = n.advertiseService("/get_image_f", get_image_f);
  ros::ServiceServer service_b = n.advertiseService("/get_image_b", get_image_b);
  ros::ServiceServer service_r = n.advertiseService("/get_image_r", get_image_r);
  ros::ServiceServer service_l = n.advertiseService("/get_image_l", get_image_l);
  cout<<"start providing service, please call"<<endl;
  // ros::ServiceServer service_1 = n.advertiseService("get_image_1", get_image_1);
  // ros::ServiceServer service_2 = n.advertiseService("get_image_2", get_image_2);
  // ros::ServiceServer service_3 = n.advertiseService("get_image_3", get_image_3);

  ros::Rate loop_rate(20);
  while (ros::ok())
  {

     ros::spinOnce();
     loop_rate.sleep();
  }
  ros::spin();
  return 0;
}
