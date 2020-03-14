//ros
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_datatypes.h>
#include <ros/package.h>
//boost
#include <boost/program_options.hpp>
//opencv
#include <opencv/cv.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
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
#include <thread>
//srvs
#include <dosuss_srvs/find_cut.h>

#define PI 3.14159

using namespace std;
using namespace cv;
//***if use commad line input please uncommit it***
namespace bpo = boost::program_options;

vector<Mat> intrinsic_matrix(4);
vector<Mat> distortion_coeffs(4);
vector<vector<Point2f>> targetPoint, sourcePoint;
Mat Birdtransform[4];
Size ShiftAdjust, Shift, chessBordWidth, mSize, carSize;
int carShift;
Rect r[4];

image_transport::Publisher image_pub;

/****************************************************************/
/*************** Parameter Configuration ************************/
/****************************************************************/
class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Publisher cam_pub;

public:
  ImageConverter(string topic):
  it_(nh_)
  {
    cam_pub = it_.advertise(topic, 5);
    // image_sub_rgb_f = it_.subscribe("/camera/image_0", 1, &outputImage0);
    // image_sub_rgb_l = it_.subscribe("/camera/image_1", 1, &outputImage1);
    // image_sub_rgb_b = it_.subscribe("/camera/image_2", 1, &outputImage2);
    // image_sub_rgb_r = it_.subscribe("/camera/image_3", 1, &outputImage3);
  }
  ~ImageConverter()
  {
  }
  void pub(sensor_msgs::ImagePtr msg)
  {
    cam_pub.publish(msg);
    //ros::spinOnce();
  }
};

int getdir (std::string dir, std::vector<std::string> &files)
{
  DIR *dp;
  struct dirent *dirp;
  if((dp  = opendir(dir.c_str())) == NULL)
  {
    return -1;
  }

  while ((dirp = readdir(dp)) != NULL) {
    std::string name = std::string(dirp->d_name);

    if(name != "." && name != "..")
    files.push_back(name);
  }
  closedir(dp);
  std::sort(files.begin(), files.end());
  if(dir.at( dir.length() - 1 ) != '/')
    dir = dir+"/";
	for(unsigned int i=0;i<files.size();i++)
	{
		if(files[i].at(0) != '/')
		files[i] = dir + files[i];
	}

  return files.size();
}

void multithreads(Mat target, dosuss_srvs::find_cut srv, Size img)
{
  target = Mat::zeros(img.height, img.width,CV_8UC3);
  for (int i=0; i<img.height;i++)
  {
    for (int j=0; j<img.width;j++)
    {
      target.at<Vec3b>(i,j)[0]=srv.response.pixel_r[j+i*img.width];
      target.at<Vec3b>(i,j)[1]=srv.response.pixel_g[j+i*img.width];
      target.at<Vec3b>(i,j)[2]=srv.response.pixel_b[j+i*img.width];
    }
  }
}

Mat img_f, img_l,img_b,img_r;
void outputImage0(const sensor_msgs::ImageConstPtr& msg)
{
   //rgb_image_f = *msg;
   cv_bridge::CvImagePtr cv_ptr;
   try
   {
     cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::RGB8);

   }
   catch (cv_bridge::Exception& e)
   {
     ROS_ERROR("cv_bridge exception: %s",e.what());
     return;
   }
   img_f =Mat::zeros(960,1280,CV_8UC3);
   img_f = cv_ptr->image;
   waitKey(1);
}

class ImageGrabber
{
public:
  ImageGrabber(string topic, string car):ic(topic), car_src(car){}

  void GrabFrames(const sensor_msgs::ImageConstPtr& msgFront,
                  const sensor_msgs::ImageConstPtr& msgLeft,
                  const sensor_msgs::ImageConstPtr& msgBack,
                  const sensor_msgs::ImageConstPtr& msgRight);

  bool do_rectify;
  Mat M1l,M2l,M1r,M2r,M1f,M2f,M1b,M2b;
  Mat K,D;
  string car_src;
  ImageConverter ic;
};

void ImageGrabber::GrabFrames(const sensor_msgs::ImageConstPtr& msgFront,
                              const sensor_msgs::ImageConstPtr& msgLeft,
                              const sensor_msgs::ImageConstPtr& msgBack,
                              const sensor_msgs::ImageConstPtr& msgRight)
{
  clock_t start,end;
  double duration;
  start = clock();
  // Copy the ros image message to cv::Mat.
  cv_bridge::CvImageConstPtr cv_ptrLeft;
  try
  {
    cv_ptrLeft = cv_bridge::toCvShare(msgLeft,"bgr8");
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  cv_bridge::CvImageConstPtr cv_ptrRight;
  try
  {
    cv_ptrRight = cv_bridge::toCvShare(msgRight,"bgr8");
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  cv_bridge::CvImageConstPtr cv_ptrBack;
  try
  {
    cv_ptrBack = cv_bridge::toCvShare(msgBack,"bgr8");
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  cv_bridge::CvImageConstPtr cv_ptrFront;
  try
  {
    cv_ptrFront = cv_bridge::toCvShare(msgFront,"bgr8");
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  if(do_rectify)
  {
    cv::Mat v[4];
    cv::remap(cv_ptrFront->image,v[0],M1f,M2f,cv::INTER_LINEAR);
    cv::remap(cv_ptrLeft ->image,v[1],M1l,M2l,cv::INTER_LINEAR);
    cv::remap(cv_ptrBack ->image,v[2],M1b,M2b,cv::INTER_LINEAR);
    cv::remap(cv_ptrRight->image,v[3],M1r,M2r,cv::INTER_LINEAR);

    //process start here;
    Mat b[4];
    Mat m = Mat(mSize, CV_8UC3 );

    for (int i = 0; i < 4; i++)
    {
      if(!v[i].data)
      {
        continue;
      }
      warpPerspective(v[i], b[i], Birdtransform[i], mSize);
      // if(i==0){
      //imshow("test",b[i]);
        Mat mask = b[i](r[i]).clone();
        b[i](r[i]).copyTo(m(r[i]),mask);
      //	}
    }
    //TODO1　边缘过渡

    //TODO2 加纯黑mask去除中心扰动

    // Rotate the image
    int iAngle = 184;
    int iImageHeight = m.rows / 2;
    int iImageWidth = m.cols / 2;
    Mat matRotation = getRotationMatrix2D( Point(iImageWidth, iImageHeight), (iAngle - 180), 1 );
    Mat m_rotated;
    warpAffine(m, m_rotated, matRotation, m.size());
    //cut useless edge
    Mat m_final = m_rotated(Rect(80,80,mSize.width-160,mSize.height-160)).clone();

    //cut useless center
    // Mat cut=imread(car_src);
    // Mat cut_temp;
    int shift_w =10;
    int shift_h = 10;
    int center_w = 450;
    int center_h = 500;
    // resize(cut,cut_temp,Size(center_w,center_h),CV_INTER_CUBIC);
    // cut_temp.copyTo(m_final(Rect(m_final.cols/2-center_w/2+shift_w,
    //                              m_final.rows/2-center_h/2+shift_h,
    //                              center_w,center_h)));

    //set black center ROI
    Mat mm=m_final(Rect(m_final.cols/2-center_w/2+shift_w,
                                 m_final.rows/2-center_h/2+shift_h,
                                 center_w,center_h));//ROI设置
    mm={Scalar(0,0,0)};//把ROI中的像素值改为黑色
    /*
    if need saving img for training, please uncommit following sentence.
    Notice: the saveDir should be valid.
    */
    // static int count = 0;
    // string Name = to_string(count++) + ".jpg";
    // string saveDir = "/media/dosu/Elements/20191025/train7/";
    // imwrite(saveDir+Name, v[0]);
    // cout<<"image: " <<count<<".jpg saved!" <<endl;

    end =clock();
    duration = double(end - start)* 500.0/ CLOCKS_PER_SEC;
    stringstream str1;
    str1<<"time per loop: "<<duration<<" ms";
    putText(m_final, str1.str(),Point(50,50),FONT_HERSHEY_PLAIN,2,CV_RGB(255,255,0),2.8,false);
    namedWindow("bird view", CV_WINDOW_NORMAL);
    imshow("bird view", m_final);

    cv_bridge::CvImage cvi;
    ros::Time time=ros::Time::now();
    cvi.header.stamp = time;
    cvi.header.frame_id = "image";
    cvi.encoding = "bgr8";
    cvi.image = m_final;
    sensor_msgs::Image im;
    cvi.toImageMsg(im);
    image_pub.publish(im);


    waitKey(10);
    //******* if need pub the top view image, please uncommit the following two lines.
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(),"bgr8",m_final).toImageMsg();
    ic.pub(msg);
    waitKey(10);
    }

}

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

void readConfig(const char* configFile = "config.yml",
                const char* targetFile = "target.yml")
{
    sourcePoint.resize(4);
    targetPoint.resize(4);
    for (int i=0;i<4;i++)
    {
      sourcePoint[i].resize(4);
      targetPoint[i].resize(4);
    }

    FileStorage fs(configFile, FileStorage::READ);
    if (fs.isOpened())
    {
        for (int i = 0; i < 4; i++)
        {
            for (int k = 0; k < 4; k++)
            {
                char buf[20];
                sprintf(buf, "sourcePoint%d%d", i, k);
                fs[buf] >> sourcePoint[i][k];
            }
        }
        //SourcePoint_OK = true;  // source point reading completed
        cout << "Config file read sucessfully!" << endl;
    }
    else{
      cout << "[ERROR] There is not such file named config.xml" << endl;
      cout << "please refer to -> Usage: ./bin/xxx --get_Points"<< endl;
      exit(-1);
    }

    FileStorage fss(targetFile, FileStorage::READ);
    if (fss.isOpened())
    {
        for (int i = 0; i < 4; i++)
        {
            for (int k = 0; k < 4; k++)
            {
                char buf[20];
                sprintf(buf, "targetPoint%d%d", i, k);
                fss[buf] >> targetPoint[i][k];
            }
        }
        //SourcePoint_OK = true;  // source point reading completed
        cout << "target file read sucessfully!" << endl;
    }
    else{
      cout << "[ERROR] There is not such file named target.xml" << endl;
      cout << "please refer to -> Usage: ./bin/xxx --get_Points"<< endl;
      exit(-1);
    }

    for (size_t i = 0; i < 4 ; i++)
    {
        Birdtransform[i] = getPerspectiveTransform(sourcePoint[i], targetPoint[i]);
    }
}

void setparam()
{
  //**************parameter setting***********
  chessBordWidth = Size(100,140); //static
  Shift = Size(600,800); //outside shift (hard changable, need relocate target points in birdView_calibration_v2)
  ShiftAdjust = Size(502,558); //inside shift (static)
  carSize = Size(340,400);  //easy changable
  carShift = 1050; //easy changable

  mSize = Size(Shift.width * 2 + ShiftAdjust.width + chessBordWidth.width * 2,
               Shift.height * 2 + ShiftAdjust.height + chessBordWidth.height * 2);

  r[0] = Rect(0, 0, mSize.width, carShift);
  r[1] = Rect(0, 0, int((mSize.width - carSize.width) / 2.0), mSize.height);
  r[2] = Rect(0, carSize.height + carShift, mSize.width, mSize.height - carSize.height - carShift);
  r[3] = Rect(int((mSize.width + carSize.width) / 2.0), 0, int((mSize.width - carSize.width) / 2.0), mSize.height);
}

int main(int argc, char** argv)
{
    //command input
    ros::init(argc, argv, "find_cut_node_realtime");
    ros::start();
    string intri_src;
    string extri_src;
    string target_src;
    string car_src;
    bool do_rectify;
    bpo::options_description desc("Program options");
    desc.add_options()
      //Options
      ("intri_src", bpo::value<string>(&intri_src)
  									-> default_value("/home/wj/CLionProjects/birdview/src/birdView_calibration_v2/calib.txt"),
  									"camera intrinsic.xml path, Usage: --intri_src ~/xxx/xxx.xml")
      ("extri_src", bpo::value<string>(&extri_src)
                    -> default_value("/home/wj/CLionProjects/birdview/src/birdView_calibration_v2/config.xml"),
                    "camera extrinsic.xml path, Usage: --extri_src ~/xxx/xxx.xml")
      ("target_src",bpo::value<string>(&target_src)
                    -> default_value("/home/wj/CLionProjects/birdview/src/birdView_calibration_v2/target.xml"),
                    "camera extrinsic.xml path, Usage: --extri_src ~/xxx/xxx.xml")
      ("car_src",   bpo::value<string>(&car_src)
                    -> default_value("/home/wj/CLionProjects/birdview/src/birdView_calibration_v2/img/ParamSetting.png"),
                    "car image path, if needed, Usage: --car_src ~/xxx/xxx.png")
      ("do_rectify",bpo::value<bool>(&do_rectify) -> default_value(true),
                    "use image from topics or not, Usage: --ros_on true/false");
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

    cout<<"start this node..."<<endl;

    ImageGrabber igb("camera/top_view", car_src);
    igb.do_rectify = do_rectify;

    readConfig(extri_src.c_str(),target_src.c_str());
    setparam();

    if(igb.do_rectify)
    {
      //read instric parameter;
      int cols =1280;
      int rows =960;
      Size imageSize =Size(cols,rows);
      readInstric(intri_src, intrinsic_matrix, distortion_coeffs);
      igb.K = intrinsic_matrix[0].clone();
      igb.D = distortion_coeffs[0].clone();
      cv::initUndistortRectifyMap(intrinsic_matrix[0],distortion_coeffs[0],Mat(),
                        intrinsic_matrix[0],
                        imageSize, CV_32F, igb.M1f, igb.M2f);
      cv::initUndistortRectifyMap(intrinsic_matrix[1],distortion_coeffs[1],Mat(),
                        intrinsic_matrix[1],
                        imageSize, CV_32F, igb.M1l, igb.M2l);
      cv::initUndistortRectifyMap(intrinsic_matrix[2],distortion_coeffs[2],Mat(),
                        intrinsic_matrix[2],
                        imageSize, CV_32F, igb.M1b, igb.M2b);
      cv::initUndistortRectifyMap(intrinsic_matrix[3],distortion_coeffs[3],Mat(),
                        intrinsic_matrix[3],
                        imageSize, CV_32F, igb.M1r, igb.M2r);
      // cv::initUndistortRectifyMap(intrinsic_matrix[0],distortion_coeffs[0],Mat(),
      //                   getOptimalNewCameraMatrix(intrinsic_matrix[0],distortion_coeffs[0], imageSize, 1, imageSize, 0),
      //                   imageSize, CV_32F, igb.M1f, igb.M2f);
      // cv::initUndistortRectifyMap(intrinsic_matrix[1],distortion_coeffs[1],Mat(),
      //                   getOptimalNewCameraMatrix(intrinsic_matrix[1],distortion_coeffs[1], imageSize, 1, imageSize, 0),
      //                   imageSize, CV_32F, igb.M1l, igb.M2l);
      // cv::initUndistortRectifyMap(intrinsic_matrix[2],distortion_coeffs[2],Mat(),
      //                   getOptimalNewCameraMatrix(intrinsic_matrix[2],distortion_coeffs[2], imageSize, 1, imageSize, 0),
      //                   imageSize, CV_32F, igb.M1b, igb.M2b);
      // cv::initUndistortRectifyMap(intrinsic_matrix[3],distortion_coeffs[3],Mat(),
      //                   getOptimalNewCameraMatrix(intrinsic_matrix[3],distortion_coeffs[3], imageSize, 1, imageSize, 0),
      //                   imageSize, CV_32F, igb.M1r, igb.M2r);

    }
    ros::NodeHandle nh;
    message_filters::Subscriber<sensor_msgs::Image> front_sub(nh, "camera/image_1", 1);
    message_filters::Subscriber<sensor_msgs::Image> left_sub(nh, "camera/image_0", 1);
    message_filters::Subscriber<sensor_msgs::Image> back_sub(nh, "camera/image_2", 1);
    message_filters::Subscriber<sensor_msgs::Image> right_sub(nh, "camera/image_3", 1);
    image_transport::ImageTransport transport(nh);
    image_pub= transport.advertise("/birdview", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image,sensor_msgs::Image,sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), front_sub,left_sub,back_sub,right_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabFrames,&igb,_1,_2,_3,_4));

    ros::spin();
    ros::shutdown();
    return 0;
}
