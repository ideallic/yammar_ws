#include <iostream>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointField.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <Eigen/Core>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/mls.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl/kdtree/kdtree_flann.h>

ros::Subscriber subRGBD;
ros::Publisher filter_pub;
image_transport::Publisher image_pub;
using namespace std;
using namespace cv;

void boud_depth(Mat& rgb,Mat& depth,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_kd);
void boud_RGB(Mat& rgb,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_kd);

Mat get_Vertical(Mat src)
{
    Mat dst = Mat::zeros(src.size(), src.type());
    return getStructuringElement(MORPH_RECT,Size(src.cols/16,1),Point(-1,-1));
}
//结构元素（获取水平算子）
Mat get_Horizontal(Mat src)
{
    Mat dst = Mat::zeros(src.size(), src.type());
    return getStructuringElement(MORPH_RECT, Size(1, src.rows / 16), Point(-1, -1));
}
//腐蚀
Mat erode_Img(Mat src,Mat kernel)
{
    Mat dst = Mat::zeros(src.size(), src.type());
    erode(src, dst, kernel);
    return dst;
}

//膨胀
Mat dilate_Img(Mat src, Mat kernel)
{
    Mat dst = Mat::zeros(src.size(), src.type());
    dilate(src, dst, kernel);
    return dst;
}


void boud_RGBD(const sensor_msgs::ImageConstPtr& rgbimg,const sensor_msgs::ImageConstPtr& depthimg) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_kd (new pcl::PointCloud<pcl::PointXYZ>);
    Mat depth_raw;

    cv_bridge::CvImageConstPtr cv_ptrdepth;
    cv_ptrdepth = cv_bridge::toCvShare(depthimg);
    depth_raw = cv_ptrdepth->image;

    Mat rgb;
    cv_bridge::CvImageConstPtr cv_ptrrgb;
    cv_ptrrgb = cv_bridge::toCvShare(rgbimg);
    rgb = cv_ptrrgb->image;

    Mat depth(depth_raw.size(),CV_16UC1);
    for(int row=0;row<rgb.rows;row++)
        for(int col=0;col<rgb.cols;col++ )
        {
            float z = float(depth_raw.at<float>(row,col))*1000;
            depth.at<ushort>(row,col)=z;

            Vec3b temp= rgb.at<Vec3b>(row,col);
            rgb.at<Vec3b>(row,col)[0]=temp[2];
            rgb.at<Vec3b>(row,col)[2]=temp[0];
        }
    boud_depth(rgb, depth,cloud_kd);
    //boud_RGB(rgb,cloud_kd);

}
//构建最近邻搜索数据
vector<Point2i> pointdepthimg;
pcl::PointXYZ Point_kd;


int main(int argc,char** argv)
{
    ros::init(argc,argv,"boud_RGBD");
    ros::NodeHandle nh;
    image_transport::ImageTransport transport(nh);
    image_pub= transport.advertise("/boud_depth", 1);

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/realsense_sr300/ylx/rgb", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/realsense_sr300/ylx/depth", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);
    sync.registerCallback(boost::bind(&boud_RGBD,_1,_2));

    ros::spin();
    return 0;
}

void boud_depth(Mat& rgb, Mat& depth,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_kd)
{
    //transform to pointcloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudin (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_uncut(new pcl::PointCloud<pcl::PointXYZRGB>);
    cloudin->header.frame_id="/frame";
    cloud_filtered->header.frame_id="/frame";
    plane_uncut->header.frame_id="/frame";

    pcl::PointXYZRGB Point;
    for(int row=0;row<rgb.rows;row++)
        for(int col=0;col<rgb.cols;col++ )
        {
            float z = float(depth.at<ushort>(row,col))/1000;
            float y = (row - 232.171) * z / 615.312;
            float x = (col - 323.844) * z / 615.372;
            if(y>0 && z<10)
            {
                Point.x=x;
                Point.y=y;
                Point.z=z;
                Point.b=rgb.ptr<uchar>(row)[col*3];
                Point.g=rgb.ptr<uchar>(row)[col*3+1];
                Point.r=rgb.ptr<uchar>(row)[col*3+2];
                cloudin->points.push_back(Point);
            }
        }

    //down_sampling
    pcl::VoxelGrid<pcl::PointXYZRGB> filter;
    filter.setInputCloud(cloudin);
    filter.setLeafSize(0.05f,0.05f,0.05f);
    filter.filter(*cloud_filtered);

    //plane_fitting uncut region
    for (int index=0; index<cloud_filtered->size();index++)
    {
        if(cloud_filtered->points[index].x<-0.7)
        {
            cloud_filtered->points[index].r=100;
            cloud_filtered->points[index].g=100;
            cloud_filtered->points[index].b=0;
            plane_uncut->push_back(cloud_filtered->points[index]);//未收割作物平面
        }
    }
    vector<int> inliers_uncut;
    pcl::SampleConsensusModelPlane<pcl::PointXYZRGB>::Ptr model_uncut(new pcl::SampleConsensusModelPlane<pcl::PointXYZRGB>(plane_uncut));
    pcl::RandomSampleConsensus<pcl::PointXYZRGB> ransac_uncut (model_uncut);
    ransac_uncut.setDistanceThreshold(0.01);
    ransac_uncut.computeModel();
    ransac_uncut.getInliers(inliers_uncut);
    Eigen::VectorXf coeff_uncut;
    ransac_uncut.getModelCoefficients(coeff_uncut);
    pcl::copyPointCloud(*plane_uncut,inliers_uncut,*plane_uncut);

    Point2i  pointdepth;
    for(int row=250;row<rgb.rows;row+=4)
        for(int col=50;col<350;col++ )
        {
            float z = float(depth.at<ushort>(row,col))/1000;
            float y = (row - 232.171) * z / 615.312;
            float x = (col - 323.844) * z / 615.372;

            float distance=abs(coeff_uncut[0]*x+coeff_uncut[1]*y+coeff_uncut[2]*z+coeff_uncut[3]);
            if(distance<0.42 && distance>0.39)
            {
                pointdepth.x=col;
                pointdepth.y=row;
                pointdepthimg.push_back(pointdepth);
                Point_kd.x=col;
                Point_kd.y=row;
                Point_kd.z=1;
                cloud_kd->points.push_back(Point_kd);
                circle(rgb, pointdepth, 3, Scalar(100, 255, 100));
                Point.x=x;
                Point.y=y;
                Point.z=z;
                Point.r=255;
                Point.g=0;
                Point.b=0;
                cloudin->points.push_back(Point);
                continue;
            }
        }
        imshow("rgb",rgb);
        waitKey(1);

    cv_bridge::CvImage cvi;
    ros::Time time=ros::Time::now();
    cvi.header.stamp = time;
    cvi.header.frame_id = "image";
    cvi.encoding = "bgr8";
    cvi.image = rgb;
    sensor_msgs::Image im;
    cvi.toImageMsg(im);
    image_pub.publish(im);

}

void boud_RGB(Mat& rgb,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_kd)
{
    Mat rgb_kd=rgb.clone();
    Mat Gaussian, Binarization, close, open, ROIimg;

    GaussianBlur(rgb, Gaussian, Size(5, 5), 0.7);

    vector<Mat> channels;
    split(Gaussian, channels);
    Mat blue = channels.at(0);
    Mat green = channels.at(1);
    Mat red = channels.at(2);
    Binarization = 2 * red - green - blue;//2R-G-B超红模型特征
    Binarization = Binarization > 57;//阈值如何自适应选择??
    //imshow("Binarization",Binarization);

    Mat kernel_close = getStructuringElement(MORPH_ELLIPSE, Size(9, 9));
    morphologyEx(Binarization, close, MORPH_CLOSE, kernel_close);//闭运算,去除小型黑洞，先膨胀，再腐蚀
    Mat holesrc(close.size(),CV_8UC1,Scalar(255));
    Mat img1=holesrc(Rect(0,180,250,299));
    Mat img2=close(Rect(0,180,250,299));
    img1.copyTo(img2); //进一步处理？
    //imshow("close",close);

    int colsROI=0;
    vector<int> count255;
    for (int i = 0; i < close.cols; i++) {
        int count = 0;
        for (int j = 0; j < close.rows; j++) {
            if (int(close.at<uchar>(j,i))==0)
            count++;
        }
        if(count==480)
            colsROI=i;
    }

    Rect ROI(colsROI-150, 250, 200, 200);//设置ROI

    Mat mask = Mat::zeros(rgb.size(), CV_8UC1);
    mask(ROI).setTo(255);
    close.copyTo(ROIimg, mask);
    //imshow("ROIimg",ROIimg);


    Canny(ROIimg, ROIimg, 100, 200, 3);
   // imshow("canny",ROIimg);
    Mat verticalLine = get_Vertical(ROIimg);
    Mat horizontalLine = get_Horizontal(ROIimg);

    Mat vertical_Line_erode = erode_Img(ROIimg, verticalLine);
    Mat vertical_Line_dilate = dilate_Img(vertical_Line_erode, verticalLine);


    Mat horizontal_Line_erode = erode_Img(ROIimg, horizontalLine);
    Mat horizontal_Line_dilate = dilate_Img(horizontal_Line_erode, horizontalLine);

    subtract(ROIimg, vertical_Line_dilate, ROIimg, Mat(), -1);
    subtract(ROIimg, horizontal_Line_dilate, ROIimg, Mat(), -1);

    Mat kernel_open = getStructuringElement(MORPH_ELLIPSE, Size(1.5, 1.5));
    morphologyEx(ROIimg, ROIimg, MORPH_OPEN, kernel_open);

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud (cloud_kd);
    int k=1;
    vector<int>v_id(k);
    vector<float>v_dist(k);

    int count_before=0;
    int count_after=0;
    Point2i  pointrgb;
    for(int row=250;row<450;row+=6)
        for(int col=colsROI+50;col>colsROI-150;col--)
        {
            if(ROIimg.at<uchar>(row,col)==255)
            {
                Point_kd.x=col;
                Point_kd.y=row;
                Point_kd.z=1;
                count_before++;
                if(kdtree.nearestKSearch(Point_kd,k,v_id,v_dist)>0)
                {
                    if(v_dist[0]<100)
                    {
//                        if(v_dist[0]>36)
//                        {
//                            pointrgb.x=col+0.9*(cloud_kd->points[v_id[0]].x-col);
//                            pointrgb.y=row+0.9*(cloud_kd->points[v_id[0]].y-row);
//                            circle(rgb_kd, pointrgb, 3, Scalar(250,0, 0));
//                        }
//                        if(v_dist[0]>25 && v_dist[0]<36)
//                        {
//                            pointrgb.x=col+0.8*(cloud_kd->points[v_id[0]].x-col);
//                            pointrgb.y=row+0.8*(cloud_kd->points[v_id[0]].y-row);
//                            circle(rgb_kd, pointrgb, 3, Scalar(250,0, 0));
//                        }
//                        if(v_dist[0]>16 && v_dist[0]<25)
//                        {
//                            pointrgb.x=col+0.7*(cloud_kd->points[v_id[0]].x-col);
//                            pointrgb.y=row+0.7*(cloud_kd->points[v_id[0]].y-row);
//                            circle(rgb_kd, pointrgb, 3, Scalar(250,0, 0));
//                        }
//                        if(v_dist[0]>9 && v_dist[0]<16)
//                        {
//                            pointrgb.x=col+0.6*(cloud_kd->points[v_id[0]].x-col);
//                            pointrgb.y=row+0.6*(cloud_kd->points[v_id[0]].y-row);
//                            circle(rgb_kd, pointrgb, 3, Scalar(250,0, 0));
//                        }
//                        else
//                        {
                            count_after++;
                            pointrgb.x=col+0.3*(cloud_kd->points[v_id[0]].x-col);
                            pointrgb.y=row+0.3*(cloud_kd->points[v_id[0]].y-row);
                            circle(rgb_kd, pointrgb, 3, Scalar(250,0, 0));
                       // }

                    }
                }
                pointrgb.x=col;
                pointrgb.y=row;
                circle(rgb, pointrgb, 3, Scalar(0,0, 250));
                continue;
            }

        }

    //imshow("vertical",ROIimg);
    //imshow("rgb",rgb);
    imshow("rgb_depth_kd",rgb_kd);

    int key=cvWaitKey(100);
    if(key==13){
        imwrite("/home/wj/Desktop/rgb_1.png",rgb);
        imwrite("/home/wj/Desktop/rgb_kd_1.png",rgb_kd);
        cout<<"count_depth: "<<cloud_kd->points.size()<<endl;
        cout<<"count__rgb_before: "<<count_before<<endl;
        cout<<"count__rgb_after: "<<count_after<<endl;
    }

    waitKey(1);
}
