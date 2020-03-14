#include <iostream>
#include <string.h>
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
#include "height_border_msgs/height_border.h"


ros::Subscriber subRGBD;
ros::Publisher filter_pub;
ros::Publisher height_border_param;
image_transport::Publisher image_pub;
using namespace std;
using namespace cv;

void boud_depth(Mat& rgb,Mat& depth,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_kd);
void boud_RGB(Mat& rgb,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_kd);
void drawArrow(cv::Mat& img, cv::Point pStart, cv::Point pEnd, int len, int alpha,
    cv::Scalar& color, int thickness = 5, int lineType = 8);
int max_num(vector<float> height);

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
    ros::init(argc,argv,"boud_rgbd");
    ros::NodeHandle nh;
    image_transport::ImageTransport transport(nh);
    image_pub= transport.advertise("/boud_depth", 1);

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/realsense_sr300/ylx/rgb", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/realsense_sr300/ylx/depth", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);
    sync.registerCallback(boost::bind(&boud_RGBD,_1,_2));
    filter_pub = nh.advertise<sensor_msgs::PointCloud2>("height_depth", 1000);
    height_border_param= nh.advertise<height_border_msgs::height_border>("/height_border_param", 1);
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
    std_msgs::Header height_borderHeader;

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

        Vec4f line_para;
        fitLine(pointdepthimg, line_para, cv::DIST_L2, 0, 1e-2, 1e-2);
        Point2i point0;
        point0.x=line_para[2];
        point0.y=line_para[3];
        double k=line_para[1]/line_para[0];
        Point2i pStart;
        Point2i pEnd;
        pStart.x=(480-point0.y+k*point0.x)/k;
        pStart.y=480;
        pEnd.x=(380-point0.y+k*point0.x)/k;
        pEnd.y=380;

        Scalar lineColor(0, 0, 255);
        drawArrow(rgb, pStart, pEnd, 10, 45, lineColor);
        double PI=3.1415926;
        int zs=-atan(k)*180 / PI;
        int xs=int(abs(-atan(k)*180 / PI-zs)*10);//保留一位小数
        string yaw=to_string(90-zs)+'.'+to_string(10-xs);
//height detection
        vector< vector<Point2i> > mask_area;
        vector<Point2i> mask_points;
        mask_points.push_back(Point2i(431,478));
        mask_points.push_back(Point2i(407,395));
        mask_points.push_back(Point2i(445,390));
        mask_points.push_back(Point2i(482,471));
        mask_area.push_back(mask_points);

        cv::Mat mask, dst_mask;
        rgb.copyTo(mask);
        mask.setTo(cv::Scalar::all(0));
        fillPoly(mask, mask_area, Scalar(255, 255, 255));
        rgb.copyTo(dst_mask, mask);
        vector<float> height;
        for(int row=0;row<rgb.rows;row++)
            for(int col=0;col<rgb.cols;col++ )
            {
                if(dst_mask.at<Vec3b>(row,col)[0] != 0)
                {
                    float z = float(depth.at<ushort>(row,col))/1000;
                    float y = (row - 232.171) * z / 615.312;
                    float x = (col - 323.844) * z / 615.372;

                    float distanceheight=abs(coeff_uncut[0]*x+coeff_uncut[1]*y+coeff_uncut[2]*z+coeff_uncut[3])*100;
                    distanceheight=floor(distanceheight);
                    height.push_back(distanceheight);
                    Point.x=x;
                    Point.y=y;
                    Point.z=z;
                    Point.b=255;
                    cloudin->points.push_back(Point);
                }
            }

        sort(height.begin(),height.end());
        int height_reslut=max_num(height);
        
        height_border_msgs::height_border height_borderMsg;
        height_borderMsg.height=height_reslut;
        height_borderMsg.yaw=yaw;
        height_borderMsg.border_close_x=pStart.x;
        height_borderMsg.border_close_y=pStart.y;
        height_borderMsg.border_far_x=pEnd.x;
        height_borderMsg.border_far_y=pEnd.y;
        
        height_borderMsg.header = height_borderHeader;
        height_border_param.publish(height_borderMsg);


//        for(int row=0;row<rgb.rows;row++)
//            for(int col=0;col<rgb.cols;col++ )
//            {
//                if(dst_mask.at<Vec3b>(row,col)[0] != 0)
//                {
//                    float z = float(depth.at<ushort>(row,col))/1000;
//                    float y = (row - 232.171) * z / 615.312;
//                    float x = (col - 323.844) * z / 615.372;
//                    float distanceheight=abs(-0.0879711*x+ 0.986212*y+0.137248*z-1.95975)*100;
//                    distanceheight=floor(distanceheight);
//                    if(distanceheight==height_reslut)
//                    {
//                        Point.x=x;
//                        Point.y=y;
//                        Point.z=z;
//                        Point.b=0;
//                        Point.g=0;
//                        Point.r=255;
//                        cloudin->points.push_back(Point);
//                    }
//                }
//            }
//
//    sensor_msgs::PointCloud2 cloud_boud;
//    cloud_filtered->width=cloud_filtered->points.size();
//    pcl::toROSMsg(*cloudin+*plane_uncut,cloud_boud);
//    filter_pub.publish(cloud_boud);

//        string angle=std::to_string(zs)+'.'+std::to_string(xs);
//        cv::putText(rgb, angle, Point2i(450,90), cv::FONT_HERSHEY_SCRIPT_SIMPLEX,2, CV_RGB(255,0,0),7);
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

void drawArrow(cv::Mat& img, cv::Point pStart, cv::Point pEnd, int len, int alpha,
     cv::Scalar& color, int thickness, int lineType)
 {
     const double PI = 3.1415926;
     Point arrow;
    //计算 θ 角（最简单的一种情况在下面图示中已经展示，关键在于 atan2 函数，详情见下面）
     double angle = atan2((double)(pStart.y - pEnd.y), (double)(pStart.x - pEnd.x));
     line(img, pStart, pEnd, color, thickness, lineType);
     //计算箭角边的另一端的端点位置（上面的还是下面的要看箭头的指向，也就是pStart和pEnd的位置）
     arrow.x = pEnd.x + len * cos(angle + PI * alpha / 180);
     arrow.y = pEnd.y + len * sin(angle + PI * alpha / 180);
     line(img, pEnd, arrow, color, thickness, lineType);
     arrow.x = pEnd.x + len * cos(angle - PI * alpha / 180);
     arrow.y = pEnd.y + len * sin(angle - PI * alpha / 180);
     line(img, pEnd, arrow, color, thickness, lineType);
 }

int max_num(vector<float> height)
{
    int count =1;
    int sum_position=0;
    vector<int> element_num;
    for(int index=0;index<height.size()-1;index++)
    {
        if(height[index]==height[index+1])
        {
            count++;
            if(index+1==height.size()-1)
                element_num.push_back(count);
        }
        if(height[index]!=height[index+1])
        {
            element_num.push_back(count);
            count=1;
        }
    }
    int Position = max_element(element_num.begin(),element_num.end()) - element_num.begin();

    for(int i=0;i<=Position;i++)
    {

        sum_position+=element_num[i];
    }
    return height[sum_position-1];
}
