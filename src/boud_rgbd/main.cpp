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
#include <pcl/visualization/pcl_visualizer.h>
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
ros::Publisher pointcloud_pub;   //发布点云
ros::Publisher height_border_param;  //发布分界线,高度信息
image_transport::Publisher image_pub;
using namespace std;
using namespace cv;

void boud_depth(Mat& rgb,Mat& depth);
void drawArrow(cv::Mat& img, cv::Point pStart, cv::Point pEnd, int len, int alpha,
               cv::Scalar& color, int thickness = 5, int lineType = 8);
float max_num(vector<float> height);
Point3f random_rotateObject(double x, double y, double z, double a, double b, double c,double a0,double b0,double c0,double theta);
double DistanceOfPointToLine(Point3f a, Point3f b, Point3f s);
void boud_RGBD(const sensor_msgs::ImageConstPtr& rgbimg,const sensor_msgs::ImageConstPtr& depthimg);
void transform_to_pointcloud(Mat& rgb,Mat& depth,pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloudin);
void filter_pointcloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloudin,pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_filtered);
void uncutRegion_search(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_filtered,pcl::PointCloud<pcl::PointXYZRGB>::Ptr& plane_uncut);
Eigen::VectorXf boundpoints_clusterd(Mat& rgb,Mat& depth, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& plane_uncut,
                          vector<Point2i>& pointdepthimg,vector<Point3f>& pointdepthimg_3d);
void heigth_detection(Mat& rgb,Mat& depth,pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloudin,height_border_msgs::height_border& height_borderMsg,Eigen::VectorXf& coeff_uncut);
void boud_points_process(Mat& rgb,vector<Point2i>& pointdepthimg,vector<Point3f>& pointdepthimg_3d, height_border_msgs::height_border& height_borderMsg,pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloudin);
void curve_detect(Mat& rgb, vector<Point2i>& pointdepthimg);

void dis_cal(vector<float>& dis,vector<Point3f>& pointdepthimg_3d,Point3f begin_standard, Point3f end_standard);

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
    pointcloud_pub = nh.advertise<sensor_msgs::PointCloud2>("height_depth", 1000);
    height_border_param=nh.advertise<height_border_msgs::height_border>("/height_border_param", 1000);
    ros::spin();
    return 0;
}

void boud_depth(Mat& rgb, Mat& depth)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudin (new pcl::PointCloud<pcl::PointXYZRGB>);
    transform_to_pointcloud(rgb,depth,cloudin);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    filter_pointcloud(cloudin,cloud_filtered);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_uncut(new pcl::PointCloud<pcl::PointXYZRGB>);
    uncutRegion_search(cloud_filtered,plane_uncut);

    vector<Point3f> pointdepthimg_3d;
    vector<Point2i> pointdepthimg;
    Eigen::VectorXf coeff_uncut;
    coeff_uncut=boundpoints_clusterd(rgb,depth,cloudin,plane_uncut,pointdepthimg,pointdepthimg_3d);

    //float m=6.81;
    std_msgs::Header height_borderHeader;
    height_border_msgs::height_border height_borderMsg;
    height_borderMsg.header = height_borderHeader;
    boud_points_process(rgb,pointdepthimg,pointdepthimg_3d,height_borderMsg,cloudin);
    curve_detect(rgb,pointdepthimg);

    heigth_detection(rgb,depth,cloudin,height_borderMsg,coeff_uncut);

    cv_bridge::CvImage cvi;
    ros::Time time = ros::Time::now();
    cvi.header.stamp = time;
    cvi.header.frame_id = "image";
    cvi.encoding = "bgr8";
    cvi.image = rgb;
    sensor_msgs::Image im;
    cvi.toImageMsg(im);
    image_pub.publish(im);
    sensor_msgs::PointCloud2 cloud_boud;
    cloud_filtered->width = cloud_filtered->points.size();
    pcl::toROSMsg(*cloudin + *plane_uncut, cloud_boud);
    pointcloud_pub.publish(cloud_boud);
   // pcl::io::savePCDFile( "/home/wj/Desktop/pointcloud.pcd", *cloudin + *plane_uncut);
    imshow("rgb", rgb);
    waitKey(1);
}

void boud_RGBD(const sensor_msgs::ImageConstPtr& rgbimg,const sensor_msgs::ImageConstPtr& depthimg) {
//图像预处理，深度信息,颜色信息
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
        for(int col=0;col<rgb.cols;col++ ) {
            float z = float(depth_raw.at<float>(row, col)) * 1000;
            depth.at<ushort>(row, col) = z;

            Vec3b temp = rgb.at<Vec3b>(row, col);
            rgb.at<Vec3b>(row, col)[0] = temp[2];
            rgb.at<Vec3b>(row, col)[2] = temp[0];
        }
    boud_depth(rgb, depth);

}
void transform_to_pointcloud(Mat& rgb,Mat& depth,pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloudin)
{
    cloudin->header.frame_id="/frame";
    pcl::PointXYZRGB Point;
    for(int row=0;row<rgb.rows;row++)
        for(int col=0;col<rgb.cols;col++ )
        {
            float z = float(depth.at<ushort>(row,col))/1000;
            float y = (row - 232.171) * z / 615.312;
            float x = (col - 323.844) * z / 615.372;

            if(y>0 && z<10)  //根据相机坐标系,只选择向下的点云,z方向十米以内的点云;
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
}

void filter_pointcloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloudin,pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_filtered)
{
    pcl::VoxelGrid<pcl::PointXYZRGB> filter;
    filter.setInputCloud(cloudin);
    filter.setLeafSize(0.05f,0.05f,0.05f);
    filter.filter(*cloud_filtered);
}

//考虑检测未收割平面,点云分割
void uncutRegion_search(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_filtered,pcl::PointCloud<pcl::PointXYZRGB>::Ptr& plane_uncut)
{
    //设置mask:寻找未收割区域
    //pcl::PointXYZRGB Point;
//    vector<vector<Point2i> > mask_area_uncut;
//    vector<Point2i> mask_points_uncut;
//    mask_points_uncut.push_back(Point2i(475, 473));
//    mask_points_uncut.push_back(Point2i(448, 336));
//    mask_points_uncut.push_back(Point2i(616, 324));
//    mask_points_uncut.push_back(Point2i(628, 476));
//    mask_points_uncut.push_back(Point2i(475, 473));
//    mask_area_uncut.push_back(mask_points_uncut);
//
//    cv::Mat mask_uncut, dst_mask_uncut;
//    rgb.copyTo(mask_uncut);
//    mask_uncut.setTo(cv::Scalar::all(0));
//    fillPoly(mask_uncut, mask_area_uncut, Scalar(255, 255, 255));
//    rgb.copyTo(dst_mask_uncut, mask_uncut);
////    imshow("jjj",dst_mask_uncut);
////
//    for (int row = 0; row < rgb.rows; row+=3)
//        for (int col = 0; col < rgb.cols; col+=3) {
//            if (dst_mask_uncut.at<Vec3b>(row, col)[0] != 0) {
//                float z = float(depth.at<ushort>(row, col)) / 1000;
//                float y = (row - 232.171) * z / 615.312;
//                float x = (col - 323.844) * z / 615.372;
//                Point.x=x;
//                Point.y=y;
//                Point.z=z;
//                Point.r=255;
//                Point.g=0;
//                Point.b=0;
//                plane_uncut->push_back(Point);
//            }
//        }

    //plane_fitting uncut region
    for (int index=0; index<cloud_filtered->size();index++)
    {
        if(cloud_filtered->points[index].x>0.5 && cloud_filtered->points[index].z<7 && cloud_filtered->points[index].x<2.5) //未收割区域,DSACN选取或人工调整
        {
            cloud_filtered->points[index].r=0;
            cloud_filtered->points[index].g=255;
            cloud_filtered->points[index].b=0;
            plane_uncut->push_back(cloud_filtered->points[index]);//未收割作物平面
        }
    }
}

Eigen::VectorXf boundpoints_clusterd(Mat& rgb, Mat& depth,pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloudin,pcl::PointCloud<pcl::PointXYZRGB>::Ptr& plane_uncut,vector<Point2i>& pointdepthimg,vector<Point3f>& pointdepthimg_3d)
{
    vector<int> inliers_uncut;
    pcl::SampleConsensusModelPlane<pcl::PointXYZRGB>::Ptr model_uncut(new pcl::SampleConsensusModelPlane<pcl::PointXYZRGB>(plane_uncut));
    pcl::RandomSampleConsensus<pcl::PointXYZRGB> ransac_uncut (model_uncut);
    ransac_uncut.setDistanceThreshold(0.01);
    ransac_uncut.computeModel();
    ransac_uncut.getInliers(inliers_uncut);
    Eigen::VectorXf coeff_uncut;
    ransac_uncut.getModelCoefficients(coeff_uncut);
    pcl::copyPointCloud(*plane_uncut,inliers_uncut,*plane_uncut);

    if(coeff_uncut[3]<0)
    {
        coeff_uncut[0]=-coeff_uncut[0];
        coeff_uncut[1]=-coeff_uncut[1];
        coeff_uncut[2]=-coeff_uncut[2];
        coeff_uncut[3]=-coeff_uncut[3];
    }
    //cout<<coeff_uncut[0]<<" "<<coeff_uncut[1]<<" "<<coeff_uncut[2]<<" "<<coeff_uncut[3]<<endl;

    Point3f  pointdepth_3d; //分界线点的三维坐标
    Point2i  pointdepth; //分界线点的二维坐标

    float A=0.022391413,B= -0.991479484,C= -0.127744922,D= 1.79071477;  //偏差基准面

    pcl::PointXYZRGB Point;

    for(int row=depth.rows-1;row>=320;row-=3)  //ROI遍历范围
        for(int col=250;col<540;col++ )
        {
            float z = float(depth.at<ushort>(row,col))/1000;
            float y = (row - 232.171) * z / 615.312;
            float x = (col - 323.844) * z / 615.372;

            float distance=abs(A*x+B*y+C*z+D);
            if(distance<0.5 && distance>0.4) //距离阈值判断
            {
                pointdepth.x=col;
                pointdepth.y=row;
                int n=pointdepthimg.size();
                if(n>5)
                {
                    if(abs(pointdepth.x-pointdepthimg[n-1].x)>10 || abs(pointdepth.x-pointdepthimg[n-2].x)>20)  //外点需要改进
                        continue;
                }

                //三维分界线点投影基准平面上
                pointdepth_3d.x=((B*B+C*C)*x-A*(B*y+C*z+D));
                pointdepth_3d.y=((A*A+C*C)*y-B*(A*x+C*z+D));
                pointdepth_3d.z=((A*A+B*B)*z-C*(A*x+B*y+D));

                pointdepthimg_3d.push_back(pointdepth_3d);
                pointdepthimg.push_back(pointdepth);
                circle(rgb, pointdepth, 3, Scalar(100, 255, 100));
                Point.x=pointdepth_3d.x;
                Point.y=pointdepth_3d.y;
                Point.z=pointdepth_3d.z;
                Point.r=255;
                Point.g=0;
                Point.b=0;
                cloudin->points.push_back(Point); //三维点云显示
            }
        }
    return coeff_uncut;
}

void boud_points_process(Mat& rgb,vector<Point2i>& pointdepthimg,vector<Point3f>& pointdepthimg_3d, height_border_msgs::height_border& height_borderMsg,pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloudin) {
    if (pointdepthimg.size() > 20) {

        //画出标准分界线
        float Standard_2Dline_fun_0=0.176318, Standard_2Dline_fun_1=0.984333,Standard_2Dline_fun_2= 379.282,Standard_2Dline_fun_3= 420.846; //二维平面标准分界线
        Point2i point0_Standard;
        point0_Standard.x = Standard_2Dline_fun_2;
        point0_Standard.y = Standard_2Dline_fun_3;
        double k_Standard = Standard_2Dline_fun_1 / Standard_2Dline_fun_0;
        Point2i pStart_Standard;
        Point2i pEnd_Standard;
        pStart_Standard.x = (480 - point0_Standard.y + k_Standard * point0_Standard.x) / k_Standard;
        pStart_Standard.y = 480;
        pEnd_Standard.x = (380 - point0_Standard.y + k_Standard * point0_Standard.x) / k_Standard;
        pEnd_Standard.y = 380;
        Scalar lineColor_Standard(0, 0, 255);
        drawArrow(rgb, pStart_Standard, pEnd_Standard, 10, 45, lineColor_Standard);

        Vec4f line_para;
        fitLine(pointdepthimg, line_para, cv::DIST_WELSCH, 0, 1e-2, 1e-2);
        //判断往左偏or右偏
        Point2i vec_bound_standard,vec_bound_cur;
        vec_bound_standard.x=pEnd_Standard.x-pStart_Standard.x;
        vec_bound_standard.y=pEnd_Standard.y-pStart_Standard.y;
        vec_bound_cur.x=(380 - line_para[3] + (line_para[1]/line_para[0]) * line_para[2]) / (line_para[1]/line_para[0])-pStart_Standard.x;
        vec_bound_cur.y=380-pStart_Standard.y;
        bool left=(vec_bound_standard.x*vec_bound_cur.y-vec_bound_cur.x*vec_bound_standard.y)>0; //left==true,则往左偏,否则往右

        Vec6f line_para_3d;
        fitLine(pointdepthimg_3d, line_para_3d, cv::DIST_WELSCH, 0, 1e-2, 1e-2);

        //cout<<line_para_3d[0]<<" "<<line_para_3d[1]<<" "<<line_para_3d[2]<<" "<<line_para_3d[3]<<" "<<line_para_3d[4]<<" "<<line_para_3d[5]<<endl;

        float Standard_line_fun_0 = 0.00251082, Standard_line_fun_1 =-0.127729, Standard_line_fun_2= 0.991806,  //分界线标准线
              Standard_line_fun_3= 0.451316,Standard_line_fun_4= 1.08251, Standard_line_fun_5=5.69316;

        if(line_para_3d[2]<0)  //调整矢量方向一致,避免余弦角计算1变179问题
        {
            line_para_3d[0]=-line_para_3d[0];
            line_para_3d[1]=-line_para_3d[1];
            line_para_3d[2]=-line_para_3d[2];
        }

        //计算分界线夹角
        float cosvalue = line_para_3d[0] * Standard_line_fun_0 +  line_para_3d[1] * Standard_line_fun_1 + line_para_3d[2] * Standard_line_fun_2;
        float arc_cosvalue_inangle = acos(cosvalue);
        if(!left) arc_cosvalue_inangle=-arc_cosvalue_inangle;

        float A_standard_plane=0.022391413,B_standard_plane= -0.991479484,C_standard_plane= -0.127744922,D_standard_plane= 1.79071477; //该平面为偏差基准面

        Point3f begin_standard = Point3f(Standard_line_fun_3, Standard_line_fun_4, Standard_line_fun_5);
        Point3f end_standard = Point3f(Standard_line_fun_3+Standard_line_fun_0, Standard_line_fun_4+Standard_line_fun_1, Standard_line_fun_5+Standard_line_fun_2);
        int distance=0;
        if(abs(arc_cosvalue_inangle*180/3.1415926)<2.5)
        {
            vector<float> dis;
            dis_cal(dis,pointdepthimg_3d,begin_standard,end_standard);
            sort(dis.begin(), dis.end());
            float dis_reslut = max_num(dis);
            distance=dis_reslut*100;
            if(line_para_3d[3]<end_standard.x) distance=-distance;
        }
        else
        {
            vector<Point3f> rotate_point;
            for(int i=0;i<pointdepthimg_3d.size();i++)
            {
                Point3f tmp_rotate_point = random_rotateObject( pointdepthimg_3d[i].x,pointdepthimg_3d[i].y,pointdepthimg_3d[i].z, 0, 0, 0,
                                                                A_standard_plane,B_standard_plane,C_standard_plane+0.03,
                                                               arc_cosvalue_inangle);
                rotate_point.push_back(tmp_rotate_point);
            }

//            Vec6f line_para_3d1;
//            fitLine(rotate_point, line_para_3d1, cv::DIST_WELSCH, 0, 1e-2, 1e-2);
//            cout<<line_para_3d1[0]*Standard_line_fun_0+line_para_3d1[1]*Standard_line_fun_1+line_para_3d1[2]*Standard_line_fun_2<<endl;

            pcl::PointXYZRGB tmp;
            for(int i=0;i<rotate_point.size();i++)
            {
                tmp.x=rotate_point[i].x;
                tmp.y=rotate_point[i].y;
                tmp.z=rotate_point[i].z;
                tmp.g=255;
                cloudin->points.push_back(tmp);
            }

            vector<float> dis;
            dis_cal(dis,rotate_point,begin_standard,end_standard);
            sort(dis.begin(), dis.end());
            float dis_reslut = max_num(dis);
            distance=dis_reslut*100;
            if(line_para_3d[3]<end_standard.x) distance=-distance;
        }

        Point2i point0;
        point0.x = line_para[2];
        point0.y = line_para[3];
        double k = line_para[1] / line_para[0];
        Point2i pStart;
        Point2i pEnd;
        pStart.x = (480 - point0.y + k * point0.x) / k;
        pStart.y = 480;
        pEnd.x = (380 - point0.y + k * point0.x) / k;
        pEnd.y = 380;


        Scalar lineColor(0, 255, 0);
        drawArrow(rgb, pStart, pEnd, 10, 45, lineColor);
        int zs = arc_cosvalue_inangle*180/3.1415926;
        int xs = abs(int((arc_cosvalue_inangle*180/3.1415926 - zs) * 10));//保留一位小数
        string angle = to_string(zs) + '.' + to_string(xs);
        height_borderMsg.angle = angle;
        height_borderMsg.dis = to_string(distance);

        //仿射变换
        Point2f raw_perceptive[4];
        raw_perceptive[0]=(Point2i(220,209));
        raw_perceptive[1]=(Point2i(420,209));
        raw_perceptive[2]=(Point2i(596,477));
        raw_perceptive[3]=(Point2i(-43,477));

        Point2f target_perceptive[4];
        target_perceptive[0]=(Point2i(220,209));
        target_perceptive[1]=(Point2i(420,209));
        target_perceptive[2]=(Point2i(420,477));
        target_perceptive[3]=(Point2i(220,477));

        Mat transmtx = getPerspectiveTransform(raw_perceptive,target_perceptive);
        cv::warpPerspective(rgb, rgb, transmtx, rgb.size());

        if(arc_cosvalue_inangle<40 && arc_cosvalue_inangle>-40 && distance<70 && distance>-70)
        {
            float value_inangle = arc_cosvalue_inangle * 180 / 3.1415926;
            int xs_value_inangle = (value_inangle - int(value_inangle)) * 10;//保留一位小数
            string angle =
                    "Ang: " + std::to_string(int(value_inangle)) + '.' + std::to_string(abs(xs_value_inangle));
            cv::putText(rgb, angle, Point2i(400, 50), cv::FONT_HERSHEY_SIMPLEX, 1, CV_RGB(255, 0, 0), 4);

            int zs_distance = distance;
            string dis = "Dis: " + std::to_string(zs_distance);
            cv::putText(rgb, dis, Point2i(400, 100), cv::FONT_HERSHEY_SIMPLEX, 1, CV_RGB(255, 0, 0), 5);
        }
    }
}

void curve_detect(Mat& rgb,vector<Point2i>& pointdepthimg) {
    if (pointdepthimg.size() > 20) {
        int path_points_size = pointdepthimg.size();
        vector<Point2i> pointdepthimg_last(pointdepthimg.begin() + path_points_size / 2,
                                           pointdepthimg.end());//路径点后半段
        vector<Point2i> pointdepthimg_before(pointdepthimg.begin(),
                                             pointdepthimg.begin() + path_points_size / 2);//路径点前半段

        Vec4f line_para_last;
        fitLine(pointdepthimg_last, line_para_last, cv::DIST_L2, 0, 1e-2, 1e-2);
        Vec4f line_para_before;
        fitLine(pointdepthimg_before, line_para_before, cv::DIST_L2, 0, 1e-2, 1e-2);

        double k_before = line_para_before[1] / line_para_before[0];
        Point2i point0_before;
        point0_before.x = line_para_before[2];
        point0_before.y = line_para_before[3];

        double k_last = line_para_last[1] / line_para_last[0];
        Point2i point0_last;
        point0_last.x = line_para_last[2];
        point0_last.y = line_para_last[3];


        double PI = 3.1415926;
        int angle_before = atan(k_before) * 180 / PI;
        int angle_last = atan(k_last) * 180 / PI;

        if (angle_before < 0)
            angle_before = 180 + angle_before;
        if (angle_last < 0)
            angle_last = 180 + angle_last;
        if (abs(angle_before - angle_last) > 45) {
            Point2i point_curve;
            point_curve.y =
                    (-k_before * point0_last.y + k_before * k_last * point0_last.x + k_last * point0_before.y -
                     k_before * k_last * point0_before.x) / (k_last - k_before);
            point_curve.x = (point_curve.y - point0_before.y + k_before * point0_before.x) / k_before;
            circle(rgb, point_curve, 10, Scalar(255, 0, 0), -1);
        }
    }
}

//绘制方向箭头
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

 //计算作物高度
float max_num(vector<float> height)
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

//旋转当前分界线上一点, 使当前分界线平行于正确行驶时的分界线,进而求得横向误差
Point3f random_rotateObject(double x, double y, double z, double a, double b, double c,double a0,double b0,double c0,double theta)
{
    //(a0,b0,c0)轴线的始点
    //(a,b,c)轴线L末点
    //(x,y,z)初始点位置
    //点Point3f 绕L旋转后的位置
    double u = a0-a;
    double v = b0-b;
    double w = c0-c;  //方向向量(u,v,w)需为单位向量！！！
    Point3f p;
    double SinA = sin(theta);
    double CosA = cos(theta);

    double uu=u*u;
    double vv=v*v;
    double ww=w*w;
    double uv=u*v;
    double uw=u*w;
    double vw=v*w;

    float t00 = uu + (vv + ww) * CosA;
    float t10 = uv * (1 - CosA) + w * SinA;
    float t20 = uw * (1 - CosA) - v * SinA;
    float t30 = 0;

    float t01 = uv * (1 - CosA) - w * SinA;
    float t11 = vv + (uu + ww) * CosA;
    float t21 = vw * (1 - CosA) + u * SinA;
    float t31 = 0;

    float t02 = uw * (1 - CosA) + v * SinA;
    float t12 = vw * (1 - CosA) - u * SinA;
    float t22 = ww + (uu + vv) * CosA;
    float t32 = 0;

    float t03 = (a * (vv + ww) - u * (b * v + c * w)) * (1 - CosA) + (b * w - c * v) * SinA;
    float t13 = (b * (uu + ww) - v * (a * u + c * w)) * (1 - CosA) + (c * u - a * w) * SinA;
    float t23 = (c * (uu + vv) - w * (a * u + b * v)) * (1 - CosA) + (a * v - b * u) * SinA;
    float t33 = 1;

    p.x = t00 * x + t01 * y + t02 * z + t03;
    p.y = t10 * x + t11 * y + t12 * z + t13;
    p.z = t20 * x + t21 * y + t22 * z + t23;
    return p;
}

//空间点到直线距离
double DistanceOfPointToLine(Point3f a, Point3f b, Point3f s)
{
    double ab = sqrt(pow((a.x - b.x), 2.0) + pow((a.y - b.y), 2.0) + pow((a.z - b.z), 2.0));
    double as = sqrt(pow((a.x - s.x), 2.0) + pow((a.y - s.y), 2.0) + pow((a.z - s.z), 2.0));
    double bs = sqrt(pow((s.x - b.x), 2.0) + pow((s.y - b.y), 2.0) + pow((s.z - b.z), 2.0));
    double cos_A = (pow(as, 2.0) + pow(ab, 2.0) - pow(bs, 2.0)) / (2 * ab*as);
    double sin_A = sqrt(1 - pow(cos_A, 2.0));
    return as*sin_A;
}

void dis_cal(vector<float>& dis,vector<Point3f>& pointdepthimg_3d,Point3f begin_standard, Point3f end_standard)
{
    Point3f s;Point3f a=begin_standard; Point3f b=end_standard;
    for(int i=0;i<pointdepthimg_3d.size();i++)
    {
        s=pointdepthimg_3d[i];
        double ab = sqrt(pow((a.x - b.x), 2.0) + pow((a.y - b.y), 2.0) + pow((a.z - b.z), 2.0));
        double as = sqrt(pow((a.x - s.x), 2.0) + pow((a.y - s.y), 2.0) + pow((a.z - s.z), 2.0));
        double bs = sqrt(pow((s.x - b.x), 2.0) + pow((s.y - b.y), 2.0) + pow((s.z - b.z), 2.0));
        double cos_A = (pow(as, 2.0) + pow(ab, 2.0) - pow(bs, 2.0)) / (2 * ab*as);
        double sin_A = sqrt(1 - pow(cos_A, 2.0));
        dis.push_back(as*sin_A);
    }
}



void heigth_detection(Mat& rgb,Mat& depth,pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloudin,height_border_msgs::height_border& height_borderMsg,Eigen::VectorXf& coeff_uncut)
{
    //height detection

        vector<float> height;
        pcl::PointXYZRGB Point;

        float A=0.022391413,B= -0.991479484,C= -0.127744922,D= 1.79071477;

        for (int row = 400; row < depth.rows; row++)
            for (int col = 200; col < 400; col++) {
                    float z = float(depth.at<ushort>(row, col)) / 1000;
                    float y = (row - 232.171) * z / 615.312;
                    float x = (col - 323.844) * z / 615.372;

                    float distanceheight = abs(A * x + B * y + C * z + D) *100;
                    if(distanceheight>80) {
                        distanceheight = floor(distanceheight);
                        height.push_back(distanceheight);
                        Point.x = x;
                        Point.y = y;
                        Point.z = z;
                        Point.b = 255;
                        cloudin->points.push_back(Point);
                    }
            }

        sort(height.begin(), height.end());
        int height_reslut = max_num(height);

        height_borderMsg.height = height_reslut;
        height_border_param.publish(height_borderMsg);

    for(int row=300;row<depth.rows;row++)
        for(int col=400;col<550;col++ )
        {
            float z = float(depth.at<ushort>(row,col))/1000;
            float y = (row - 232.171) * z / 615.312;
            float x = (col - 323.844) * z / 615.372;

            float distanceheight = abs(A * x + B * y + C * z + D) *100;
            distanceheight=floor(distanceheight);
            if(distanceheight==height_reslut)
            {
                Point.x = x;
                Point.y = y;
                Point.z = z;
                Point.b = 0;
                Point.g = 0;
                Point.r = 255;
                cloudin->points.push_back(Point);
            }
        }

}

