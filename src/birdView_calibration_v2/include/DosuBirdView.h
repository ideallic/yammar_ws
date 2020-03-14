/*
This is C++ version of DosuBirdView package, which is used for Bird
eye image stitch with 4 RGB cameras.
Notice: any question please contact DOSU ${cbbsjtu@126.com}
*/

#ifndef DOSUBIRDVIEW_H
#define DOSUBIRDVIEW_H

//support sys
#include <iostream>
#include <string>
#include <vector>
//ros
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
//#include <geometry_msgs/Pose2D.h>
//#include <geometry_msgs/PoseWithCovarianceStamped.h>
//#include <tf/transform_datatypes.h>
#include <ros/package.h>
//boost
#include <boost/program_options.hpp>
//opencv
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
//srvs
#include "dosuss_srvs/find_cut.h"

using namespace std;
using namespace cv;
//***if use commad line input please uncommit it***
namespace bpo = boost::program_options;

// calculate correspondence point for every input
// 0: left up  1: right up  2:rigth down  3: left down

namespace DosuBird
{
	//define BirdView
	class BirdView
	{
	public:
		BirdView(bool click_points, const char* configFile = NULL, const char* carImg_dir = NULL
			       )
		{
			SourcePoint_OK=ParamSet_OK = false;
			maskHeigth = clickCount = camID = 0;
			targetPoint.resize(4);
			sourcePoint.resize(4);
	    try
	    {
			 	carPic = imread(carImg_dir,CV_8UC4);
	    }
	    catch (...)
	    {
				std::cout <<"[WARNING] Car model view pic not found!\n";
	    }
			for (int i = 0;i < 4; i++)
			{
				targetPoint[i].resize(4);
				sourcePoint[i].resize(4);
			}
			// check if config file exist.
			if (configFile)
			{
				readConfig(configFile);
			}
		}

		void setPath(const string& extri,
								   const string& target,
								   const string& car)
		{
			extri_src = extri;
			target_src = target;
			car_src = car;
		}

		void setInternalShift(int W, int H)
		{
			ShiftAdjust = Size(W,H);
			ParamSet_OK = false;
			//setParam();
		}
		void setShift(int W, int H)
		{
			Shift = Size(W,H);
			ParamSet_OK = false;
			// setParam();
		}
		void setCarSize(int W,int H)
		{
			carSize = Size(W, H);
			ParamSet_OK = false;
			// setParam();
		}
		void setCarShift(int H)
		{
			carShift = H;
			ParamSet_OK = false;
			setParam();
		}
		void setChessSize(int W, int H)
		{
			chessBordWidth.width = W;
	    chessBordWidth.height = H;
			ParamSet_OK = false;
			// setParam();
		}
		void setMaskHeigth(int maskHeigth_)
		{
			maskHeigth = maskHeigth_;
			ParamSet_OK = false;
			// setParam();
		}
	  Mat transformView(Mat* v)
	  {
	        if (!SourcePoint_OK)
	        {
	            std::cerr<<"[ERROR] Source Points have not been pointed! please Add function sourcePointClick to get Source Points!\n";
	            throw "[ERROR] Source Points have not been pointed! please Add function sourcePointClick to get Source Points!\n";
	        }
	        if (!ParamSet_OK)
	        {
	            setParam();
	        }

	        Mat b[4];
	        Mat m = Mat(mSize, CV_8UC3 );

	        //int seq[4] = { 0,1,2,3 };
	        for (int i = 0; i < 4; i++)
	        {
	            if(!v[i].data)
	            {
	                continue;
	            }
	            warpPerspective(v[i], b[i], Birdtransform[i], mSize);
				//	 	  if(i==3){
					//		imshow("test",b[i]);
							Mat mask = b[i](r[i]).clone();
							b[i](r[i]).copyTo(m(r[i]),mask);
					//		}

	        }

	 				// Rotate the image
					int iAngle = 184;
					int iImageHeight = m.rows / 2;
					int iImageWidth = m.cols / 2;
					Mat matRotation = getRotationMatrix2D( Point(iImageWidth, iImageHeight), (iAngle - 180), 1 );
				  Mat m_rotated;
				  warpAffine( m, m_rotated, matRotation, m.size());

					//cut useless edge
				  Mat m_1 = m_rotated(Rect(80,80,mSize.width-160,mSize.height-160)).clone();
				  // Mat cut=imread(car_src);
		 	 	  // Mat cut_temp;
					// int a =250;
		 		  // resize(cut,cut_temp,Size(m_1.cols-2*a,600),CV_INTER_CUBIC);
		 		  // cut_temp.copyTo(m_1(Rect(a,600,m_1.cols-2*a,600)));

	        if(carPic.data) carPicTmp.copyTo(m(carPicPos));
	        //drawing target points
	        const Scalar color[4] = { Scalar(255,0,0),Scalar(0,255,0), Scalar(255,255,0), Scalar(0,255,255) };
	        for (int i = 0; i < 16; i++) circle(m, targetPoint[i / 4][i % 4], 5, color[i / 4], 5);
	        return m_1;
	    }
	    void saveConfig(const char* configFile = "config.xml")
	    {
	        for (int i = 0;i < 4; i++)
	        {
	            if (sourcePoint[i].empty())
	            {
	                std::cout << "[ERROR] sourcePoint has not been comfired all\n"<<std::endl;
	                return ;
	            }
	        }
	        FileStorage fs(configFile, FileStorage::WRITE);
	        if (fs.isOpened())
	        {
	            for (int i = 0; i < 4; i++)
	            {
	                for (int k = 0; k < 4; k++)
	                {
	                    char buf[20];
	                    sprintf(buf, "sourcePoint%d%d", i, k);
	                    write(fs, buf, sourcePoint[i][k]);
	                }
	            }
	            fs.release();
	            std::cout << "\n extrinsic save complete! \n\n";
	        }
	    }
			void saveTarget(const char* targetFile = "target.xml")
		 {
				 for (int i = 0;i < 4; i++)
				 {
						 if (targetPoint[i].empty())
						 {
								 std::cout << "[ERROR] sourcePoint has not been comfired all\n"<<std::endl;
								 return ;
						 }
				 }
				 FileStorage fs(targetFile, FileStorage::WRITE);
				 if (fs.isOpened())
				 {
					   char buf0[20];
					   sprintf(buf0, "mSize");
					 	 write(fs, buf0, mSize);
						 for (int i = 0; i < 4; i++)
						 {
								 for (int k = 0; k < 4; k++)
								 {
										 char buf[20];
										 sprintf(buf, "targetPoint%d%d", i, k);
										 write(fs, buf, targetPoint[i][k]);
								 }
						 }
						 fs.release();
						 std::cout << "\n target save complete! \n\n";
				 }
		 }
	    void readConfig(const char* configFile = "config.xml")
	    {
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
	            SourcePoint_OK = true;  // source point reading completed
	            ParamSet_OK = false; // setting parma
	            //setParam();
	            cout << "Config file read sucessfully!" << endl;
	        }
	        else{
						cout << "[ERROR] There is not such file named config.xml" << endl;
						cout << "please refer to -> Usage: ./bin/xxx --get_Points"<< endl;
						exit(-1);
					}
	    }
	    void sourcePointClick(Mat *v)
	    {
	        //setParam(1);
	        // click corner-points and record them
	        printf("cam: %d ,pointID: %d  ", camID, clickCount);
	        const char *windowsName = "Source point set";
	        namedWindow(windowsName);
	        setMouseCallback(windowsName,on_MouseHandle, (void*)this);
	        for(int i=0;i<4;i++)
	        {
	            for(int j=0;j<4;j++)
	            {
	                sourcePoint[i][j]= Point2f(0,0);
	            }
	        }

	        for (camID = 0, clickCount = 0; camID<4;)
	        {
	            for (int i = 0; i < sourcePoint[camID].size(); i++)
	            {
	                circle(v[camID], sourcePoint[camID][i], 5, Scalar(255, 0, 0), 2);
	            }
	            imshow(windowsName, v[camID]);
	            if (waitKey(20) == 'j')	break;
	        }
	        setMouseCallback(windowsName, NULL, NULL);
	        destroyWindow(windowsName);
	        saveConfig(extri_src.c_str());/*save source's points*/
	        SourcePoint_OK = true;
	    }
	    void sourcePointClick(cv::VideoCapture *v)
	    {
	        //setParam(1);
	        Mat ans;
	        // click corner-points and record them
	        printf("cam: %d ,pointID: %d  ", camID, clickCount);
	        const char *windowsName = "Source point set";
	        namedWindow(windowsName);
	        setMouseCallback(windowsName,on_MouseHandle, (void*)this);
	        for(int i=0;i<4;i++)
	        {
	            sourcePoint[i].clear();
	        }
	        for (camID = 0, clickCount = 0; camID<4;)
	        {
	            v[camID] >> ans;
	            for (int i = 0; i < sourcePoint[camID].size(); i++)
	            {
	                circle(ans, sourcePoint[camID][i], 5, Scalar(255, 0, 0), 2);
	            }
	            imshow(windowsName, ans);
	            if (waitKey(20) == 'j')	break;
	        }
	        setMouseCallback(windowsName, NULL, NULL);
	        destroyWindow(windowsName);
	        saveConfig(extri_src.c_str());/*save source's points*/
	        SourcePoint_OK = true;
	    }
	    static void on_MouseHandle(int e, int x, int y, int flag, void* param)
	    {
	        BirdView & birdView = *(BirdView*)param;
	        int camID = birdView.camID;
	        switch (e)
	        {
	            case EVENT_LBUTTONUP:
	            {
	                birdView.sourcePoint[birdView.camID][birdView.clickCount] = Point2f(x, y)	;
	                printf("x:%d y:%d\n", x, y);
	                birdView.clickCount++;
	                if (birdView.clickCount> 3)
	                {
	                    birdView.clickCount = 0;
	                    birdView.Birdtransform[camID] = getPerspectiveTransform(birdView.sourcePoint[camID], birdView.targetPoint[camID]);
	                    birdView.camID++;
	                }
	                if (birdView.camID<3)
	                {
	                    printf("cam: %d ,pointID: %d  ", birdView.camID, birdView.clickCount);
	                }
	                else printf("\n");
	            }
	            default: break;
	        }
	    }

	private:
		Rect r[4],carPicPos;
		int clickCount, camID, maskHeigth, carShift;
		Mat Birdtransform[4],maskF, maskB, carPic,carPicTmp;
		vector<vector<Point2f> > targetPoint, sourcePoint;
		Size ShiftAdjust, Shift, chessBordWidth, mSize, carSize;
		bool SourcePoint_OK,ParamSet_OK;

		string target_src, extri_src, car_src;

		//string configFile, targetFile;

		void setParam(bool tranformCheck = false)
	  {
	        //// WARMING will show when Transform is running but not all parameters have been set
	        if (Shift.area()== 0)
	        {
	            if (tranformCheck)std::cout << "[WARMING] Shift has not been set! Default value will be used" << std::endl;
	            Shift.width = Shift.height = 200;
	        }
	        if (chessBordWidth.area() == 0)
	        {
	            if (tranformCheck)std::cout << "[WARMING] chessBordWidth has not been set! Default value will be used" << std::endl;
	            chessBordWidth.width = chessBordWidth.height = 60;
	        }
	        if (ShiftAdjust.area() == 0)
	        {
	            if (tranformCheck)std::cout << "[WARMING] ShiftAdjust has not been set! Default value will be used" << std::endl;
	            ShiftAdjust.width = 36;
	            ShiftAdjust.height = 27;
	        }
	        if (carSize.area() == 0)
	        {
	            if (tranformCheck)std::cout << "[WARMING] carSize has not been set! Default value will be used" << std::endl;
	            carSize = Size(240, 380);
	        }
					if (carShift == 0)
					{
							if (tranformCheck)std::cout << "[WARMING] carShift has not been set! Default value will be used" << std::endl;
							carShift = 200;
					}
	        if (maskHeigth >=100 || maskHeigth <=0)
	        {
	            if (tranformCheck)std::cout << "[WARMING] maskHeigth has not been set! Default value will be used" << std::endl;
	            maskHeigth = 200;
	        }
	        if (!ParamSet_OK)
	        {
	            /*The size of the entire output image*/
	            mSize = Size(Shift.width * 2 + ShiftAdjust.width + chessBordWidth.width * 2,
	                         Shift.height * 2 + ShiftAdjust.height + chessBordWidth.height * 2);
	            /*make targetPoint, need chessBordWidth,mSize,Shift*/
	            /*forward*/
	            targetPoint[0][0] = (Point2f(Shift.width + chessBordWidth.width, Shift.height));
	            targetPoint[0][1] = (Point2f(Shift.width + chessBordWidth.width + ShiftAdjust.width +0+10, Shift.height));
	            targetPoint[0][2] = (Point2f(Shift.width + chessBordWidth.width + ShiftAdjust.width+0+10, Shift.height + chessBordWidth.height));
	            targetPoint[0][3] = (Point2f(Shift.width + chessBordWidth.width, Shift.height + chessBordWidth.height));

	            /*left*/
	            targetPoint[1][0] = (Point2f(Shift.width + 20.0+0+5, mSize.height - chessBordWidth.height - Shift.height));
	            targetPoint[1][1] = (Point2f(Shift.width + 20.0, Shift.height + 100.0 + chessBordWidth.height+0-13));
	            targetPoint[1][2] = (Point2f(Shift.width + chessBordWidth.width -20.0, Shift.height + chessBordWidth.height + 100.0+0-13));
	            targetPoint[1][3] = (Point2f(Shift.width + chessBordWidth.width -20.0+0+5, mSize.height - chessBordWidth.height - Shift.height));

	            /*backward*/
	            targetPoint[2][0] = (Point2f(mSize.width - chessBordWidth.width - Shift.width, mSize.height - Shift.height));
	            targetPoint[2][1] = (Point2f(Shift.width + chessBordWidth.width, mSize.height - Shift.height));
	            targetPoint[2][2] = (Point2f(Shift.width + chessBordWidth.width, mSize.height - Shift.height - chessBordWidth.height + 40));
	            targetPoint[2][3] = (Point2f(mSize.width - chessBordWidth.width - Shift.width, mSize.height - Shift.height - chessBordWidth.height +40));
	            /*right*/
	            targetPoint[3][0] = (Point2f(mSize.width - Shift.width -20+0+4, Shift.height + chessBordWidth.height +100 + 0-10));
	            targetPoint[3][1] = (Point2f(mSize.width - Shift.width -20, mSize.height - Shift.height - chessBordWidth.height+0+30));
	            targetPoint[3][2] = (Point2f(mSize.width - Shift.width - chessBordWidth.width +20, mSize.height - Shift.height - chessBordWidth.height+0+30));
	            targetPoint[3][3] = (Point2f(mSize.width - Shift.width - chessBordWidth.width +20+0+10, Shift.height + chessBordWidth.height +100+0-10));

	            //need  Shift, chessBordWidth, ShiftAdjust, mSize
	            /*roi*/
	            r[0] = Rect(0, 0, mSize.width, carShift);
							r[1] = Rect(0, 0, int((mSize.width - carSize.width) / 2.0), mSize.height);
							if((mSize.height - carSize.height - carShift)<=0)
							{
								cout<<"Error::carShift is too big -> please reduce the value"<<endl;
								exit(1);
							}
	            r[2] = Rect(0, carSize.height + carShift, mSize.width, mSize.height - carSize.height - carShift);
	            r[3] = Rect(int((mSize.width + carSize.width) / 2.0), 0, int((mSize.width - carSize.width) / 2.0), mSize.height);

	            // maskF = Mat(r[1].size(), CV_8UC1, Scalar(1));
							//
	            // maskB = Mat(r[1].size(), CV_8UC1, Scalar(1));
							//
	            // /*make mask, need r */
	            // vector<vector<Point> > maskVec;
	            // /*forward*/
	            // maskVec.push_back(vector<Point>());
	            // maskVec[0].push_back(Point(0, r[1].height));
	            // maskVec[0].push_back(Point(0, r[1].height - maskHeigth));
	            // maskVec[0].push_back(Point(r[0].width, r[1].height));
	            // maskVec.push_back(vector<Point>());
	            // maskVec[1].push_back(Point(r[1].width, r[1].height));
	            // maskVec[1].push_back(Point(r[1].width, r[1].height - maskHeigth));
	            // maskVec[1].push_back(Point(r[1].width - r[2].width, r[1].height));
	            // /*backward*/
	            // maskVec.push_back(vector<Point>());
	            // maskVec[2].push_back(Point(0, 0));
	            // maskVec[2].push_back(Point(0, maskHeigth));
	            // maskVec[2].push_back(Point(r[0].width, 0));
	            // maskVec.push_back(vector<Point>());
	            // maskVec[3].push_back(Point(mSize.width, 0));
	            // maskVec[3].push_back(Point(mSize.width, maskHeigth));
	            // maskVec[3].push_back(Point(mSize.width - r[2].width, 0));
	            // /*draw  mask*/
							//
	            // drawContours(maskF, maskVec, 0, Scalar(0), CV_FILLED);
	            // drawContours(maskF, maskVec, 1, Scalar(0), CV_FILLED);
	            // drawContours(maskB, maskVec, 2, Scalar(0), CV_FILLED);
	            // drawContours(maskB, maskVec, 3, Scalar(0), CV_FILLED);


	            for (size_t i = 0; i < 4 ; i++)
	            {
	                Birdtransform[i] = getPerspectiveTransform(sourcePoint[i], targetPoint[i]);
	            }

	            if(carPic.data)
	            {
	                //Size newCarSize = Size(carSize.width-2*ShiftAdjust.width,carSize.height-2*ShiftAdjust.height);
	                resize(carPic,carPicTmp,carSize,CV_INTER_CUBIC);
	                carPicPos = Rect(int((mSize.width - carSize.width) / 2.0),
	                                 carShift,
	                                 carSize.width,carSize.height);
	            }

	            ParamSet_OK = true;
	        }
					saveTarget(target_src.c_str());
	    }
	};

	void readInstric(const string& calib, vector<Mat>& intrinsic_matrix, vector<Mat>& distortion_coeffs);
}

#endif //DOSUBIRDVIEW_H
