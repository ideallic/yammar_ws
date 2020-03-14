/*
This is C++ version of DosuBirdView package, which is used for Bird
eye image stitch with 4 RGB cameras.
Notice: any question please contact DOSU ${cbbsjtu@126.com}
*/

#include  "DosuBirdView.h"

using namespace DosuBird;

int main(int argc, char** argv)
{
	//initialization
	ros::init(argc, argv, "find_cut_node_realtime");
	ros::NodeHandle nh;

	//*********FIRST****************
	/*
		命令行输入:
		--calib_mode  鼠标点选外参标定目标点
		--intri_src   内参文件目录
		--extri_src   外参文件目录（没有则创建）
		--target_src  外参文件投影文件(没有则创建)
		--car_src     遮挡mask
		--img_dir     拼接标定图像源
	*/
	string intri_src;
	string extri_src;
	string target_src;
	string car_src;
	string img_dir;
	bool calib_mode;
	bool ros_on;
	bpo::options_description desc("Program options");
	desc.add_options()
		//Options
		("calib_mode",bpo::value<bool>(&calib_mode) -> default_value(false),
									"calibrate by mouse or not, Usage: --calib_mode true/false")
		("ros_on",bpo::value<bool>(&ros_on) -> default_value(true),
									"use image from topics or not, Usage: --ros_on true/false")
		("intri_src", bpo::value<string>(&intri_src)
									-> default_value("/home/dosu/dosuss_ws_test/src/birdView_calibration_v2/calib.txt"),
									"camera intrinsic.xml path, Usage: --intri_src ~/xxx/xxx.xml")
		("extri_src", bpo::value<string>(&extri_src)
									-> default_value("/home/dosu/dosuss_ws_test/src/birdView_calibration_v2/config.xml"),
									"camera extrinsic.xml path, Usage: --extri_src ~/xxx/xxx.xml")
	  ("target_src",bpo::value<string>(&target_src)
									-> default_value("/home/dosu/dosuss_ws_test/src/birdView_calibration_v2/target.xml"),
								  "camera extrinsic_target.xml path, Usage: --extri_src ~/xxx/xxx.xml")
	  ("car_src",   bpo::value<string>(&car_src)
								  -> default_value("/home/dosu/dosuss_ws_test/src/birdView_calibration_v2/img/ParamSetting.png"),
									"car img path, Usage: --car_src ~/xxx/xxx.png")
		("img_dir",   bpo::value<string>(&img_dir)
									-> default_value("/home/dosu/dosuss_ws_test/src/birdView_calibration_v2/img"),
									"images used for calibration, Usage: --img_dir ~/xxx/xxx/");
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

	//mode 1-> calib by local images;
	cout<<"start this node..."<<endl;

	Mat v[4], v_temp[4];

	if(!ros_on)
	{
		for (int i = 0; i < 4; i++)
		{
			// char buf[10];
			// sprintf(buf, "%d.png", i);
			string buf = img_dir + "/" + to_string(i) + ".png";
			v[i] = imread(buf);
		}
	}

	if(ros_on)
	{
		ros::ServiceClient client_f = nh.serviceClient<dosuss_srvs::find_cut>("/get_image_f");
		ros::ServiceClient client_l = nh.serviceClient<dosuss_srvs::find_cut>("/get_image_l");
		ros::ServiceClient client_b = nh.serviceClient<dosuss_srvs::find_cut>("/get_image_b");
		ros::ServiceClient client_r = nh.serviceClient<dosuss_srvs::find_cut>("/get_image_r");
		//ImageConverter top_view("camera/top_view");

		dosuss_srvs::find_cut srv_f;
		dosuss_srvs::find_cut srv_l;
		dosuss_srvs::find_cut srv_b;
		dosuss_srvs::find_cut srv_r;
		srv_f.request.start =true;
		srv_l.request.start =true;
		srv_b.request.start =true;
		srv_r.request.start =true;
		int p_w = 1280;
		int p_h = 960;
		vector<uchar> pixel_r;
		vector<uchar> pixel_g;
		vector<uchar> pixel_b;

		pixel_r.resize(1280*960);
		pixel_g.resize(1280*960);
		pixel_b.resize(1280*960);
		cout<<"extrinsic calibration start with image topics ..."<<endl;
		if (client_f.call(srv_f)&&client_l.call(srv_l)&&client_b.call(srv_b)&&client_r.call(srv_r))
		{
			try
			{
				 pixel_r =  srv_f.response.pixel_r;
				 pixel_g =  srv_f.response.pixel_g;
				 pixel_b =  srv_f.response.pixel_b;
				 v_temp[0] = Mat::zeros(p_h,p_w,CV_8UC3);
				 for (int i=0; i<p_h;i++)
				 {
						for (int j=0; j<p_w;j++)
						{
									v_temp[0].at<Vec3b>(i,j)[0]=pixel_b[j+i*p_w];
									v_temp[0].at<Vec3b>(i,j)[1]=pixel_g[j+i*p_w];
									v_temp[0].at<Vec3b>(i,j)[2]=pixel_r[j+i*p_w];

						}
				 }

				 pixel_r =  srv_l.response.pixel_r;
				 pixel_g =  srv_l.response.pixel_g;
				 pixel_b =  srv_l.response.pixel_b;
				 v_temp[1] = Mat::zeros(p_h,p_w,CV_8UC3);
				 for (int i=0; i<p_h;i++)
				 {
						for (int j=0; j<p_w;j++)
						{
									v_temp[1].at<Vec3b>(i,j)[0]=pixel_b[j+i*p_w];
									v_temp[1].at<Vec3b>(i,j)[1]=pixel_g[j+i*p_w];
									v_temp[1].at<Vec3b>(i,j)[2]=pixel_r[j+i*p_w];

						}
				}
				 pixel_r =  srv_b.response.pixel_r;
				 pixel_g =  srv_b.response.pixel_g;
				 pixel_b =  srv_b.response.pixel_b;
				 v_temp[2] = Mat::zeros(p_h,p_w,CV_8UC3);
				 for (int i=0; i<p_h;i++)
				 {
						for (int j=0; j<p_w;j++)
						{
									v_temp[2].at<Vec3b>(i,j)[0]=pixel_b[j+i*p_w];
									v_temp[2].at<Vec3b>(i,j)[1]=pixel_g[j+i*p_w];
									v_temp[2].at<Vec3b>(i,j)[2]=pixel_r[j+i*p_w];
						}
				}
				 pixel_r =  srv_r.response.pixel_r;
				 pixel_g =  srv_r.response.pixel_g;
				 pixel_b =  srv_r.response.pixel_b;
				 v_temp[3] = Mat::zeros(p_h,p_w,CV_8UC3);
				 for (int i=0; i<p_h;i++)
				 {
						for (int j=0; j<p_w;j++)
						{
									v_temp[3].at<Vec3b>(i,j)[0]=pixel_b[j+i*p_w];
									v_temp[3].at<Vec3b>(i,j)[1]=pixel_g[j+i*p_w];
									v_temp[3].at<Vec3b>(i,j)[2]=pixel_r[j+i*p_w];

						}
				 }

		 }
		 catch (cv_bridge::Exception& e)
		 {
				ROS_ERROR("cv_bridge exception: %s", e.what());
				return 1;
		 }

		 //read instric parameter;
		 vector<Mat> intrinsic_matrix(4);
		 vector<Mat> distortion_coeffs(4);
		 readInstric(intri_src, intrinsic_matrix, distortion_coeffs);

		 undistort(v_temp[0],v[0],intrinsic_matrix[0],distortion_coeffs[0]);
		 undistort(v_temp[1],v[1],intrinsic_matrix[1],distortion_coeffs[1]);
		 undistort(v_temp[2],v[2],intrinsic_matrix[2],distortion_coeffs[2]);
		 undistort(v_temp[3],v[3],intrinsic_matrix[3],distortion_coeffs[3]);


		}
		else
		{
			cout<<"WARNING: it seems service call failed, please check the service node"<<endl;
			return 1;
		}
	}

	if(calib_mode)
	{
		BirdView b(true);
		b.setPath(extri_src, target_src,car_src);
		b.setChessSize(100,140);
		b.setShift(600,800); //(900,1000)
		b.setInternalShift(502,558);
		b.setCarSize(340,400);

		b.sourcePointClick(v);
		b.setCarShift(1050); //must be the last set function
		//b.saveTarget()
		//b.setMaskHeigth(200);
		//b.setInternalShift(27,27);

		while (ros::ok())
		{
			namedWindow("bird view", CV_WINDOW_NORMAL);
      imshow("bird view", b.transformView(v));
			if (waitKey(20) == 27)	break;
		}
	}

	if(!calib_mode)
	{
		BirdView b(true, extri_src.c_str());
		b.setPath(extri_src, target_src,car_src);
		//calibration parameter setting
		//b.setCarSize(240, 380);
		b.setChessSize(100,140);
		b.setShift(600,800); //(900,1000)
		b.setInternalShift(502,558);
		b.setCarSize(340,400);

		//b.sourcePointClick(v);
		b.setCarShift(1050); //must be the last set function
		//b.setMaskHeigth(200);
		//b.setInternalShift(27,27);

		while (ros::ok())
		{
			namedWindow("bird view", CV_WINDOW_NORMAL);
			imshow("bird view", b.transformView(v));
			if (waitKey(20) == 27)	break;
		}
	}

  ros::spin();
  return 0;
}
