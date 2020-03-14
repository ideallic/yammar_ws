#include  "DosuBirdView.h"

namespace DosuBird
{
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
}
