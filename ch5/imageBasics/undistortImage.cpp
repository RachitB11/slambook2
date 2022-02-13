#include <opencv2/opencv.hpp>
#include <string>

using namespace std;

string image_file = "./distorted.png";  // We should get the abs path instead of relative path

int main(int argc, char **argv) {
  cout << __FILE__ << endl;

  // In thie program we implement the undistortion by ourselves rather than using
  // opencv which does have an undistort function
  // rad tan params
  double k1 = -0.28340811, k2 = 0.07395907, p1 = 0.00019359, p2 = 1.76187114e-05;
  // Intrinsics (all in pixels)
  double fx = 458.654, fy = 457.296, cx = 367.215, cy = 248.375;

  cv::Mat image = cv::imread(image_file, 0);  // 0 indicates grayscale CV_8UC1
  int rows = image.rows, cols = image.cols;
  cv::Mat image_undistort =
      cv::Mat(rows, cols, CV_8UC1);  // Allocate space for distorted image

  // Cycle through all the pixels
  for (int v = 0; v < rows; v++) {
    for (int u = 0; u < cols; u++) {
      // Steps to undistortion:
      // - Estimate the normalized coordinates which are the values on the z=1 plane
      // - Estimate the radius and hence the distorted normalized coordinates
      // - Convert the distorted normalized coordinates to distorted pixel coordinates
      // - These are the locations in the distorted image that we need to sample from
      // to get the current pixel coordinate data in the undistorted image
      double x = (u - cx) / fx, y = (v - cy) / fy;
      double r = sqrt(x * x + y * y);
      double x_distorted = x * (1 + k1 * r * r + k2 * r * r * r * r) + 2 * p1 * x * y +
                           p2 * (r * r + 2 * x * x);
      double y_distorted = y * (1 + k1 * r * r + k2 * r * r * r * r) +
                           p1 * (r * r + 2 * y * y) + 2 * p2 * x * y;
      double u_distorted = fx * x_distorted + cx;
      double v_distorted = fy * y_distorted + cy;

      // Check if the distorted pixel coordinates are in the image window
      if (u_distorted >= 0 && v_distorted >= 0 && u_distorted < cols && v_distorted < rows) {
        image_undistort.at<uchar>(v, u) = image.at<uchar>((int)v_distorted, (int)u_distorted);
      } else {
        image_undistort.at<uchar>(v, u) = 0;
      }
    }
  }

  // Show the distorted and undistorted image
  cv::imshow("distorted", image);
  cv::imshow("undistorted", image_undistort);
  cv::waitKey();
  return 0;
}
