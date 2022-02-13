#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <pangolin/pangolin.h>
#include <string>
#include <unistd.h>
#include <vector>

using namespace std;
using namespace Eigen;

// Load the images
string left_file = "./left.png";
string right_file = "./right.png";

// Method in pangolin to plot the grayscale pointcloud
void showPointCloud(const vector<Vector4d, Eigen::aligned_allocator<Vector4d>> &pointcloud);

int main(int argc, char **argv) {

  // Intrinsics
  double fx = 718.856, fy = 718.856, cx = 607.1928, cy = 185.2157;
  // Baseline
  double b = 0.573;

  // Read the images as grayscales and get the disparity map
  cv::Mat left = cv::imread(left_file, 0);
  cv::Mat right = cv::imread(right_file, 0);
  // Semi global block matching to get the disparity maps
  // Very parameter dependent, read the paper to understand
  cv::Ptr<cv::StereoSGBM> sgbm =
      cv::StereoSGBM::create(0, 96, 9, 8 * 9 * 9, 32 * 9 * 9, 1, 63, 10, 100, 32);
  cv::Mat disparity_sgbm, disparity;
  sgbm->compute(left, right, disparity_sgbm);
  disparity_sgbm.convertTo(disparity, CV_32F, 1.0 / 16.0f);

  // Vector to store the pointcloud
  vector<Vector4d, Eigen::aligned_allocator<Vector4d>> pointcloud;

  // Cycle through th pixels
  for (int v = 0; v < left.rows; v++)
    for (int u = 0; u < left.cols; u++) {
      // Check if the disparity (which is in pixels) is valid
      if (disparity.at<float>(v, u) <= 0.0 || disparity.at<float>(v, u) >= 96.0) continue;

      // Initialize the position an the grayscale value
      Vector4d point(0, 0, 0, left.at<uchar>(v, u) / 255.0);

      // Steps to get pointcloud:
      // - Get the depth using the fx (in pixels), the disparity (in pixels) and
      // the baseline (in meters)
      // Get the normalized coordinates which are basically projections on plane z=1
      // Multiply the x,y coordinates with the depth and th z coordinate is the depth
      double depth = fx * b / (disparity.at<float>(v, u));
      double x = (u - cx) / fx;
      double y = (v - cy) / fy;
      point[0] = x * depth;
      point[1] = y * depth;
      point[2] = depth;

      pointcloud.push_back(point);
    }

  cv::imshow("disparity", disparity / 96.0);
  cv::waitKey(0);
  // Show the pointcloud in pangolin
  showPointCloud(pointcloud);
  return 0;
}

void showPointCloud(const vector<Vector4d, Eigen::aligned_allocator<Vector4d>> &pointcloud) {

  if (pointcloud.empty()) {
    cerr << "Point cloud is empty!" << endl;
    return;
  }

  pangolin::CreateWindowAndBind("Point Cloud Viewer", 1024, 768);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  pangolin::OpenGlRenderState s_cam(
      pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
      pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0));

  pangolin::View &d_cam =
      pangolin::CreateDisplay()
          .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
          .SetHandler(new pangolin::Handler3D(s_cam));

  while (pangolin::ShouldQuit() == false) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    d_cam.Activate(s_cam);
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

    glPointSize(2);
    glBegin(GL_POINTS);
    for (auto &p : pointcloud) {
      glColor3f(p[3], p[3], p[3]);
      glVertex3d(p[0], p[1], p[2]);
    }
    glEnd();
    pangolin::FinishFrame();
    usleep(5000);  // sleep 5 ms
  }
  return;
}
