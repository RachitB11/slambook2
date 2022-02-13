#include <chrono>
#include <iostream>

using namespace std;

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char **argv) {
  // Read the image using args
  cv::Mat image;
  image = cv::imread(argv[1]);  // cv::imread to read the image

  // Check if the data is null
  if (image.data == nullptr) {
    cerr << "The file " << argv[1] << "does not exist." << endl;
    return 0;
  }

  // Print some info about the image
  cout << "Image width " << image.cols << ",Image height " << image.rows
       << ",Number of channels" << image.channels() << endl;
  cv::imshow("image", image);  // cv::imshow to show the image
  cv::waitKey(0);              // Display and wait for keyboard input

  // Check if the image is a singe grayscale or colored image
  if (image.type() != CV_8UC1 && image.type() != CV_8UC3) {
    // Print invalid format
    cout << "The format is not supported." << endl;
    return 0;
  }

  // Read through the data in the image
  // Using std::chrono to count the read time
  chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
  for (size_t y = 0; y < image.rows; y++) {
    // Use the cv::Mat::ptr to get the data in the row
    unsigned char *row_ptr = image.ptr<unsigned char>(y);  // row_ptr typecast to unsigned char
    for (size_t x = 0; x < image.cols; x++) {
      // Remember x,y is the u,v coordinates with origin in the top left corner,
      // x in the right and y pointing down
      unsigned char *data_ptr =
          &row_ptr[x * image.channels()];  // data_ptr contains the data for a pixel
      // Cycle through the image channels to get the individual channel data
      for (int c = 0; c != image.channels(); c++) {
        unsigned char data = data_ptr[c];
      }
    }
  }
  chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
  chrono::duration<double> time_used =
      chrono::duration_cast<chrono::duration<double>>(t2 - t1);
  cout << "遍历图像用时：" << time_used.count() << " 秒。" << endl;

  // More cv::Mat operations
  // Make a shallow copy, = generates a reference
  cv::Mat image_another = image;
  // Set the right had 100x100 block of image_another to black
  image_another(cv::Rect(0, 0, 100, 100)).setTo(0);
  // Since its a shallow copy it'll also have the black patch
  cv::imshow("image", image);
  cv::waitKey(0);

  // Make a deep copy to avoid referencing
  cv::Mat image_clone = image.clone();
  image_clone(cv::Rect(0, 0, 100, 100)).setTo(255);
  cv::imshow("image", image);
  cv::imshow("image_clone", image_clone);
  cv::waitKey(0);

  // We are not going to copy the OpenCV's documentation here
  // please take a look at it for other image operations like clipping, rotating and
  // scaling.
  cv::destroyAllWindows();
  return 0;
}
