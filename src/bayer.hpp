#include <opencv2/opencv.hpp>

// Adopted from
// https://answers.opencv.org/question/114387/how-to-convert-from-bgr-to-bayerrg
static inline cv::Mat ConvertBGR2BayerRG(cv::Mat BGRImage) {
  cv::Mat BayerImage(BGRImage.rows, BGRImage.cols, CV_8UC1);
  for (int row = 0; row < BayerImage.rows; row++) {
    for (int col = 0; col < BayerImage.cols; col++) {
      int channel;
      // R G   2 1
      // G B   1 0
      if (row % 2 == 0) {
        // even columns and even rows = red = channel:2
        // even columns and odd rows = green = channel:1
        channel = (col % 2 == 0) ? 2 : 1;
      } else {
        // odd columns and even rows = green = channel:1
        // odd columns and odd rows = blue = channel:0
        channel = (col % 2 == 0) ? 1 : 0;
      }
      BayerImage.at<uchar>(row, col) =
          BGRImage.at<cv::Vec3b>(row, col).val[channel];
    }
  }
  return BayerImage;
}

