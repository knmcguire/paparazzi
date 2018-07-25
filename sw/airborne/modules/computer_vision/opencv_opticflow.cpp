/*
 * Copyright (C) C. De Wagter
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/computer_vision/opencv_example.cpp"
 * @author C. De Wagter
 * A simple module showing what you can do with opencv on the bebop.
 */


#include "opencv_opticflow.h"

#include <opencv2/core/core.hpp>
#include "opencv2/video/tracking.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/video/tracking.hpp"

#include "opencv_image_functions.h"

using namespace cv;
using namespace std;

#include "std.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/time.h>

static void drawOptFlowMap(const Mat &flow, Mat &cflowmap, int step,
                           double, const Scalar &color)
{
  for (int y = 0; y < cflowmap.rows; y += step)
    for (int x = 0; x < cflowmap.cols; x += step) {
      const Point2f &fxy = flow.at<Point2f>(y, x);
      line(cflowmap, Point(x, y), Point(cvRound(x + fxy.x), cvRound(y + fxy.y)),
           color);
      circle(cflowmap, Point(x, y), 2, color, -1);
    }
}
static UMat prevgray;

bool opencv_opticflow(char *img, int width, int height, float *flow_x, float *flow_y)
{
 // struct timeval stop, start;
  //gettimeofday(&start, NULL);

  // Create a new image, using the original bebop image.
  Mat image(height, width, CV_8UC2, img);

  // Initializing umats and mats
  UMat gray, uflow;
  Mat cflow, flow, frame;

  //Turn image to grayscale
  cvtColor(image, gray, CV_YUV2GRAY_Y422);

  //resize image for faster computation
  Size size(40, 40);
  UMat gray_small;
  resize(gray, gray_small, size);


  if (!prevgray.empty()) {

	  // OpenCV optical flow FarneBack
    calcOpticalFlowFarneback(prevgray, gray_small, uflow, 0.5, 1, 15, 2, 4, 1.2, 0);
    uflow.copyTo(flow);

    vector<Mat> channels(2);
    split(flow, channels);
    Scalar flow_x_s = mean(channels[0]);
    Scalar flow_y_s = mean(channels[1]);
    *flow_x = flow_x_s.val[0];
    *flow_y = flow_y_s.val[0];

#ifdef OPENCV_OPTICFLOW_DEBUG
    //Draw flow map
    cvtColor(prevgray, cflow, COLOR_GRAY2BGR);
    drawOptFlowMap(flow, cflow, 8, 1.5, Scalar(0, 255, 0));
    //Calculate mean flow
    printf("mean flow: x:%f y:%f \n", flow_x.val[0], flow_y.val[0]);
    Size size(width, height);
    Mat big_cflow;
    resize(cflow, big_cflow, size); //resize image
#else
    Mat big_cflow;
    gray.copyTo(big_cflow);
#endif
    grayscale_opencv_to_yuv422(big_cflow, img, width, height);

    //TODO: NEEDS FIX() cvtColor, convertion is not correct, only part of the image is tranvered
    // cvtColor(big_cflow, big_cflow, CV_BGR2GRAY);
  }
  std::swap(prevgray, gray_small);

 // gettimeofday(&stop, NULL);
 // printf("It's %f hz\n", (float)(1/((stop.tv_usec - start.tv_usec)/1000000.0)));

  return true;
}
