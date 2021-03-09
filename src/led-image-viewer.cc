#include "led-matrix.h"
#include "pixel-mapper.h"
#include "content-streamer.h"

#include <fcntl.h>
#include <float.h>
#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

#include <algorithm>
#include <map>
#include <string>
#include <vector>

#include <Magick++.h>
#include <magick/image.h>

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio/videoio.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>

using rgb_matrix::GPIO;
using rgb_matrix::Canvas;
using rgb_matrix::FrameCanvas;
using rgb_matrix::RGBMatrix;
using rgb_matrix::StreamReader;

typedef int64_t tmillis_t;

static std::vector< cv::Vec3b > available_colors;

volatile bool interrupt_received = false;
static void InterruptHandler(int signo) {
  interrupt_received = true;
}

static tmillis_t GetTimeInMillis() {
  struct timeval tp;
  gettimeofday(&tp, NULL);
  return tp.tv_sec * 1000 + tp.tv_usec / 1000;
}

static void InitializeAvailableColors() {
  // Colors in BGR
  cv::Vec3b green_color(149.0, 224.0, 170.0);
  cv::Vec3b red_color(121.0, 124.0, 205.0);
  // cv::Vec3b blue_color(252.0, 249.0, 204.0);
  // cv::Vec3b gray_color(201.0, 195.0, 184.0);
  cv::Vec3b brown_color(73.0, 95.0, 113.0);
  // cv::Vec3b yellow_color(152.0, 221.0, 211.0);
  // cv::Vec3b orange_color(64.0, 88.0, 178.0);
  // cv::Vec3b white_color(255.0, 255.0, 255.0);
  // cv::Vec3b black_color(0.0, 0.0, 0.0);

  // available_colors = {green_color, red_color, blue_color, gray_color, brown_color, yellow_color, orange_color, white_color, black_color};
  available_colors = {green_color, red_color, brown_color};
}

static cv::Vec3b FindNearestColor(cv::Vec3b received_color) {
  double min_distance = DBL_MAX;
  cv::Vec3b nearest_color(0.0, 0.0, 0.0);

  for (auto it : available_colors) {
    double distance = sqrt(pow(it.val[0] - received_color.val[0], 2.0) + pow(it.val[1] - received_color.val[1], 2.0) + pow(it.val[2] - received_color.val[2], 2.0));
    if (distance < min_distance) {
      nearest_color = it;
      min_distance = distance;
    }
  }

  return nearest_color;
}

static int usage(const char *progname) {
  fprintf(stderr, "usage: %s [options]\n", progname);

  fprintf(stderr, "\nGeneral LED matrix options:\n");
  rgb_matrix::PrintMatrixFlags(stderr);

  return 1;
}

int main(int argc, char *argv[]) {

  InitializeAvailableColors();

  // Setup Videocapture from webcam
  cv::VideoCapture cap;
  int deviceID = 0;
  int apiID = cv::CAP_V4L2;
  cap.open(deviceID, apiID);

  // Initialize the GraphicsMagick image-processing library
  Magick::InitializeMagick(*argv);

  // Initialize RGBMatrix with options:
  // Hardware gpio mapping: adafruit-hat (compatible with bonnet)
  // Rows of display: 64
  // Cols of display: 64
  // Brightness level: 80%
  RGBMatrix::Options matrix_options;
  matrix_options.hardware_mapping = "adafruit-hat";
  matrix_options.rows = 64;
  matrix_options.cols = 64;
  matrix_options.brightness = 80;

  rgb_matrix::RuntimeOptions runtime_opt;
  if (!rgb_matrix::ParseOptionsFromFlags(&argc, &argv, &matrix_options, &runtime_opt)) {
    return usage(argv[0]);
  }
  // Prepare matrix
  const char *stream_output = NULL;
  runtime_opt.do_gpio_init = (stream_output == NULL);
  RGBMatrix *canvas = rgb_matrix::CreateMatrixFromOptions(matrix_options, runtime_opt);
  if (canvas == NULL) {
    return 1;
  }

  // Set up offscreen canvas
  FrameCanvas *offscreen_canvas = canvas->CreateFrameCanvas();

  printf("Size: %dx%d. Hardware gpio mapping: %s\n", canvas->width(), canvas->height(), matrix_options.hardware_mapping);

  // Interrupt Handler
  signal(SIGTERM, InterruptHandler);
  signal(SIGINT, InterruptHandler);

  if(!cap.isOpened()) { std::cout << "Cannot open webcam" << std::endl; }

  // std::string fname = "/home/ubuntu/chameleon/ME499_Chameleon/test_images/test.bmp";
  // cv::imwrite(fname, frame);

  // Define rectangular region of interest
  // Start position
  // cv::Rect ROI(310, 220, 250, 250);
  // End Position
  // cv::Rect ROI(140, 220, 250, 250);
  
  // *************************************************
  // Test image
  // cv::Mat frame;
  // cap >> frame;
  // if(!cv::imwrite("img_before_cropped.png", frame)) {
  //   std::cout << "Failed to save image" << std::endl;
  // }
  // cv::Mat cropped_frame(frame, ROI);
  // if(!cv::imwrite("img_cropped.png", cropped_frame)) {
  //   std::cout << "Failed to save image" << std::endl;
  // }
  // cv::Mat hough_circles;
  // cv::resize(cropped_frame, hough_circles, cv::Size(), 4.0, 4.0);
  // cv::Mat hough_areas = hough_circles.clone();
  // cv::Mat gray;
  // cv::cvtColor(hough_circles, gray, cv::COLOR_BGR2GRAY);
  // cv::medianBlur(gray, gray, 5);
  // std::vector< cv::Vec3f > circles;
  // cv::HoughCircles(gray, circles, cv::HOUGH_GRADIENT, 1, 8, 80, 10, 2, 10);
  // for(size_t i = 0; i < circles.size(); i++) {
  //   cv::Vec3i c = circles[i];
  //   cv::Point center = cv::Point(c[0], c[1]);
  //   cv::circle(hough_circles, center, 1, cv::Scalar(0,100,100), 3, cv::LINE_AA);
  //   int radius = c[2];
  //   circle(hough_circles, center, radius, cv::Scalar(255,0,255), 3, cv::LINE_AA);
  // }
  // if(!cv::imwrite("img_hough_circles.png", hough_circles)) {
  //   std::cout << "Failed to save image" << std::endl;
  // }

  // std::sort(circles.begin(), circles.end(), 
  //   [](const cv::Vec3f& a, const cv::Vec3f& b) {
  //     if (a[0] == b[0]) return a[1] < b[1];
  //     return a[0] < b[0];
  //   });
  // std::vector< std::vector<cv::Point> > areas;
  // for(size_t i = 0; i < circles.size(); i++) {
  //   cv::Vec3i c = circles[i];
  //   cv::Point center = cv::Point(c[0], c[1]);
  //   if (areas.empty()) {
  //     std::vector<cv::Point> new_area = {center};
  //     areas.push_back(new_area);
  //   } else {
  //     bool find_area = false;
  //     for (auto& area : areas) {
  //       for (auto& point : area) {
  //         if (sqrt(pow(point.x - center.x, 2) + pow(point.y - center.y, 2)) < 50) {
  //           area.push_back(center);
  //           find_area = true;
  //           break;
  //         }
  //       }
  //       if (find_area) break;
  //     }
  //     if (!find_area) {
  //       std::vector<cv::Point> new_area = {center};
  //       areas.push_back(new_area);
  //     }
  //   }
  // }

  // std::vector< bool > is_merged(sizeof(areas), false);
  // std::vector< std::vector<cv::Point> > merge_areas;
  // for (int i = 0; i < sizeof(areas); i++) {
  //   if (!is_merged[i]) {
  //     for (int j = i; j < sizeof(areas); j++) {
  //       if (!is_merged[j])
  //       {
  //         bool is_joined = false;
  //         for (auto& parent_point : areas[i]) {
  //           for (auto& child_point : areas[j]) {
  //             if (sqrt(pow(child_point.x - parent_point.x, 2) + pow(child_point.y - parent_point.y, 2)) < 20) {
  //               areas[i].insert(areas[i].end(), areas[j].begin(), areas[j].end());
  //               child_area = {cv::Point(-1, -1)};
  //               is_joined = true;
  //               break;
  //             }
  //           }
  //           if (is_joined) break;
  //         }
  //       }
  //     }
  //     merge_areas.push_back(parent_area);
  //     parent_area = {cv::Point(-1, -1)};
  //   }
  // }

  // std::cout << areas.size() << std::endl;
  // for (auto& area : areas) {
  //   cv::Point poly[1][4];
  //   poly[0][0] = cv::Point(1279, 1279); // top left
  //   poly[0][1] = cv::Point(0, 1279); // top right
  //   poly[0][2] = cv::Point(0, 0); // bottom right
  //   poly[0][3] = cv::Point(1279, 0); // bottom left
  //   for (auto& point : area) {
  //     if(point.x < poly[0][0].x) poly[0][0].x = point.x;
  //     if(point.y < poly[0][0].y) poly[0][0].y = point.y;
  //     if(point.x > poly[0][1].x) poly[0][1].x = point.x;
  //     if(point.y < poly[0][1].y) poly[0][1].y = point.y;
  //     if(point.x > poly[0][2].x) poly[0][2].x = point.x;
  //     if(point.y > poly[0][2].y) poly[0][2].y = point.y;
  //     if(point.x < poly[0][3].x) poly[0][3].x = point.x;
  //     if(point.y > poly[0][3].y) poly[0][3].y = point.y;
  //   }
  //   // const cv::Point* ppt[1] = {poly[0]};
  //   // int npt[] = {4};
  //   // cv::rectangle(hough_areas, top_left, bottom_right, cv::Scalar(0, 255, 0));
  //   // cv::fillPoly(hough_areas, ppt, npt, 1, cv::Scalar(0, 255, 0));
  //   cv::line(hough_areas, poly[0][0], poly[0][2], cv::Scalar(0, 255, 0), cv::LINE_AA);
  //   cv::line(hough_areas, poly[0][1], poly[0][3], cv::Scalar(0, 255, 0), cv::LINE_AA);
  // }
  // if(!cv::imwrite("img_hough_areas.png", hough_areas)) {
  //   std::cout << "Failed to save image" << std::endl;
  // }

  // *************************************************
  // Runtime
  const tmillis_t start_time_ms = GetTimeInMillis();
  while(cap.isOpened() && !interrupt_received) {
    // 64x64 pixel matrix
    float pixels[64][64][3];
    tmillis_t delta_time_ms = GetTimeInMillis() - start_time_ms;

    cv::Rect ROI(265-(int)((265-115)*delta_time_ms/720000), 55, 250, 250);
    cv::Mat frame;
    cap >> frame;
    // Crop frame
    cv::Mat cropped_frame(frame, ROI);

    // Hough Circles Approach
    cv::Mat src;
    // Resize frame
    cv::resize(cropped_frame, src, cv::Size(), 4.0, 4.0);
    // printf("%d x %d\n", src.rows, src.cols);
    // HoughCircles
    cv::Mat gray;
    cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);
    cv::medianBlur(gray, gray, 5);
    std::vector< cv::Vec3f > circles;
    cv::HoughCircles(gray, circles, cv::HOUGH_GRADIENT, 1, 8, 80, 10, 2, 10);
    
    // Initialize pixel matrix
    for (int i = 0; i < 64; i++) {
      for (int j = 0; j < 64; j++) {
        pixels[i][j][0] = 0.f;
        pixels[i][j][1] = 0.f;
        pixels[i][j][2] = 0.f;
      }
    }

    // Get circles' center color  
    for (std::size_t i = 0; i < circles.size(); i++) {
      cv::Vec3i c = circles[i];
      cv::Vec3b intensity = src.at<cv::Vec3b>(c[1], c[0]);
      // std::cout << "[" << intensity.val[0] << ", " << intensity.val[1] << ", " << intensity.val[2] << "]" << std::endl;
      cv::Vec3b display_color = FindNearestColor(intensity);
      int x = (int)floor(c[0]/15.625);
      int y = (int)floor(c[1]/15.625);
      pixels[x][y][0] = display_color.val[0];
      pixels[x][y][1] = display_color.val[1];
      pixels[x][y][2] = display_color.val[2];
    }

    // Clear black dots noise
    for (size_t i = 0; i < 64; ++i) {
      for (size_t j = 0; j < 64; ++j) {
        if (pixels[i][j][0] == 0.0){
          int is_noise = 0;
          float cover_pixel[3] = {0.0, 0.0, 0.0};

          if (i > 0 && pixels[i-1][j][0] != 0.0) {
            is_noise += 1;
            cover_pixel[0] = pixels[i-1][j][0];
            cover_pixel[1] = pixels[i-1][j][1];
            cover_pixel[2] = pixels[i-1][j][2];
          }
          if (i < 63 && pixels[i+1][j][0] != 0.0) {
            is_noise += 1;
            cover_pixel[0] = pixels[i+1][j][0];
            cover_pixel[1] = pixels[i+1][j][1];
            cover_pixel[2] = pixels[i+1][j][2];
          }
          if (j > 0 && pixels[i][j-1][0] != 0.0) {
            is_noise += 1;
            cover_pixel[0] = pixels[i][j-1][0];
            cover_pixel[1] = pixels[i][j-1][1];
            cover_pixel[2] = pixels[i][j-1][2];
          }
          if (j < 63 && pixels[i][j+1][0] != 0.0) {
            is_noise += 1;
            cover_pixel[0] = pixels[i][j+1][0];
            cover_pixel[1] = pixels[i][j+1][1];
            cover_pixel[2] = pixels[i][j+1][2];
          }

          if (is_noise >= 3) {
            pixels[i][j][0] = cover_pixel[0];
            pixels[i][j][1] = cover_pixel[1];
            pixels[i][j][2] = cover_pixel[2];
          }
        }
      }
    }

    // Display Image
    for (size_t y = 0; y < 64; ++y) {
      for (size_t x = 0; x < 64; ++x) {
        offscreen_canvas->SetPixel(x, y, (int)pixels[x][y][2], (int)pixels[x][y][1], (int)pixels[x][y][0]);
      }
    }
    
    offscreen_canvas = canvas->SwapOnVSync(offscreen_canvas);
  }

  if (interrupt_received) {
    fprintf(stderr, "Caught signal. Exiting.\n");
  }

  // Shut down the RGB matrix. Release webcam
  canvas->Clear();
  delete canvas;
  cap.release();
  cv::destroyAllWindows();

  return 0;
}

