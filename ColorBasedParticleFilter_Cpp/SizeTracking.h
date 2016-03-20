#pragma once
#include "opencv2/objdetect.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;

int calc_first_class_points(Mat m);
int calc_second_class_points(Mat& m);
void create_central_difference_image(Mat& dest, int offset, Mat& m);
bool feature_point_or_not(Mat& m, int x, int y);
bool checking_sign(int, int);