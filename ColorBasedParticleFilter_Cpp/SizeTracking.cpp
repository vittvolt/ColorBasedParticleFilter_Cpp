#include "SizeTracking.h"

int calc_first_class_points(Mat m) {
	int width = m.cols;
	int height = m.rows;
	int count = 0;

	for (int x = 1; x < width - 1; x++) {
		for (int y = 1; y < height - 1; y++) {
			int central_intensity = m.at<uchar>(Point(x, y));

			int i1 = m.at<uchar>(Point(x - 1, y - 1)) - central_intensity;
			int i9 = m.at<uchar>(Point(x + 1, y + 1)) - central_intensity;

			int i2 = m.at<uchar>(Point(x, y - 1)) - central_intensity;
			int i8 = m.at<uchar>(Point(x, y + 1)) - central_intensity;

			int i3 = m.at<uchar>(Point(x + 1, y - 1)) - central_intensity;
			int i7 = m.at<uchar>(Point(x - 1, y + 1)) - central_intensity;

			int i4 = m.at<uchar>(Point(x - 1, y)) - central_intensity;
			int i6 = m.at<uchar>(Point(x + 1, y)) - central_intensity;

			/*if (i1 * i9 > 0 && i2 * i8 > 0 && i3 * i7 > 0 && i4 * i6 > 0)
				count++; */
			if (checking_sign(i1, i9) && checking_sign(i2, i8) && checking_sign(i3, i7) && checking_sign(i4, i6))
				count++;
		}
	}
	return count;
}

int calc_second_class_points(Mat& m) {
	Mat central_difference_image_0(m.rows, m.cols, CV_8UC1);
	Mat central_difference_image_1(m.rows, m.cols, CV_8UC1);
	Mat central_difference_image_2(m.rows, m.cols, CV_8UC1);
	Mat central_difference_image_3(m.rows, m.cols, CV_8UC1);
	int count = 0;

	create_central_difference_image(central_difference_image_0, 0, m);
	create_central_difference_image(central_difference_image_1, 1, m);
	create_central_difference_image(central_difference_image_2, 2, m);
	create_central_difference_image(central_difference_image_3, 3, m);

	for (int x = 1; x < m.cols - 1; x++) {
		for (int y = 1; y < m.rows - 1; y++) {
			uchar val = 0;
			if (feature_point_or_not(central_difference_image_0, x, y)
				&& feature_point_or_not(central_difference_image_1, x, y)
				&& feature_point_or_not(central_difference_image_2, x, y)
				&& feature_point_or_not(central_difference_image_3, x, y)) {

				count++;
			}
		}
	}
	return count;
}

void create_central_difference_image(Mat& dest, int offset, Mat& m) {
	int width = m.cols;
	int height = m.rows;

	for (int x = 1; x < width - 1; x++) {
		for (int y = 1; y < height - 1; y++) {
			int central_intensity = m.at<uchar>(Point(x, y));
			uchar val = 0;

			int i1 = m.at<uchar>(Point(x - 1, y - 1)) - central_intensity;
			int i9 = m.at<uchar>(Point(x + 1, y + 1)) - central_intensity;

			int i2 = m.at<uchar>(Point(x, y - 1)) - central_intensity;
			int i8 = m.at<uchar>(Point(x, y + 1)) - central_intensity;

			int i3 = m.at<uchar>(Point(x + 1, y - 1)) - central_intensity;
			int i7 = m.at<uchar>(Point(x - 1, y + 1)) - central_intensity;

			int i4 = m.at<uchar>(Point(x - 1, y)) - central_intensity;
			int i6 = m.at<uchar>(Point(x + 1, y)) - central_intensity;

			if (offset == 0)
				val = i6 - i4;
			else if (offset == 1)
				val = i3 - i7;
			else if (offset == 2)
				val = i2 - i8;
			else if (offset == 3)
				val = i1 - i9;

			dest.at<uchar>(y, x) = val;
		}
	}
}

bool feature_point_or_not(Mat& m, int x, int y) {
	int width = m.cols;
	int height = m.rows;

	int central_intensity = m.at<uchar>(Point(x, y));

	int i1 = m.at<uchar>(Point(x - 1, y - 1)) - central_intensity;
	int i9 = m.at<uchar>(Point(x + 1, y + 1)) - central_intensity;

	int i2 = m.at<uchar>(Point(x, y - 1)) - central_intensity;
	int i8 = m.at<uchar>(Point(x, y + 1)) - central_intensity;

	int i3 = m.at<uchar>(Point(x + 1, y - 1)) - central_intensity;
	int i7 = m.at<uchar>(Point(x - 1, y + 1)) - central_intensity;

	int i4 = m.at<uchar>(Point(x - 1, y)) - central_intensity;
	int i6 = m.at<uchar>(Point(x + 1, y)) - central_intensity;

	/*if (i1 * i9 > 0 && i2 * i8 > 0 && i3 * i7 > 0 && i4 * i6 > 0)
		return true;
	else {
		return false;
	} */
	if (checking_sign(i1, i9) && checking_sign(i2, i8) && checking_sign(i3, i7) && checking_sign(i4, i6))
		return true;
	else
		return false;
}

bool checking_sign(int x, int y) {
	if (x == 0 || y == 0)
		return false;
	else
		return ((x < 0) == (y < 0));
}