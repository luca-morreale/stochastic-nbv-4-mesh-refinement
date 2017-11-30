#ifndef FILTER_POINTS

#include <opencv2/core/core.hpp>

#include <cstdlib>
#include <iostream>
#include <map>
#include <set>
#include <utility>

#include <meshac/SfMData.h>

void outlierFiltering(meshac::SfMData &sfm_data_, std::vector<bool>& inliers, const float outlierThreshold);

int GaussNewton(const std::vector<cv::Mat> &cameras, const std::vector<cv::Point2f> &points, cv::Point3f init3Dpoint, cv::Point3f &optimizedPoint, const float outlierThreshold);

int point2D3DJacobian(const std::vector<cv::Mat> &cameras, const cv::Mat &cur3Dpoint, cv::Mat &J, cv::Mat &hessian);

#endif // FILTER_POINTS
