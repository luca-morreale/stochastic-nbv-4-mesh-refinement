//  Copyright 2014 Andrea Romanoni
//
//  This file is part of manifoldReconstructor.
//
//  edgePointSpaceCarver is free software: you can redistribute it
//  and/or modify it under the terms of the GNU General Public License as
//  published by the Free Software Foundation, either version 3 of the
//  License, or (at your option) any later version see
//  <http://www.gnu.org/licenses/>..
//
//  edgePointSpaceCarver is distributed in the hope that it will be
//  useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.

#include <omp.h>
#include <opencv2/core/core.hpp>
#include <opencv2/line_descriptor/descriptor.hpp>

#include <realtimeMR/CameraPointsCollection.h>
#include <realtimeMR/utilities/Chronometer.h>
#include <realtimeMR/utilities/Logger.h>
//#include <ORBIncrementalParser.h>
#include <realtimeMR/ReconstructFromSLAMData.h>
#include <OpenMvgParser.h>
#include <realtimeMR/ReconstructFromSfMData.h>
#include <realtimeMR/ConfigParser.h>
#include <realtimeMR/types_config.hpp>
#include <realtimeMR/types_reconstructor.hpp>
#include <cstdlib>
#include <iostream>
#include <map>
#include <set>
#include <utility>

// accuracy check imports
#include <meshac/PointAccuracyModel.hpp>
#include <meshac/ResidualPointAccuracyModel.hpp>
#include <meshac/BasicPhotogrammetristAccuracyModel.hpp>
#include <meshac/InvertedResidualPointAccuracyModel.hpp>
#include <meshac/Color.hpp>
#include <meshac/DeterminantVarianceEstimator.hpp>
#include <meshac/WorstEigenvalueVarianceEstimator.hpp>
#include <meshac/AverageVarianceEstimator.hpp>
#include <meshac/VertexColorer.hpp>

#include <aliases.hpp>
#include <ReportGenerator.hpp>

#define OMP_THREADS 8
#define TIMING
#define COLOR

SfMData sfm_data_;

int point2D3DJacobian(const std::vector<cv::Mat> &cameras, const cv::Mat &cur3Dpoint, cv::Mat &J, cv::Mat &hessian);
int GaussNewton(const std::vector<cv::Mat> &cameras, const std::vector<cv::Point2f> &points, cv::Point3f init3Dpoint, cv::Point3f &optimizedPoint, const float outlierThreshold);
void outlierFiltering(std::vector<bool>& inliers, const float outlierThreshold);

int main(int argc, char **argv) {

    omp_set_num_threads(OMP_THREADS);

    ManifoldReconstructionConfig confManif;
    std::string input_file;
    std::string config_file;
    std::string out_report;

    int maxIterations_ = 0;


    if (argc < 3) {
        std::cout << argv[0] << " mvg.json off_file outputlog" << std::endl;
        return 1;
    }

    input_file = argv[1];
    // second param not used, required for standard
    out_report = argv[3];

    config_file = "res/config/default.json";
    std::cout << "Using default color configuration res/config/colors.json" << std::endl;
    std::cout << "Using default configuration res/config/default.json" << std::endl;
    std::cout << "max_iterations not set" << std::endl << std::endl;

    ConfigParser configParser = ConfigParser();
    confManif = configParser.parse(config_file);

    std::cout << "input set to: " << input_file << std::endl;
    std::cout << "config set to: " << config_file << std::endl;
    std::cout << "max_iterations set to: " << maxIterations_ << std::endl;
    std::cout << confManif.toString() << std::endl;


    CameraPointsCollection incData;
    OpenMvgParser op_openmvg(input_file);
    op_openmvg.parse();
    sfm_data_ = op_openmvg.getSfmData();

    std::string pathPrefix = input_file.substr(0, input_file.find_last_of("/"));
    pathPrefix = pathPrefix.substr(0, pathPrefix.find_last_of("/")+1);
    std::pair<double, double> pixelSize(0.0003527, 0.0003527);
    
    auto accuracyModel = new meshac::BasicPhotogrammetristAccuracyModel(sfm_data_);
    auto estimator = new meshac::WorstEigenvalueVarianceEstimator(accuracyModel, sfm_data_.points_);
    // auto estimator = new meshac::DeterminantVarianceEstimator(accuracyModel, sfm_data_.points_);
    // auto estimator = new meshac::AverageVarianceEstimator(accuracyModel, sfm_data_.points_);

    std::vector<bool> inliers;
    outlierFiltering(inliers, confManif.outlierFilteringThreshold);

    std::vector<glm::vec3> points;
    for (int i = 0; i < inliers.size(); i++) {
        if (inliers[i]) {
            points.push_back(sfm_data_.points_[i]);
        }
    }
    
    ReportGenerator report(estimator, points);

#ifdef TIMING
    millis accStart = now();
    millis accCount;    
#endif

    report.generateReport(out_report);

#ifdef TIMING
    accCount = now() - accStart;
    std::cout << std::endl << std::endl << "Total time to estimate accuracy: " << accCount.count() << "ms" << std::endl;
#endif

    std::cout << "DONE!" << std::endl;


    delete estimator;

    return 0;
}

int point2D3DJacobian(const std::vector<cv::Mat> &cameras, const cv::Mat &cur3Dpoint, cv::Mat &J, cv::Mat &hessian) {

    int numMeasures = cameras.size();
    cv::Mat cur3DPointHomog = cv::Mat(4, 1, CV_32F);

    cur3DPointHomog.at<float>(0, 0) = cur3Dpoint.at<float>(0, 0);
    cur3DPointHomog.at<float>(1, 0) = cur3Dpoint.at<float>(1, 0);
    cur3DPointHomog.at<float>(2, 0) = cur3Dpoint.at<float>(2, 0);
    cur3DPointHomog.at<float>(3, 0) = 1.0;

    J = cv::Mat(2 * numMeasures, 3, CV_32FC1);  //2 rows for each point: one for x, the other for y
    hessian = cv::Mat(3, 3, CV_32FC1);

    for (int curMeas = 0; curMeas < numMeasures; ++curMeas) {
        cv::Mat curReproj = cameras[curMeas] * cur3DPointHomog;
        float xH = curReproj.at<float>(0, 0);
        float yH = curReproj.at<float>(1, 0);
        float zH = curReproj.at<float>(2, 0);
        float p00 = cameras[curMeas].at<float>(0, 0);
        float p01 = cameras[curMeas].at<float>(0, 1);
        float p02 = cameras[curMeas].at<float>(0, 2);
        float p10 = cameras[curMeas].at<float>(1, 0);
        float p11 = cameras[curMeas].at<float>(1, 1);
        float p12 = cameras[curMeas].at<float>(1, 2);
        float p20 = cameras[curMeas].at<float>(2, 0);
        float p21 = cameras[curMeas].at<float>(2, 1);
        float p22 = cameras[curMeas].at<float>(2, 2);

        //d(P*X3D)/dX
        J.at<float>(2 * curMeas, 0) = (p00 * zH - p20 * xH) / (zH * zH);
        J.at<float>(2 * curMeas + 1, 0) = (p10 * zH - p20 * yH) / (zH * zH);

        //d(P*X3D)/dY
        J.at<float>(2 * curMeas, 1) = (p01 * zH - p21 * xH) / (zH * zH);
        J.at<float>(2 * curMeas + 1, 1) = (p11 * zH - p21 * yH) / (zH * zH);

        //d(P*X3D)/dZ
        J.at<float>(2 * curMeas, 2) = (p02 * zH - p22 * xH) / (zH * zH);
        J.at<float>(2 * curMeas + 1, 2) = (p12 * zH - p22 * yH) / (zH * zH);
    }

    hessian = J.t() * J;
    float d;
    d = cv::determinant(hessian);
    if (d < 0.0000000001) {
        return -1;
    } else {
        return 1;
    }
}

int GaussNewton(const std::vector<cv::Mat> &cameras, const std::vector<cv::Point2f> &points, cv::Point3f init3Dpoint, cv::Point3f &optimizedPoint, const float outlierThreshold) {
    int numMeasures = points.size();
    cv::Mat r = cv::Mat(numMeasures * 2, 1, CV_32F);

    cv::Mat curEstimate3DPoint = cv::Mat(3, 1, CV_32F);
    cv::Mat curEstimate3DPointH = cv::Mat(4, 1, CV_32F);
    curEstimate3DPoint.at<float>(0, 0) = init3Dpoint.x;
    curEstimate3DPoint.at<float>(1, 0) = init3Dpoint.y;
    curEstimate3DPoint.at<float>(2, 0) = init3Dpoint.z;

    cv::Mat J, H;
    float last_mse = 0;
    int i;
    for (i = 0; i < 30; i++) {

        float mse = 0;
        //compute residuals
        for (int curMeas = 0; curMeas < numMeasures; ++curMeas) {
            curEstimate3DPointH.at<float>(0, 0) = curEstimate3DPoint.at<float>(0, 0);
            curEstimate3DPointH.at<float>(1, 0) = curEstimate3DPoint.at<float>(1, 0);
            curEstimate3DPointH.at<float>(2, 0) = curEstimate3DPoint.at<float>(2, 0);
            curEstimate3DPointH.at<float>(3, 0) = 1.0;
            cv::Mat cur2DpositionH = cameras[curMeas] * curEstimate3DPointH;

            r.at<float>(2 * curMeas, 0) = ((points[curMeas].x - cur2DpositionH.at<float>(0, 0) / cur2DpositionH.at<float>(2, 0)));
            mse += r.at<float>(2 * curMeas, 0) * r.at<float>(2 * curMeas, 0);

            r.at<float>(2 * curMeas + 1, 0) = ((points[curMeas].y - cur2DpositionH.at<float>(1, 0) / cur2DpositionH.at<float>(2, 0)));
            mse += r.at<float>(2 * curMeas + 1, 0) * r.at<float>(2 * curMeas + 1, 0);
#ifdef DEBUG_OPTIMIZATION_VERBOSE
            if(i==0) {
                std::cout<<"CurMeas: "<<curMeas<<std::endl<<"curEstimate3DPointH="<< curEstimate3DPointH.t()<<std::endl;
                std::cout<<"CurCam"<<cameras[curMeas]<<std::endl;
                std::cout<<"cur2DpositionH: "<<cur2DpositionH.at<float>(0, 0)/cur2DpositionH.at<float>(2, 0)<<", "<<cur2DpositionH.at<float>(1, 0)/cur2DpositionH.at<float>(2, 0)<<std::endl;
                std::cout<<"points[curMeas]: "<<points[curMeas]<<std::endl;
                std::cout<<"residual on x: "<<r.at<float>(2 * curMeas, 0)<<std::endl;
                std::cout<<"residual on y: "<<r.at<float>(2 * curMeas + 1 , 0)<<std::endl;
                std::cout<<std::endl;}
#endif
        }

        if (abs(mse / (numMeasures * 2) - last_mse) < 0.0000000005) {
            break;
        }
        last_mse = mse / (numMeasures * 2);

        if (point2D3DJacobian(cameras, curEstimate3DPoint, J, H) == -1) {
            return -1;
        }
#ifdef DEBUG_OPTIMIZATION_VERBOSE
        std::cout<<"J: "<<J<<std::endl;
        std::cout<<"H: "<<H<<std::endl;
#endif

        curEstimate3DPoint += H.inv() * J.t() * r;

#ifdef DEBUG_OPTIMIZATION
        std::cout << "It= " << i << " last_mse " << last_mse << std::endl;
#endif
    }

    if (last_mse < outlierThreshold/*3 pixels*/) {
        optimizedPoint.x = curEstimate3DPoint.at<float>(0, 0);
        optimizedPoint.y = curEstimate3DPoint.at<float>(1, 0);
        optimizedPoint.z = curEstimate3DPoint.at<float>(2, 0);
        return 1;
    } else {
        return -1;
    }
}

void outlierFiltering(std::vector<bool>& inliers, const float outlierThreshold) {

    inliers.assign(sfm_data_.points_.size(), false);
    std::vector<cv::Mat> cameras;
    std::vector<cv::Point2f> measures;
    cv::Point3f init3Dpoint;
    cv::Point3f optimizedPoint;

    for (int curPt3D = 0; curPt3D < sfm_data_.points_.size(); curPt3D++) {
        cameras.clear();
        cameras.assign(sfm_data_.camViewingPointN_[curPt3D].size(), cv::Mat());
        for (int curC = 0; curC < sfm_data_.camViewingPointN_[curPt3D].size(); curC++) {
            cameras[curC] = cv::Mat(4, 4, CV_32F);
            for (int row = 0; row < 4; row++) {
                for (int col = 0; col < 4; col++) {
                    cameras[curC].at<float>(row, col) = sfm_data_.camerasList_[sfm_data_.camViewingPointN_[curPt3D][curC]].cameraMatrix[row][col];
                }
            }

        }

        measures.clear();
        measures.assign(sfm_data_.point2DoncamViewingPoint_[curPt3D].size(), cv::Point2f());
        for (int curMeas = 0; curMeas < sfm_data_.point2DoncamViewingPoint_[curPt3D].size(); curMeas++) {
            measures[curMeas].x = sfm_data_.point2DoncamViewingPoint_[curPt3D][curMeas].x;
            measures[curMeas].y = sfm_data_.point2DoncamViewingPoint_[curPt3D][curMeas].y;
        }

        init3Dpoint.x = sfm_data_.points_[curPt3D].x;
        init3Dpoint.y = sfm_data_.points_[curPt3D].y;
        init3Dpoint.z = sfm_data_.points_[curPt3D].z;
        

        if (GaussNewton(cameras, measures, init3Dpoint, optimizedPoint, outlierThreshold) != -1) {

            sfm_data_.points_[curPt3D].x = optimizedPoint.x;
            sfm_data_.points_[curPt3D].y = optimizedPoint.y;
            sfm_data_.points_[curPt3D].z = optimizedPoint.z;
            inliers[curPt3D] = true;
        }
    }

}


/**
Building:
8594ms (with load)

Fortress:
74266ms (with load)

Car:
30081ms (with load)

*/
