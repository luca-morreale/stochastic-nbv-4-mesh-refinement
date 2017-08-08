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
#include <meshac/PhotogrammetristAccuracyModel.hpp>
#include <meshac/ComputerVisionAccuracyModel.hpp>
#include <meshac/Color.hpp>
#include <meshac/DeterminantVarianceEstimator.hpp>
#include <meshac/WorstEigenvalueVarianceEstimator.hpp>
#include <meshac/AverageVarianceEstimator.hpp>
#include <meshac/VertexColorer.hpp>

#define OMP_THREADS 8

#define COLOR
//#define PRODUCE_STATS
//#define SAVE_POINTS_TO_OFF_AND_EXIT

SfMData sfm_data_;
meshac::VertexColorerPtr meshColorer;

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

int main(int argc, char **argv) {

    omp_set_num_threads(OMP_THREADS);

    utilities::Logger log;
    std::ofstream statsFile, visiblePointsFile;
    ManifoldReconstructionConfig confManif;
    std::string input_file;
    std::string color_file;
    std::string config_file;

    int maxIterations_ = 0;


    if (argc < 2) {
        std::cout << argv[0] << " mvg.json" << std::endl;
        return 1;
    }

    input_file = argv[1];
    color_file = "res/config/colors.json";
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

    log.startEvent();

    std::cout << "parsing" << std::endl;

    CameraPointsCollection incData;
    OpenMvgParser op_openmvg(input_file);
    op_openmvg.parse();
    sfm_data_ = op_openmvg.getSfmData();
    std::cout << "sfm: " << sfm_data_.numCameras_ << " cams; " << sfm_data_.numPoints_ << " points" << std::endl << std::endl;

    ReconstructFromSLAMData m(confManif);

    std::string pathPrefix = input_file.substr(0, input_file.find_last_of("/"));
    pathPrefix = pathPrefix.substr(0, pathPrefix.find_last_of("/")+1);
    std::pair<double, double> pixelSize(0.0003527, 0.0003527);
    // meshac::PhotogrammetristAccuracyModel accuracyModel(sfm_data_, pathPrefix, pixelSize);
    meshac::ComputerVisionAccuracyModel accuracyModel(sfm_data_, pathPrefix, pixelSize);
    meshColorer = new meshac::VertexColorer(color_file, new meshac::WorstEigenvalueVarianceEstimator(&accuracyModel, sfm_data_.points_));
    // meshColorer = new meshac::VertexColorer(color_file, new meshac::DeterminantVarianceEstimator(&accuracyModel, sfm_data_.points_));
    // meshColorer = new meshac::VertexColorer(color_file, new meshac::AverageVarianceEstimator(&accuracyModel, sfm_data_.points_));

    m.setExpectedTotalIterationsNumber((maxIterations_) ? maxIterations_ + 1 : sfm_data_.numCameras_);

    

    std::cout << "computing inliers" << std::endl;

    std::vector<bool> inliers;
    outlierFiltering(inliers, confManif.outlierFilteringThreshold);

    for (int cameraIndex = 0; cameraIndex < sfm_data_.camerasList_.size(); cameraIndex++) {
        CameraType* camera = &sfm_data_.camerasList_[cameraIndex];
        camera->idCam = cameraIndex;

        incData.addCamera(camera);
    }

    for (int pointIndex = 0; pointIndex < sfm_data_.points_.size(); pointIndex++) {
        if (inliers[pointIndex]) {
            PointType* point = new PointType();
            point->idPoint = pointIndex;
            point->position = sfm_data_.points_[pointIndex];

#ifdef COLOR
            // this is already after the outlier filtering, thus no outlier are computed??mkdi
            meshac::Color color = meshColorer->getColorForPoint(pointIndex);
            std::cout << "color " << color.to_string() << std::endl;
            point->r = color.r;
            point->g = color.g;
            point->b = color.b;
            point->a = color.a;
#endif
            incData.addPoint(point);
        }
    }
    delete meshColorer;

    for (int cameraIndex = 0; cameraIndex < sfm_data_.camerasList_.size(); cameraIndex++) {

        int inliersCount = 0, inlierPointIndex = 0;
        for (auto pointIndex : sfm_data_.pointsVisibleFromCamN_[cameraIndex])
            if (inliers[pointIndex]) inliersCount++;

//      std::cout << "camera " << cameraIndex << "i: " << inliersCount << "i/m" << inliersCount / confManif.maxPointsPerCamera << std::endl;

        for (auto pointIndex : sfm_data_.pointsVisibleFromCamN_[cameraIndex]) {
            if (confManif.maxPointsPerCamera < inliersCount) {
//          std::cout <<  pointIndex << "\t" << pointCount << "%" << confManif.maxPointsPerCamera << " = " << pointCount%(inliersCount/confManif.maxPointsPerCamera) << std::endl;
                if (inliers[pointIndex]) {

                    if (inlierPointIndex < confManif.maxPointsPerCamera){// && !(inlierPointIndex % ((int)((float)inliersCount / confManif.maxPointsPerCamera)))) {
                        incData.addVisibility(cameraIndex, pointIndex);
                    }

                    inlierPointIndex++;
                }

            } else {
                if (inliers[pointIndex]) incData.addVisibility(cameraIndex, pointIndex);
            }
        }

    }

    for (auto index_camera : incData.getCameras()) {
        std::cout << "camera " << index_camera.second->idCam << ":\t " << index_camera.second->visiblePointsT.size() << " points" << std::endl;
    }

    std::cout << "Inliers: " << incData.numPoints() << "\t/\t" << sfm_data_.numPoints_ << std::endl << std::endl;

    // Main loop
    for (auto index_camera : incData.getCameras()) {
        CameraType* camera = (index_camera.second);

        if (camera == NULL) {
            continue;
        }

        // If maxIterations_ is set, only execute ReconstructFromSLAMData::addCamera maxIterations_ times
        if (maxIterations_ && m.iterationCount >= maxIterations_) break;

        log.startEvent();

        m.addCamera(camera);

        // Skip the manifold update for the first confManif.initial_manifold_update_skip cameras
        if (m.iterationCount > confManif.initialTriangulationUpdateSkip && !(m.iterationCount % confManif.triangulationUpdateEvery)) m.update();
        
        if (m.iterationCount > confManif.initialTriangulationUpdateSkip && !(m.iterationCount % confManif.triangulationUpdateEvery)) m.integrityCheck();

        if (m.iterationCount && !(m.iterationCount % confManif.saveMeshEvery)) m.saveMesh("output/from_gen_config/", "current", true);
        //if (m.iterationCount && !(m.iterationCount % confManif.save_manifold_every)) m.saveManifold("output/partial/", std::to_string(m.iterationCount));

        //if (m.iterationCount && !(m.iterationCount % confManif.saveMeshEvery)) m.getOutputManager()->writeMeshToOff("output/from_gen_config/current_from_OutputManager.off");

        log.endEventAndPrint("main loop\t\t\t\t\t\t", true);
        std::cout << std::endl;

        if (m.iterationCount > confManif.initialTriangulationUpdateSkip && !(m.iterationCount % confManif.triangulationUpdateEvery)) m.insertStatValue(log.getLastDelta());
    }

    // Do a last manifold update in case op.numCameras() isn't a multiple of confManif.manifold_update_every
    if (m.iterationCount > confManif.initialTriangulationUpdateSkip) m.update();

    m.saveMesh("output/from_gen_config/", "final", true);

    log.endEventAndPrint("main\t\t\t\t\t\t", true);

    return 0;
}
