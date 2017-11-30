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

#include <cstdlib>
#include <iostream>
#include <map>
#include <set>
#include <utility>

// accuracy check imports
#include <meshac/AverageVarianceEstimator.hpp>
#include <meshac/Color.hpp>
#include <meshac/ComputerVisionAccuracyModel.hpp>
#include <meshac/DeterminantVarianceEstimator.hpp>
#include <meshac/InvariantAccuracyModel.hpp>
#include <meshac/PointAccuracyModel.hpp>
#include <meshac/SfMData.h>
#include <meshac/VertexColorer.hpp>
#include <meshac/WorstEigenvalueVarianceEstimator.hpp>

#include <aliases.hpp>
#include <filter_points.hpp>
#include <OpenMvgParser.h>
#include <ReportGenerator.hpp>

#define OMP_THREADS 8
#define TIMING

meshac::SfMData sfm_data_;

int main(int argc, char **argv) {

    omp_set_num_threads(OMP_THREADS);

    std::string input_file;
    std::string out_report;

    int maxIterations_ = 0;


    if (argc < 3) {
        std::cout << argv[0] << " mvg.json off_file outputlog" << std::endl;
        return 1;
    }

    input_file = argv[1];
    // second param not used, required for standard
    out_report = argv[3];


    OpenMvgParser op_openmvg(input_file);
    op_openmvg.parse();
    sfm_data_ = op_openmvg.getSfmData();

    std::string pathPrefix = input_file.substr(0, input_file.find_last_of("/"));
    pathPrefix = pathPrefix.substr(0, pathPrefix.find_last_of("/")+1);
    std::pair<double, double> pixelSize(0.0003527, 0.0003527);
    
    auto accuracyModel = new meshac::InvariantAccuracyModel(sfm_data_, pathPrefix, pixelSize);
    // auto accuracyModel = new meshac::ComputerVisionAccuracyModel(sfm_data_, pathPrefix, pixelSize);
    auto estimator = new meshac::WorstEigenvalueVarianceEstimator(accuracyModel, sfm_data_.points_);
    // auto estimator = new meshac::DeterminantVarianceEstimator(accuracyModel, sfm_data_.points_);
    // auto estimator = new meshac::AverageVarianceEstimator(accuracyModel, sfm_data_.points_);

    std::vector<bool> inliers;
    outlierFiltering(sfm_data_, inliers, 0.25);

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
