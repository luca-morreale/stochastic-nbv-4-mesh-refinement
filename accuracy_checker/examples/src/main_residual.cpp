#include <omp.h>
#include <opencv2/core/core.hpp>

#include <manifoldReconstructor/Chronometer.h>
#include <manifoldReconstructor/CameraPointsCollection.h>
#include <manifoldReconstructor/Chronometer.h>
#include <manifoldReconstructor/Logger.h>
#include <manifoldReconstructor/ReconstructFromSLAMData.h>
#include <manifoldReconstructor/ReconstructFromSfMData.h>
#include <manifoldReconstructor/ConfigParser.h>
#include <manifoldReconstructor/types_config.hpp>
#include <manifoldReconstructor/types_reconstructor.hpp>

#include <cstdlib>
#include <iostream>
#include <map>
#include <set>
#include <utility>

// accuracy check imports
#include <meshac/AverageVarianceEstimator.hpp>
#include <meshac/Color.hpp>
#include <meshac/DeterminantVarianceEstimator.hpp>
#include <meshac/InvertedResidualPointAccuracyModel.hpp>
#include <meshac/PointAccuracyModel.hpp>
#include <meshac/ResidualPointAccuracyModel.hpp>
#include <meshac/SfMData.h>
#include <meshac/VertexColorer.hpp>
#include <meshac/WorstEigenvalueVarianceEstimator.hpp>

#include <aliases.hpp>
#include <convertSfM.hpp>
#include <filter_points.hpp>
#include <OpenMvgParser.h>

#define OMP_THREADS 8
#define COLOR

meshac::SfMData sfm_data;
meshac::VertexColorerPtr meshColorer;



int main(int argc, char **argv) {

    omp_set_num_threads(OMP_THREADS);

    utilities::Logger log;
    std::ofstream statsFile, visiblePointsFile;
    ManifoldReconstructionConfig confManif;
    std::string input_file;
    std::string color_file;
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
    sfm_data = op_openmvg.getSfmData();
    std::cout << "sfm: " << sfm_data.numCameras_ << " cams; " << sfm_data.numPoints_ << " points" << std::endl << std::endl;

    ReconstructFromSLAMData m(confManif);

    std::string pathPrefix = input_file.substr(0, input_file.find_last_of("/"));
    pathPrefix = pathPrefix.substr(0, pathPrefix.find_last_of("/")+1);
    std::pair<double, double> pixelSize(0.0003527, 0.0003527);
    meshac::ResidualPointAccuracyModel accuracyModel(sfm_data);
    // meshac::InvertedResidualPointAccuracyModel accuracyModel(sfm_data);
    auto estimator = new meshac::WorstEigenvalueVarianceEstimator(&accuracyModel, sfm_data.points_);
    meshColorer = new meshac::VertexColorer(color_file, estimator);
    // meshColorer = new meshac::VertexColorer(color_file, new meshac::DeterminantVarianceEstimator(&accuracyModel, sfm_data.points_));
    // meshColorer = new meshac::VertexColorer(color_file, new meshac::AverageVarianceEstimator(&accuracyModel, sfm_data.points_));

    m.setExpectedTotalIterationsNumber((maxIterations_) ? maxIterations_ + 1 : sfm_data.numCameras_);

    std::cout << "computing inliers" << std::endl;

    std::vector<bool> inliers;
    outlierFiltering(sfm_data, inliers, confManif.outlierFilteringThreshold);
    SfMData sfm_data_ = convertSfMData(sfm_data);

    for (int cameraIndex = 0; cameraIndex < sfm_data_.camerasList_.size(); cameraIndex++) {
        CameraType* camera = &sfm_data_.camerasList_[cameraIndex];
        camera->idCam = cameraIndex;
//      std::cout << "camera " << camera->idCam << std::endl;

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
    //delete(meshColorer);

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

/***
Inverted:
-Building 
    Total time to estimate accuracy: 5092ms
    Total time to create mesh: 10507ms
-Fortress
    Total time to estimate accuracy: 9935ms
    Total time to create mesh: 302581ms
-Car
    Total time to estimate accuracy: 6667ms
    Total time to create mesh: 51333ms

Residual:
-Building
    Total time to estimate accuracy: 5076ms
    Total time to create mesh: 11134ms
-Fortress
    Total time to estimate accuracy: 10750ms
    Total time to create mesh: 319682ms
-Car
    Total time to estimate accuracy: 6491ms
    Total time to create mesh: 54912ms

 ****/