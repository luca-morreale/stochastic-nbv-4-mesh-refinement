
#include <cstdlib>
#include <iostream>
#include <omp.h>

#include <OpenMvgParser.h>

#include <opview/type_definition.h>
#include <opview/AutonomousPSOCamGenerator.hpp>
#include <opview/utilities.hpp>

#include <aliases.hpp>
#include <utilities.h>

#define TIMING

#define OMP_THREADS 8
#define ARGS 4

#define RESAMPLE 10


int main(int argc, char **argv) {
    
    omp_set_num_threads(OMP_THREADS);

    if (argc < ARGS + 1) {
        std::cout << "Usage: " << argv[0] << " mgvjson.json meshfile.off accScore.txt output.txt" << std::endl;
        return 1;
    }

    std::string jsonFile = argv[1];
    std::string meshFile = argv[2];
    std::string score = argv[3];
    std::string output = argv[4];
    
    OpenMvgParser op_openmvg(jsonFile);
    op_openmvg.parse();
    // std::cout << "sfm: " << op_openmvg.getSfmData().numCameras_ << " cams; " << op_openmvg.getSfmData().numPoints_ << " points" << std::endl << std::endl;

    SfMData sfm_data_ = op_openmvg.getSfmData();

    std::vector<glm::vec3> cams;
    for (auto cam : sfm_data_.camerasList_) {
        cams.push_back(cam.center);
    }

    auto scores = utilities::readScores(score);

    opview::CameraGeneralConfiguration camConfig(1920, 1080, 959.9965); // building & fort
    // opview::CameraGeneralConfiguration camConfig(1920, 1080, 1662.8); // 1662.8 car

    opview::MeshConfiguration meshConfig(meshFile, cams, scores.points, scores.normals, scores.uncertainty);

    // // centroid, normal

    // opview::SpaceBounds bounds(glm::vec3(-40, 0, -40), glm::vec3(0, 70, 40)); // building
    opview::SpaceBounds bounds(glm::vec3(-40, 0, -130), glm::vec3(40, 70, 100)); // fortress
    // opview::SpaceBounds bounds(glm::vec3(-15, 0, -15), glm::vec3(15, 10, 0)); // car
    opview::ParticlesInformation particles(10000, 10, 30);

    opview::StochasticConfiguration stoConfig(RESAMPLE, particles, bounds); // size_t resamplingNum, size_t particles, size_t particlesUniform
    // maybe is better if less point in uniform? and then increase in the case of mc?

    size_t maxPoints = 10;
    long double thresholdUncertainty = 100;
    
    opview::AutonomousPSOCamGenerator model(camConfig, meshConfig, stoConfig, maxPoints);

#ifdef TIMING
    millis start = now();
#endif

    auto result = model.estimateBestCameraPosition();

#ifdef TIMING
        std::cout << std::endl << std::endl << "Total time to compute optimal pose: " << (now()-start).count() << "ms" << std::endl;
#endif

    std::ofstream out(output);
    for (int i = 0; i < result.size(); i++) {
        out << result[i] << " ";
    }
    out.close();

    return 0;
}


/**
Building:
100k -> 13743ms
10K -> 909ms


Fortress:
100K -> 43337ms
10K -> 2807ms

Car:
100K -> 17615ms
10K -> 1200ms
*/
