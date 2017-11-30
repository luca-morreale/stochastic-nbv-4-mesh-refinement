#include <omp.h>

#include <OpenMvgParser.h>

#include <opview/BruteForceSolverGenerator.hpp>
#include <opview/AutonomousMultipointHierarchicalGraphicalModel.hpp>
#include <opview/SolverGenerator.hpp>
#include <opview/type_definition.h>

#include <aliases.hpp>
#include <utilities.h>

#define TIMING

#define OMP_THREADS 8
#define DEPTH 10
#define DISCRETE_LABELS 3
#define ARGS 4


int main(int argc, char **argv) {
    
    omp_set_num_threads(OMP_THREADS);

    if (argc < ARGS + 1) {
        std::cout << "Usage: " << argv[0] << " mgvjson.json meshfile.off accuracy_points.txt out.txt" << std::endl;
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
    std::cout << scores.points.size() << std::endl;

    // opview::SolverGeneratorPtr solver = new opview::FlipperSolverGenerator();
    // opview::SolverGeneratorPtr solver = new opview::ICMSolverGenerator();
    // opview::SolverGeneratorPtr solver = new opview::LOCSolverGenerator();
    opview::SolverGeneratorPtr solver = new opview::BruteForceSolverGenerator();

    // opview::SpaceBounds bounds(glm::vec3(-40, 0, -40), glm::vec3(0, 70, 40)); // building
    // opview::SpaceBounds bounds(glm::vec3(-40, 0, -130), glm::vec3(40, 70, 100)); // fortress
    opview::SpaceBounds bounds(glm::vec3(-15, 0, -15), glm::vec3(15, 10, 0)); // car
    opview::OrientationHierarchicalConfiguration config(DEPTH, DISCRETE_LABELS, bounds, {30, 30, 20, 20, 10});

    //opview::CameraGeneralConfiguration camConfig(1920, 1080, 959.9965); // building fort
    opview::CameraGeneralConfiguration camConfig(1920, 1080, 1662.8); // car
    // GLMVec3List &points, GLMVec3List &normals, DoubleList &uncertainty
    opview::MeshConfiguration meshConfig(meshFile, cams, scores.points, scores.normals, scores.uncertainty);

    size_t maxPoints = 10;
    long double thresholdUncertainty = 1; // useless

    std::cout << "start" << std::endl;
    opview::AutonomousMultipointHierarchicalGraphicalModel model(solver, config, camConfig, meshConfig, maxPoints);
    std::cout << "model\n";

#ifdef TIMING
    millis start = now();
#endif

    auto result = model.estimateBestCameraPosition();

#ifdef TIMING
        std::cout << std::endl << std::endl << "Total time to compute optimal pose: " << (now()-start).count() << "ms" << std::endl;
#endif

    result[3] = opview::deg2rad(result[3]);
    result[4] = opview::deg2rad(result[4]);

    std::ofstream out(output);
    for (int i = 0; i < result.size(); i++) {
        out << result[i] << " ";
    }
    out.close();

    return 0;
}
/**
Building:
167813ms


Fortress:
568311ms

Car:
218697ms

*/
