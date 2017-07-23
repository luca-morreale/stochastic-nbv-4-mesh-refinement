#include <omp.h>

#include <OpenMvgParser.h>

#include <opview/BruteForceSolverGenerator.hpp>
#include <opview/FlipperSolverGenerator.hpp>
#include <opview/HierarchicalDiscreteGraphicalModel.hpp>
#include <opview/ICMSolverGenerator.hpp>
#include <opview/LOCSolverGenerator.hpp>
#include <opview/AutonomousMultipointHierarchicalGraphicalModel.hpp>
#include <opview/SolverGenerator.hpp>
#include <opview/type_definition.h>

#define OMP_THREADS 8
#define DEPTH 30
#define DISCRETE_LABELS 3


int main(int argc, char **argv) {
    
    omp_set_num_threads(OMP_THREADS);

    std::string jsonFile = argv[1];
    std::string meshFile = argv[2];
    
    OpenMvgParser op_openmvg(jsonFile);
    op_openmvg.parse();
    std::cout << "sfm: " << op_openmvg.getSfmData().numCameras_ << " cams; " << op_openmvg.getSfmData().numPoints_ << " points" << std::endl << std::endl;

    SfMData sfm_data_ = op_openmvg.getSfmData();

    std::vector<glm::vec3> cams;
    for (auto cam : sfm_data_.camerasList_) {
        cams.push_back(cam.center);
    }

    std::vector<glm::vec3> points;
    std::vector<glm::vec3> normals;
    std::vector<double> uncertainty;

    double x, y, z, unc;
    std::ifstream cin("points.txt");
    while(!cin.eof()) {
        cin >> x >> y >> z >> unc;
        points.push_back(glm::vec3(x, y, z));
        uncertainty.push_back(unc);
    }
    cin.close();

    cin.open("normals.txt");
    while(!cin.eof()) {
        cin >> x >> y >> z;
        normals.push_back(glm::vec3(x, y, z));
    }

    // opview::SolverGeneratorPtr solver = new opview::FlipperSolverGenerator();
    // opview::SolverGeneratorPtr solver = new opview::ICMSolverGenerator();
    // opview::SolverGeneratorPtr solver = new opview::LOCSolverGenerator();
    opview::SolverGeneratorPtr solver = new opview::BruteForceSolverGenerator();

    opview::OrientationHierarchicalConfiguration config(DEPTH, DISCRETE_LABELS, {30, 30, 20, 20, 10});
    opview::CameraGeneralConfiguration camConfig(1920, 1080, 959.9965);
    // GLMVec3List &points, GLMVec3List &normals, DoubleList &uncertainty
    opview::MeshConfiguration meshConfig(meshFile, cams, points, normals, uncertainty);

    size_t maxPoints = 10;
    long double thresholdUncertainty = 100000;

    opview::AutonomousMultipointHierarchicalGraphicalModel model(solver, config, camConfig, meshConfig, maxPoints, thresholdUncertainty);



    model.estimateBestCameraPosition();


    return 0;
}
