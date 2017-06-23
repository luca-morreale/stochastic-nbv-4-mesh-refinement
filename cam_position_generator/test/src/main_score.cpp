#include <omp.h>

#include <OpenMvgParser.h>

#include <opview/BruteForceSolverGenerator.hpp>
#include <opview/ScorerGM.hpp>
#include <opview/SolverGenerator.hpp>
#include <opview/type_definition.h>

#include <gsl/gsl_blas.h>
#include <gsl/gsl_linalg.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_randist.h>
#include <gsl/gsl_rng.h>
#include <gsl/gsl_sf_gamma.h>
#include <gsl/gsl_vector.h>



#define OMP_THREADS 8
#define DEPTH 10
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


    opview::SolverGeneratorPtr solver = new opview::BruteForceSolverGenerator();

    opview::OrientationHierarchicalConfiguration config(DEPTH, DISCRETE_LABELS, {30, 30, 20, 20, 10});
    opview::CameraGeneralConfiguration camConfig(1920, 1080, 959.9965);
    
    opview::ScorerGM model(solver, config, camConfig, meshFile, cams); // 0.712458, -0.462458, 0.462458
    
    // auto centroid = glm::vec3(-0.402134, -0.0704882, 2.31928);
    auto centroid = glm::vec3(-0.402134, -0.0704882, 2.31928);
    auto normal = glm::vec3(-0.14341, 0.238821, -0.960416);
    opview::EigVector5 camPose;
    camPose << 0.0166327, -0.484079, -0.16239, 350, 340;
    
    // centroid, normal
    double score = model.score(camPose, centroid, normal);
    std::cout << score << std::endl;


    return 0;
}
