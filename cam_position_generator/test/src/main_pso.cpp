
#include <cstdlib>
#include <iostream>
#include <omp.h>

#include <OpenMvgParser.h>

#include <opview/type_definition.h>
#include <opview/PSOCamGenerator.hpp>
#include <opview/LocalPSOCamGenerator.hpp>
#include <opview/utilities.hpp>


#define OMP_THREADS 8


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

    opview::CameraGeneralConfiguration camConfig(1920, 1080, 959.9965);

    glm::vec3 centroid(-0.402134, -0.0704882, 2.31928);

    glm::vec3 normal(-0.14341, 0.238821, -0.960416);
    // // centroid, normal

    opview::MCConfiguration mcConfig(30, 100000, 100, 30); // size_t resamplingNum, size_t particles, size_t particlesUniform
    // maybe is better if less point in uniform? and then increase in the case of mc?

    opview::PSOConfiguration psoConfig(mcConfig, glm::vec3(-3, -3, -3), glm::vec3(3, 3, 3));
    
    opview::PSOCamGenerator model(camConfig, meshFile, cams, psoConfig);
    // opview::LocalPSOCamGenerator model(camConfig, meshFile, cams, psoConfig);

    model.estimateBestCameraPosition(centroid, normal);

    return 0;
}
