
#include <opview/GraphicalModelBuilder.hpp>

#include <OpenMvgParser.h>

#include <omp.h>

#include <iostream>

int main(int argc, char **argv) {
    
    omp_set_num_threads(4);
    
    OpenMvgParser op_openmvg(argv[1]);
    op_openmvg.parse();

    auto sfm_data_ = op_openmvg.getSfmData();
    

    std::vector<glm::vec3> cams;
    for (auto cam : sfm_data_.camerasList_) {
        cams.push_back(cam.center);
    }

    opview::GraphicalModelBuilder model(cams);

    auto centroid = glm::vec3(-0.402134, -0.0704882, 2.31928);
    auto normVector = glm::vec3(-0.14341, 0.238821, -0.960416);

    // centroid, normal
    std::cout << "created class\n";
    model.estimateBestCameraPosition(centroid, normVector);
    std::cout << "created model\n";
    //model.solve();


    return 0;
}

