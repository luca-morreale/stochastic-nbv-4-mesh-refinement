#include <omp.h>

#include <OpenMvgParser.h>

#include <meshac/FaceAccuracyModel.hpp>
#include <meshac/NCCFaceAccuracyModel.hpp>


#define OMP_THREADS 8


int main(int argc, char **argv) {
    
    omp_set_num_threads(OMP_THREADS);

    std::string jsonFile = argv[1];
    std::string meshFile = argv[2];
    
    OpenMvgParser op_openmvg(jsonFile);
    op_openmvg.parse();
    std::cout << "sfm: " << op_openmvg.getSfmData().numCameras_ << " cams; " << op_openmvg.getSfmData().numPoints_ << " points" << std::endl << std::endl;

    SfMData sfm_data_ = op_openmvg.getSfmData();

    std::string pathPrefix = jsonFile.substr(0, jsonFile.find_last_of("/"));
    pathPrefix = pathPrefix.substr(0, pathPrefix.find_last_of("/")+1);

    meshac::FaceAccuracyModelPtr model;

    model = new meshac::NCCFaceAccuracyModel(meshFile, sfm_data_, pathPrefix);

    for (int i = 0; i < 2000; i++) {
        std::cout << "result: " << model->getAccuracyForFace(i) << std::endl;
        // model->getAccuracyForFace(i);
    }

    // poche viste comuni dalle diverse camere
    
    delete model;
    
    return 0;
}
