
#include <OpenMvgParser.h>
#include <CamReader.hpp>
#include <aliases.h>
#include <visualization_utils.hpp>

using namespace camplacing;

SfMData sfm_data_;

int main(int argc, char **argv)
{
    if (argc < 3) {
        std::cout << "missing arguments" << std::endl;
        return 1;
    }

    omp_set_num_threads(8);

    std::string input_file, cams_file;

    int maxIterations_ = 0;

    input_file = argv[2];
    cams_file = argv[1];
    std::cout << "Using default configuration res/config/default.json" << std::endl;
    std::cout << "max_iterations not set" << std::endl << std::endl;

    std::cout << "parsing" << std::endl;
    OpenMvgParser op_openmvg(input_file);
    op_openmvg.parse();
    std::cout << "sfm: " << op_openmvg.getSfmData().numCameras_ << " cams; " << op_openmvg.getSfmData().numPoints_ << " points" << std::endl << std::endl;

    sfm_data_ = op_openmvg.getSfmData();

    auto tmpCams = sfm_data_.camerasList_;
    GLMVec3List defaultCams;
    GLMMat3List defaultRots;

    for (auto cam : tmpCams) {
        defaultCams.push_back(cam.center);
        defaultRots.push_back(cam.rotation);
    }

    std::cout<<"parsed \n";
    CamReader reader(cams_file);
    reader.parse();
    GLMVec3List cams = { reader.getBestCamera() };
    GLMMat3List rots = { reader.getRotationOfBest() };

    std::string outname = cams_file.substr(0, cams_file.find_last_of(".json")-4) + "_best_score";

    generateJsonFromData(cams, defaultCams, rots, defaultRots, outname);
    
    return 0;
}

