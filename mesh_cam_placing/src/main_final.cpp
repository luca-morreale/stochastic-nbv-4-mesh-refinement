
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
    
    OpenMvgParser op_openmvg(input_file);
    op_openmvg.parse();
    
    sfm_data_ = op_openmvg.getSfmData();

    auto tmpCams = sfm_data_.camerasList_;
    GLMVec3List defaultCams;
    GLMMat3List defaultRots;

    for (auto cam : tmpCams) {
        defaultCams.push_back(cam.center);
        defaultRots.push_back(cam.rotation);
    }

    CamReader reader(cams_file);
    reader.parse();
    GLMVec3List cams = { reader.getFinalCamera() };
    GLMMat3List rots = { reader.getRotationOfFinal() };

    std::string outname = cams_file.substr(0, cams_file.find_last_of(".json")-4) + "_final";

    generateJsonFromData(cams, defaultCams, rots, defaultRots, outname);
    
    return 0;
}

