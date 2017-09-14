
#include <OpenMvgParser.h>
#include <CamReader.hpp>
#include <aliases.h>
#include <visualization_utils.hpp>
#include <sphere_handler.hpp>

using namespace camplacing;

SfMData sfm_data_;

int main(int argc, char **argv)
{
    if (argc < 6) {
        std::cout << argv[0] << " camfile.txt mvg.json x y z" << std::endl;
        return 1;
    }

    omp_set_num_threads(8);

    std::string input_file, cams_file;

    int maxIterations_ = 0;

    cams_file = argv[1];
    input_file = argv[2];
    float x = std::strtod(argv[3], NULL);
    float y = std::strtod(argv[4], NULL);
    float z = std::strtod(argv[5], NULL);

    GLMVec3 poi = GLMVec3(x, y, z);
    
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
    GLMVec3List cams = { reader.getBestCamera() };
    GLMMat3List rots = { reader.getRotationOfBest() };

    std::string outname = cams_file.substr(0, cams_file.find_last_of(".json")-4) + "_best_score";

    generateJsonFromData(cams, defaultCams, rots, defaultRots, outname, poi);
    
    return 0;
}
