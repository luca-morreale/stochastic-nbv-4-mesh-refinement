
#include <manifoldReconstructor/OpenMvgParser.h>
#include <manifoldReconstructor/SfMData.h>
#include <manifoldReconstructor/types_reconstructor.hpp>


#include <meshac/PhotogrammetristAccuracyModel.hpp>
#include <meshac/ComputerVisionAccuracyModel.hpp>

#define POINT 1

int main(int argc, char **argv) {
    std::srand(0);
    
    std::string cloudPath = argv[1];
    OpenMvgParser op(argv[1]);
    op.parse();
    
    SfMData points = op.getSfmData();

    cloudPath = cloudPath.substr(0, cloudPath.find_last_of("/"));
    std::string pathPrefix = cloudPath.substr(0, cloudPath.find_last_of("/")+1);

    meshac::PhotogrammetristAccuracyModel model(points, pathPrefix);
    //meshac::ComputerVisionAccuracyModel model(points, pathPrefix);

    //std::cout << points.points_[POINT][0] << " " << points.points_[POINT][1] << " " << points.points_[POINT][2] << std::endl;
    auto acc = model.getCompleteAccuracyForPoint(POINT);
    //auto acc = model.getAccuracyForPoint(POINT);

    std::cout << acc << std::endl;

    return 0;
}

