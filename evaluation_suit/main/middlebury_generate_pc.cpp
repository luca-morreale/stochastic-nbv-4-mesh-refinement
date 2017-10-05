#include <cstdlib>
#include <iostream>
#include <fstream>
#include <sstream>

#include <utilities.hpp>
#include <aliases.h>
#include <type_definition.h>
#include <OpenMvgJsonHandler.hpp>
#include <MiddleburyDatasetReader.hpp>
#include <OpenMvgSysCall.hpp>


#define ARGS 2

cameval::PoseList cameraPoses;

void log(std::string msg)
{
    std::cout << msg << std::endl;
}

cameval::Pose getPose(cameval::Camera &cam)
{
    auto t = glm::vec3(cam.t);
    auto R = glm::mat3(cam.R);
    t = - R * t;
    // R = glm::transpose(R);
    return cameval::Pose(t, R);
}

void setupCameraPoses(cameval::CameraList &cameras, cameval::StringList &views)
{
    for (int i = 0; i < cameras.size(); i++) {
        cameraPoses.push_back(getPose(cameras[i]));
    }
}

void setDefaultCameraPoses(cameval::OpenMvgJsonHandler &mvgJsonHandler)
{
    for (int index = 0; index < cameraPoses.size(); index++) {
        mvgJsonHandler.setCamPosition(index, cameraPoses[index].first, cameraPoses[index].second);
    }
}

void initOpenMvg()
{
    cameval::OpenMvgSysCall::initOpenMvg();

    cameval::OpenMvgJsonHandler mvgJsonHandler("matches/sfm_data.json");
    log("\nSet default camera poses");
    setDefaultCameraPoses(mvgJsonHandler);
    mvgJsonHandler.saveChanges();

}
    
void generateBasic()
{
    initOpenMvg();

    std::string jsonFile = "matches/sfm_data.json";
    std::string pairs = "";
    
    cameval::OpenMvgSysCall::extractMvgFeatures(jsonFile, pairs);

    log("\nCompute Structure");
    cameval::OpenMvgSysCall::computeStructureFromPoses(jsonFile, 0);

}


int main(int argc, char **argv) {

    if (argc < ARGS + 1) {
        std::cout << "Usage: " << std::string(argv[0]) << " cam_poses imageFolder" << std::endl;
        return 1;
    }

    std::string basicPoseFilename = std::string(argv[1]);
    cameval::OpenMvgSysCall::baseImageFolder = std::string(argv[2]);

    // TODO set up intrinsic parameters

    cameval::MiddleburyDatasetReader reader(basicPoseFilename);
    auto cameras = reader.getCamerasMatrices();
    auto views = reader.getViews();
    log("\nBasic poses size: " + std::to_string(cameras.size()));

    cameval::OpenMvgSysCall::intrinsicParams =  "";
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            cameval::OpenMvgSysCall::intrinsicParams += std::to_string(cameras[0].K[i][j]);
            if (i != 2 || j != 2) cameval::OpenMvgSysCall::intrinsicParams += ";";
        }
    }

    setupCameraPoses(cameras, views);
    
    generateBasic();


    return 0;
}
