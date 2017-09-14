#include <cstdlib>
#include <iostream>
#include <fstream>
#include <sstream>

#include <utilities.hpp>
#include <aliases.h>
#include <OpenMvgJsonHandler.hpp>
#include <PoseReader.hpp>
#include <OpenMvgSysCall.hpp>

#define ARGS 2

cameval::PoseList cameraPoses;

void log(std::string msg)
{
    std::cout << msg << std::endl;
}

void setDefaultCameraPoses(cameval::OpenMvgJsonHandler &mvgJsonHandler)
{
    for (int index = 0; index < cameraPoses.size(); index++) {
        mvgJsonHandler.setCamPosition(index, cameraPoses[index].first, cameraPoses[index].second);
    }

}

std::string computeDistance(std::string &alignedCloud, std::string &groundTruthFilename)
{
    std::string logfilename = alignedCloud.substr(0, alignedCloud.find_last_of("/")) + "/log_distance.txt";

    // NOTE no need to remove cameras because assumed main_Structure... already output a structure without cameras and in ascii

    std::string command = "CloudCompare -SILENT -LOG_FILE " + logfilename + " -O " + alignedCloud + " -O " + groundTruthFilename + " -c2c_dist";
    system(command.c_str());

    return logfilename;
}

void initOpenMvg()
{
    cameval::OpenMvgSysCall::initOpenMvg();

    cameval::OpenMvgJsonHandler mvgJsonHandler("matches/sfm_data.json");
    log("\nSet default camera poses");
    setDefaultCameraPoses(mvgJsonHandler);
    mvgJsonHandler.saveChanges();

}
    
void generateBasic(std::string &groundTruthFilename)
{
    initOpenMvg();

    std::string jsonFile = "matches/sfm_data.json";
    std::string pairs = "";
    
    cameval::OpenMvgSysCall::extractMvgFeatures(jsonFile, pairs);

    log("\nCompute Structure");
    std::string mvgFolder = cameval::OpenMvgSysCall::computeStructureFromPoses(jsonFile, 0);

    std::string inputCloud = mvgFolder + "/sfm_data.ply";
    
    log("\nCompute Distance");
    std::string logFile = computeDistance(inputCloud, groundTruthFilename); 
}


int main(int argc, char **argv) {

    if (argc < ARGS + 1) {
        std::cout << "Usage: " << std::string(argv[0]) << " cam_poses gt.asc" << std::endl;
        return 1;
    }

    std::string basicPoseFilename = std::string(argv[1]);
    std::string gt = std::string(argv[2]);


    std::ifstream cin("K.txt");
    cin >> cameval::OpenMvgSysCall::intrinsicParams;
    cin.close();


    cameval::PoseReader reader(basicPoseFilename);
    cameraPoses = reader.getPoses();
    log("\nBasic poses size: " + std::to_string(cameraPoses.size()));

    
    generateBasic(gt);


    return 0;
}
