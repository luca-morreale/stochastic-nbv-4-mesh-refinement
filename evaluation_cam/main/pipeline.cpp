#include <cstdlib>
#include <iostream>
#include <fstream>

#include <utilities.hpp>
#include <aliases.h>
#include <EvaluatorSequence.hpp>

#define ARGS 2

void log(std::string msg);
std::string convert(std::string &poseFile);

int main(int argc, char **argv) {

    if (argc < ARGS + 1) {
        log("Usage: " + std::string(argv[0]) + " pose_sequence_file.txt mode[s/u]");
        return 1;
    }

    std::string poseFile = argv[1];
    std::string mode = argv[2];
    std::string fixedPosefile;

    if (mode.compare("s") == 0) {
        fixedPosefile = convert(poseFile);
    } else if (mode.compare("u") == 0) {
        fixedPosefile = poseFile;
    } else {
        log("mode must be either u or s");
        return 1;
    }

    std::string convertedPosesFile = "output/output_transform.txt";
    std::string closestPosesFile = "output/output_closest.txt";
    std::string outputEvaluation = "output/output_evaluation.txt";

    std::string command = "./transform b " + fixedPosefile + " " + convertedPosesFile;
    system(command.c_str());

    command = "./closest output_mapping.txt " + convertedPosesFile + " " + closestPosesFile;
    system(command.c_str());

    std::string groundTruthFilename = "pcl_gt.asc";
    std::string basicPosesFilename = "basicposecam.txt";
    std::string baseImageFolder = "images";
    std::string intrinsicParams = "959.9965;0;960;0;-959.9965;540;0;0;1";
    std::string sshconfig = "ssh_config.txt";

    cameval::EvaluatorSequence eval(closestPosesFile, groundTruthFilename, basicPosesFilename, baseImageFolder, intrinsicParams, outputEvaluation, sshconfig);
    eval.evaluate();

    if (mode.compare("s") == 0) {
        cameval::FileHandler::cleanAll({fixedPosefile, convertedPosesFile, closestPosesFile});
    } else {
        cameval::FileHandler::cleanAll({convertedPosesFile, closestPosesFile});
    }


    return 0;
}

std::string convert(std::string &poseFile)
{
    std::string newFile = "converted_poses.txt";
    std::ofstream cout(newFile);
    std::ifstream cin(poseFile);

    while(!cin.eof()) {
        std::string x, y, z, lx, ly,lz;
        cin >> x >> y >> z >> lx >> ly >> lz;
        cout << x << "_" << y << "_" << z << "_" << lx << "_" << ly << "_" << lz << std::endl;
    }

    cin.close();
    cout.close();

    return newFile;
}

void log(std::string msg)
{
    std::cout << msg << std::endl;
}
