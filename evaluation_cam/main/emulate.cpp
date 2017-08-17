#include <cstdlib>
#include <iostream>
#include <fstream>
#include <sstream>

#include <utilities.hpp>
#include <aliases.h>
#include <PoseEvaluator.hpp>

#define ARGS 3

void log(std::string msg);
std::string convert(std::string &poseFile);
std::string transformToLookat(std::string &fixedPosefile);
std::vector<std::string> transform(std::vector<std::string> &blocks);

int main(int argc, char **argv) {

    if (argc < ARGS + 1) {
        log("Usage: " + std::string(argv[0]) + " pose_sequence_file.txt mode[s/u] basicpvfile.pov");
        return 1;
    }

    std::string poseFile = argv[1];
    std::string mode = argv[2];
    std::string basicPovFile = argv [3];
    std::string fixedPosefile;

    if (mode.compare("s") == 0) {
        fixedPosefile = convert(poseFile);
    } else if (mode.compare("u") == 0) {
        fixedPosefile = poseFile;
    } else {
        log("mode must be either u or s");
        return 1;
    }

    fixedPosefile = transformToLookat(fixedPosefile);

    std::string convertedPosesFile = "output/output_transform.txt";
    std::string outputEvaluation = "output/output_evaluation.txt";
    std::string command;
    
    command = "./transform b " + fixedPosefile + " " + convertedPosesFile;
    system(command.c_str());

    std::string groundTruthFilename = "pcl_gt.asc";
    std::string basicPosesFilename = "basicposecam.txt";
    std::string baseImageFolder = "images";
    std::string intrinsicParams = "959.9965;0;960;0;-959.9965;540;0;0;1";
    
    cameval::PoseEvaluator eval(convertedPosesFile, groundTruthFilename, basicPosesFilename, baseImageFolder, intrinsicParams, outputEvaluation, basicPovFile);
    eval.evaluate();

    cameval::FileHandler::cleanAll({fixedPosefile, convertedPosesFile});

    return 0;
}

std::string transformToLookat(std::string &fixedPosefile)
{
    std::ifstream cin(fixedPosefile);
    std::stringstream stream;

    while(!cin.eof()) {
        std::string line;
        cin >> line;
        if (line.size() == 0) continue;

        std::vector<std::string> blocks;
        boost::split(blocks, line, boost::is_any_of("_"));

        blocks = transform(blocks);
        for (int i = 0; i < blocks.size(); i++) {
            stream << blocks[i] << " ";
        }
        stream << std::endl;
    }

    cin.close();

    std::string newFile = "converted_poses.txt";
    std::ofstream cout(newFile);
    cout << stream.str();
    cout.close();

    return newFile;
}

std::vector<std::string> transform(std::vector<std::string> &blocks)
{
    std::vector<std::string> list(blocks.begin(), blocks.end());
    float roll = 0;
    float pitch = strtod(blocks[3].c_str(), NULL);
    float yaw = strtod(blocks[4].c_str(), NULL);

    glm::mat3 rot = cameval::rotationMatrix(roll, cameval::rad(pitch), cameval::rad(yaw));

    glm::vec3 zdir(0.0f, 0.0f, 5.0f);
    zdir = rot * zdir;

    list[3] = std::to_string(zdir[0]);
    list[4] = std::to_string(zdir[1]);
    list.push_back(std::to_string(zdir[2]));
    return list;
}

std::string convert(std::string &poseFile)
{
    std::string newFile = "converted_poses.txt";
    std::ofstream cout(newFile);
    std::ifstream cin(poseFile);

    while(!cin.eof()) {
        std::string x, y, z, ly,lz;
        cin >> x >> y >> z >> ly >> lz;
        if (x.size() == 0 || y.size() == 0 || z.size() == 0 || ly.size() == 0 || lz.size() == 0) continue;
        cout << x << "_" << y << "_" << z << "_" << ly << "_" << lz << std::endl;
    }

    cin.close();
    cout.close();

    return newFile;
}

void log(std::string msg)
{
    std::cout << msg << std::endl;
}
