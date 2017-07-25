
#include <omp.h>

#include <fstream>
#include <iostream>

#include <glm/glm.hpp>

#include <KDTree.hpp>
#include <Mapper.hpp>
#include <utilities.hpp>

#define OMP_THREADS 1

#define ARGS 4

void fillVectors(std::string &line, glm::vec3 &position, glm::vec3 &lookAt);
std::pair<glm::vec3, glm::vec3> transformToAnglePose(glm::vec3 &position, glm::vec3 &lookAt);
glm::vec3 computeAngles(glm::mat3 &rot);
std::string createEntry(std::pair<glm::vec3, glm::vec3> &closest);
double slowMappingToResult(std::pair<glm::vec3, glm::vec3> pose, std::string &results, std::string &mapping);

/* Prints real position, closest position and result */
void printFinalPose(std::pair<glm::vec3, glm::vec3> &closest, std::pair<glm::vec3, glm::vec3> &pose, std::string &results, std::string &mapping);


int main(int argc, char **argv) {

    omp_set_num_threads(OMP_THREADS);

    if (argc < ARGS + 1) {
        std::cout << "Usage: " << argv[0] << " database.txt mapping.txt results.txt points.txt" << std::endl;
        return 1;
    }

    std::string database = argv[1];
    std::string mappingFile = argv[2];
    std::string resultsFile = argv[3];
    std::string pointFile = argv[4];

    std::string content = cameval::readStringFromFile(database);

    std::vector<std::string> entryPoses, correctPoses;
    boost::split(entryPoses, content, boost::is_any_of("\n"));
    for (int i = 0; i < entryPoses.size(); i++) {
        auto realPose = cameval::Mapper::slowMappingToPose(entryPoses[i]);
        
        glm::vec3 angles = computeAngles(realPose.second);
        auto pp = std::make_pair(realPose.first, angles);
        correctPoses.push_back(createEntry(pp));
    }

    cameval::KDTree tree(correctPoses);

    std::ifstream cin(pointFile);
    std::string line;
    while (!cin.eof()) {
        cin >> line;
        glm::vec3 position, angles;
        fillVectors(line, position, angles);

        auto pose = std::make_pair(position, angles);
        std::pair<glm::vec3, glm::vec3> closest = tree.searchClosestPoint(pose);

        printFinalPose(closest, pose, resultsFile, mappingFile);

    }
    cin.close();

    return 0;
}

void fillVectors(std::string &line, glm::vec3 &position, glm::vec3 &lookAt)
{
    std::vector<std::string> blocks;
    boost::split(blocks, line, boost::is_any_of(" "));
    
    for (int i = 0; i < 3; i++) {
        position[i] = std::strtod(blocks[i + 1].c_str(), NULL);
    }
    for (int i = 0; i < 3; i++) {
        lookAt[i] = std::strtod(blocks[i + 1 + 3].c_str(), NULL);
    }
}


std::pair<glm::vec3, glm::vec3> transformToAnglePose(glm::vec3 &position, glm::vec3 &lookAt)
{

    std::pair<glm::vec3, glm::vec3> pose;
    glm::mat4 view = glm::lookAt(position, lookAt, glm::vec3(0, -1, 0));
    glm::mat3 rot(view);
    glm::vec3 angle = computeAngles(rot);

    return pose;
}

glm::vec3 computeAngles(glm::mat3 &rot)
{
    glm::vec3 angle;
    // NOTE remember to invert rows and cols
    angle.x = std::atan2(rot[0][1], rot[0][0]);    // roll
    angle.y = std::atan2(-rot[0][2], std::sqrt(std::pow(rot[1][2],2) + std::pow(rot[2][2], 2)));   // pitch
    angle.z = std::atan2(rot[1][2], rot[2][2]);    // yaw

    angle.x = cameval::deg(angle.x);
    angle.y = cameval::deg(angle.y);
    angle.z = cameval::deg(angle.z);

    return angle;
}

std::string createEntry(std::pair<glm::vec3, glm::vec3> &closest)
{
    return "this/is/the/prefix_" + std::to_string(closest.first.x) + "_" + std::to_string(closest.first.y) + "_" + std::to_string(closest.first.z) + "_" + 
            std::to_string(closest.second.x) + "_" + std::to_string(closest.second.y) + "_" + std::to_string(closest.second.z) + ".suffix";
}

void printFinalPose(std::pair<glm::vec3, glm::vec3> &closest, std::pair<glm::vec3, glm::vec3> &pose, std::string &results, std::string &mapping)
{
    std::cout << closest.first.x << " " << closest.first.y << " " << closest.first.z << " ";
    std::cout << closest.second.x << " " << closest.second.y << " " << closest.second.y << std::endl << "\t";

    std::cout << pose.first.x << " " << pose.first.y << " " << pose.first.z << " ";
    std::cout << pose.second.x << " " << pose.second.y << " " << pose.second.z << " ";

    std::cout << slowMappingToResult(pose, results, mapping) << std::endl;
}

double slowMappingToResult(std::pair<glm::vec3, glm::vec3> pose, std::string &results, std::string &mapping)
{
    std::string entry, pose_real;
    std::ifstream cin(mapping);
    while (!cin.eof()) {
        cin >> entry >> pose_real;
        std::pair<glm::vec3, glm::vec3> tmp = cameval::parseEntry(pose_real);
        std::pair<glm::vec3, glm::vec3> compare = transformToAnglePose(tmp.first, tmp.second);

        if (glm::all(glm::equal(compare.first, pose.first)) && glm::all(glm::equal(compare.second, pose.second))) {
            cin.close();
            break;
        }
    }

    entry = entry.substr(entry.find_last_of("/") + 1, entry.size());
    entry = entry.substr(0, entry.find_last_of("."));
    
    cin.open(results);

    while (!cin.eof()) {
        std::string filename, result;
        cin >> filename >> result;

        filename = cameval::trim(filename);
        if (filename.compare(entry) == 0) {
            cin.close();
            return std::strtod(result.c_str(), NULL);
        }
    }

    cin.close();
    return -1.0;
}