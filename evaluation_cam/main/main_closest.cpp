#include <omp.h>

#include <fstream>
#include <iostream>

#include <glm/glm.hpp>

#include <KDTree.hpp>
#include <InputReader.hpp>
#include <utilities.hpp>

#define OMP_THREADS 1

#define ARGS 3

void fillVectors(std::string &line, glm::vec3 &position, glm::vec3 &lookAt);
std::pair<glm::vec3, glm::vec3> transformToAnglePose(glm::vec3 &position, glm::vec3 &lookAt);
glm::vec3 computeAngles(glm::mat3 &rot);


/* Prints real position, closest position and result */
void printFinalPose(std::ofstream &cout, std::pair<glm::vec3, glm::vec3> &closest, std::pair<glm::vec3, glm::vec3> &pose);


int main(int argc, char **argv) {

    omp_set_num_threads(OMP_THREADS);

    if (argc < ARGS + 1) {
        std::cout << "Usage: " << argv[0] << " mapping.txt points.txt outputfile.txt" << std::endl;
        return 1;
    }

    std::string mappingFile = argv[1];
    std::string pointFile = argv[2];
    std::string outputfile = argv[3];

    cameval::StringPoseMapping mapping = cameval::InputReader::readMappingDatabase(mappingFile);
    cameval::PoseList poses = mapping.second;
    cameval::AnglePoseList anglePoses;

    for (int i = 0; i < poses.size(); i++) {
        glm::vec3 angles = computeAngles(poses[i].second);
        anglePoses.push_back(std::make_pair(poses[i].first, angles));
    }

    cameval::KDTree tree(anglePoses);

    std::ifstream cin(pointFile);
    std::ofstream cout(outputfile);
    std::string line;
    while (!cin.eof()) {
        cin >> line;
        cameval::AnglePose pose = cameval::parseEntry(line);

        pose = transformToAnglePose(pose.first, pose.second);
        std::pair<glm::vec3, glm::vec3> closest = tree.searchClosestPoint(pose);

        printFinalPose(cout, closest, pose);
    }
    cin.close();
    cout.close();

    return 0;
}

std::pair<glm::vec3, glm::vec3> transformToAnglePose(glm::vec3 &position, glm::vec3 &lookAt)
{
    glm::mat4 view = glm::lookAt(position, lookAt, glm::vec3(0, -1, 0));
    glm::mat3 rot(view);
    glm::vec3 angle = computeAngles(rot);

    return std::make_pair(position, angle);
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

void printFinalPose(std::ofstream &cout, std::pair<glm::vec3, glm::vec3> &closest, std::pair<glm::vec3, glm::vec3> &pose)
{
    cout << closest.first.x << "_" << closest.first.y << "_" << closest.first.z << "_";
    cout << closest.second.x << "_" << closest.second.y << "_" << closest.second.z << std::endl << "\t";
    cout << pose.first.x << "_" << pose.first.y << "_" << pose.first.z << "_";
    cout << pose.second.x << "_" << pose.second.y << "_" << pose.second.z;
    cout << std::endl;
}
