
#include <cmath>
#include <iostream>
#include <fstream>

#include <glm/gtx/string_cast.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <aliases.h>

#define ARGS 1

void printUsage(char *execName);
void mappingPose(std::ifstream &cin, std::ofstream &cout);
float deg(float rad);
float rad(float deg);

int main(int argc, char **argv)
{
    
    if (argc < ARGS + 1){
        printUsage(argv[0]);
        return 1;
    }
    std::string posesListFilename(argv[1]);

    std::ifstream cin(posesListFilename);
    std::ofstream cout("output_mapping.txt");

    while(!cin.eof()) {
        mappingPose(cin, cout);
    }

    cin.close();
    cout.close();

    return 0;
}

void printUsage(char *execName)
{
    std::cout << execName << " list_pose_filename" << std::endl;
}

void mappingPose(std::ifstream &cin, std::ofstream &cout)
{
    Vec3 position, lookAt, angles;
    std::string filename;
    cin >> filename >> position.x >> position.y >> position.z >> lookAt.x >> lookAt.y >> lookAt.z;

    ProjectionMatrix view = glm::lookAt(position, lookAt, Vec3(0, 1, 0));
    RotationMatrix rot(view);

    angles.x = std::atan2(rot[0][1], rot[0][0]);    // roll
    angles.y = std::atan2(-rot[0][2], std::sqrt(std::pow(rot[1][2],2) + std::pow(rot[2][2], 2)));   // pitch
    angles.z = std::atan2(rot[1][2], rot[2][2]);    // yaw

    angles.x = deg(angles.x);
    angles.y = deg(angles.y);
    angles.z = deg(angles.z);

    std::cout << glm::to_string(angles) << std::endl;

    cout << filename << " " << position.x << "_" << position.y << "_" << position.z;
    cout << "_" << angles.x << "_" << angles.y << "_" << angles.z << std::endl;
}

std::string extractPrefix(std::string &filename)
{
    return filename.substr(0, filename.find_first_of("_"));
}

float deg(float rad)
{
    return rad * 180.0f / M_PI;
}

float rad(float deg)
{
    return deg * M_PI / 180.0f;
}
