
#include <cmath>
#include <iostream>
#include <fstream>

#include <glm/gtx/string_cast.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <aliases.h>

#define ARGS 8

void printUsage(char *execName);
float deg(float rad);
float rad(float deg);

int main(int argc, char **argv)
{    
    if (argc < ARGS + 1){
        printUsage(argv[0]);
        return 1;
    }
    std::string posesListFilename(argv[1]);
    float ptx = strtod(argv[2], NULL);
    float pty = strtod(argv[3], NULL);
    float ptz = strtod(argv[4], NULL);
    float ltx = strtod(argv[5], NULL);
    float lty = strtod(argv[6], NULL);
    float ltz = strtod(argv[7], NULL);
    int n = atoi(argv[8]);

    std::ifstream cin(posesListFilename);
    Vec3 position, lookAt, angles;
    cin >> position.x >> position.y >> position.z >> lookAt.x >> lookAt.y >> lookAt.z;

    Vec3 deltaPos(ptx / (float)n, pty / (float)n, ptz / (float)n);
    Vec3 deltaLook(ltx / (float)n, lty / (float)n, ltz / (float)n);


    for (int i = 0; i < n; i++) {
        auto pos = position + deltaPos * (float)i;
        auto la = lookAt + deltaLook * (float)i;

        std::cout << pos.x << " " << pos.y << " " << pos.z << " " << la.x << " " << la.y << " " << la.z << std::endl; 
    }

    cin.close();

    return 0;
}

void printUsage(char *execName)
{
    std::cout << execName << " list_pose_filename ptx pty ptz ltx lty ltz n" << std::endl;
}

float deg(float rad)
{
    return rad * 180.0f / M_PI;
}

float rad(float deg)
{
    return deg * M_PI / 180.0f;
}

