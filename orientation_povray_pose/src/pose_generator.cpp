
#include <cmath>
#include <iostream>
#include <fstream>

#include <glm/gtx/string_cast.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <aliases.h>

#define ARGS 3

void printUsage(char *execName);
float deg(float rad);
float rad(float deg);

RotationMatrix rotationMatrix(float yaw, float pitch, float roll);
RotationMatrix rotationMatriax(Vec3 &angles);
RotationMatrix rotationRoll(float roll);
RotationMatrix rotationPitch(float pitch);
RotationMatrix rotationYaw(float yaw);

int main(int argc, char **argv)
{    
    if (argc < ARGS + 1){
        printUsage(argv[0]);
        return 1;
    }
    std::string posesListFilename(argv[1]);
    float angle = strtod(argv[2], NULL);
    int n = atoi(argv[3]);

    std::ifstream cin(posesListFilename);
    Vec3 position, lookAt, angles;
    cin >> position.x >> position.y >> position.z >> lookAt.x >> lookAt.y >> lookAt.z;

    Vec3 v = position - lookAt;

    float deltaDeg = angle / (float)n;

    RotationMatrix rot = rotationPitch(rad(deltaDeg));

    for (int i = 0; i < n; i++) {
        RotationMatrix rot = rotationPitch(rad(deltaDeg * i));

        auto pos = v * rot + lookAt;

        std::cout << pos.x << " " << pos.y << " " << pos.z << " " << lookAt.x << " " << lookAt.y << " " << lookAt.z << std::endl; 
    }

    cin.close();

    return 0;
}

void printUsage(char *execName)
{
    std::cout << execName << " list_pose_filename angle n" << std::endl;
}

float deg(float rad)
{
    return rad * 180.0f / M_PI;
}

float rad(float deg)
{
    return deg * M_PI / 180.0f;
}

RotationMatrix rotationMatrix(float yaw, float pitch, float roll)
{
    return (rotationYaw(yaw) * rotationPitch(pitch)) * rotationRoll(roll);
}

RotationMatrix rotationMatrix(Vec3 &angles)
{
    return (rotationYaw(angles.z) * rotationPitch(angles.y)) * rotationRoll(angles.x);
}

RotationMatrix rotationRoll(float roll)
{
    return RotationMatrix(1.0f, 0.0f, 0.0f,
            0.0f, std::cos(roll), -std::sin(roll),
            0.0f, std::sin(roll), std::cos(roll));
}

RotationMatrix rotationPitch(float pitch)
{
    return RotationMatrix(std::cos(pitch), 0.0f, std::sin(pitch),
                    0.0f, 1.0f, 0.0f,
                    -std::sin(pitch), 0.0f, std::cos(pitch));
}

RotationMatrix rotationYaw(float yaw)
{
    return RotationMatrix(std::cos(yaw), -std::sin(yaw), 0.0f,
                    std::sin(yaw), std::cos(yaw), 0.0f,
                    0.0f, 0.0f, 1.0f);
}
