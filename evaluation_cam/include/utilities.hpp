#ifndef EVALUATION_CAMERA_POSITION_UTILITIES_H_
#define EVALUATION_CAMERA_POSITION_UTILITIES_H_

#include <algorithm> 
#include <functional>
#include <fstream>
#include <cctype>
#include <locale>
#include <cmath>

#include <boost/algorithm/string.hpp>

#include <aliases.h>

namespace cameval {

    void log(std::string msg);

    std::string &ltrim(std::string &s);
    std::string &rtrim(std::string &s);
    std::string &trim(std::string &s);
    std::string readStringFromFile(std::string &filename);
    AnglePose parseEntry(std::string &entry);

    GLMVec3 convert(EigVector3 &arr);
    GLMMat3 convert(EigMatrix3 &mat);

    float rad(float deg);
    float deg(float rad);
    GLMMat3 rotationMatrix(float yaw, float pitch, float roll);
    GLMMat3 rotationMatrix(GLMVec3 &angles);
    GLMMat3 rotationRoll(float roll);
    GLMMat3 rotationPitch(float pitch);
    GLMMat3 rotationYaw(float yaw);

} // namespace cameval

#endif // EVALUATION_CAMERA_POSITION_UTILITIES_H_
