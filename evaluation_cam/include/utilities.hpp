#ifndef EVALUATION_CAMERA_POSITION_UTILITIES_H_
#define EVALUATION_CAMERA_POSITION_UTILITIES_H_

#include <algorithm> 
#include <functional> 
#include <cctype>
#include <locale>
#include <cmath>

#include <aliases.h>

namespace cameval {

    void log(std::string msg);

    std::string &ltrim(std::string &s);
    std::string &rtrim(std::string &s);
    std::string &trim(std::string &s);

    GLMVec3 convert(EigVector3 &arr);
    GLMMat3 convert(EigMatrix3 &mat);

    EigMatrix3 rotationMatrix(float yaw, float pitch, float roll);

} // namespace cameval

#endif // EVALUATION_CAMERA_POSITION_UTILITIES_H_
