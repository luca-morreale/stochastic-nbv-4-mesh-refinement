#include <utilities.hpp>

namespace cameval {

    void log(std::string msg)
    {
        std::cout << msg << std::endl;
    }

    // trim from start
    std::string &ltrim(std::string &s) {
        s.erase(s.begin(), std::find_if(s.begin(), s.end(),
                std::not1(std::ptr_fun<int, int>(std::isspace))));
        return s;
    }

    // trim from end
    std::string &rtrim(std::string &s) {
        s.erase(std::find_if(s.rbegin(), s.rend(),
                std::not1(std::ptr_fun<int, int>(std::isspace))).base(), s.end());
        return s;
    }

    // trim from both ends
    std::string &trim(std::string &s) {
        return ltrim(rtrim(s));
    }

    GLMVec3 convert(EigVector3 &arr)
    {
        return GLMVec3(arr[0], arr[1], arr[2]);
    }

    GLMMat3 convert(EigMatrix3 &mat)
    {
        return GLMMat3(mat(0, 0), mat(0, 1), mat(0, 2), mat(1, 0), mat(1, 1), mat(1, 2), mat(2, 0), mat(2, 1), mat(2, 2));
    }

    EigMatrix3 rotationMatrix(float yaw, float pitch, float roll)
    {
        // Calculate rotation about x axis
        EigMatrix3 Rx;
        Rx << 1.0f, 0.0f, 0.0f, 
                0.0f, std::cos(roll), -std::sin(roll),
                0.0f, std::sin(roll), std::cos(roll);
        // Calculate rotation about y axis
        EigMatrix3 Ry;
        Ry << std::cos(pitch), 0.0f, std::sin(pitch), 
                0.0f, 1.0f, 0.0f,
                -std::sin(pitch), 0.0f, std::cos(pitch);
        // Calculate rotation about z axis
        EigMatrix3 Rz;
        Rz << std::cos(yaw), -std::sin(yaw), 0.0f, 
                std::sin(yaw), std::cos(yaw), 0.0f,
                0.0f, 0.0f, 1.0f;
        return (Rz * Ry) * Rx;
    }

} // namespace cameval
