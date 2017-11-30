#ifndef MESH_ACCURACY_UNEXPECTED_POINT_EXCEPTION_H
#define MESH_ACCURACY_UNEXPECTED_POINT_EXCEPTION_H

#include <stdexcept>

namespace meshac {

    class UnexpectedPointException : public std::runtime_error {
    public:
        UnexpectedPointException(std::string msg) : std::runtime_error(msg)
        { /*    */ }

        UnexpectedPointException(Point &point) : std::runtime_error("Point (" + std::to_string(point[0]) + ", " + 
                                std::to_string(point[1]) + ", " + std::to_string(point[2]) + ") is not part of the mesh.")
        { /*    */ }

        UnexpectedPointException(GLMVec3 &point) : std::runtime_error("Point ("+ std::to_string(point.x) + ", " + 
                                std::to_string(point.y) + ", " + std::to_string(point.z) + ") is not part of the mesh.")
        { /*    */ }

        UnexpectedPointException() : std::runtime_error("Given point is not part of the set given.")
        { /*    */ }

    };

} // namespace meshac

#endif // MESH_ACCURACY_UNEXPECTED_POINT_EXCEPTION_H
