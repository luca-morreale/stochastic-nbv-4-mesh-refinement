#ifndef MESH_ACCURACY_INVALID_UPDATE_EXCEPTION_H
#define MESH_ACCURACY_INVALID_UPDATE_EXCEPTION_H

#include <stdexcept>

namespace meshac {

    class UnexpectedPointException : public std::runtime_error {
    public:
        UnexpectedPointException(std::string msg) : std::runtime_error(msg)
        { /*    */ }

        UnexpectedPointException(GLMVec3 &point) : std::runtime_error("Point ("+ std::to_string(point.x) + ", " + 
                                std::to_string(point.y) + ", " + std::to_string(point.z) + ") is not part of the mesh.")
        { /*    */ }

        UnexpectedPointException() : std::runtime_error("Given point is not part of the set given.")
        { /*    */ }

    };

} // namespace meshac

#endif // MESH_ACCURACY_INVALID_UPDATE_EXCEPTION_H
