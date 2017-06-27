#ifndef MESH_ACCURACY_UNEXPECTED_TRIANGLE_EXCEPTION_H
#define MESH_ACCURACY_UNEXPECTED_TRIANGLE_EXCEPTION_H

#include <stdexcept>

namespace meshac {

    class UnexpectedTriangleException : public std::runtime_error {
    public:
        UnexpectedTriangleException(std::string msg) : std::runtime_error(msg)
        { /*    */ }

        UnexpectedTriangleException(GLMVec3 &a, GLMVec3 &b, GLMVec3 &c) : std::runtime_error("There is no triangle composed by " 
                        "("+ std::to_string(a.x) + "; " + std::to_string(a.y) + "; " + std::to_string(a.z) + ")" 
                        "("+ std::to_string(b.x) + "; " + std::to_string(b.y) + "; " + std::to_string(b.z) + ")" 
                        "("+ std::to_string(c.x) + "; " + std::to_string(c.y) + "; " + std::to_string(c.z) + ").")
        { /*    */ }

        UnexpectedTriangleException() : std::runtime_error("The given points does not form a triangle present in the mesh given.")
        { /*    */ }

    };

} // namespace meshac

#endif // MESH_ACCURACY_UNEXPECTED_TRIANGLE_EXCEPTION_H
