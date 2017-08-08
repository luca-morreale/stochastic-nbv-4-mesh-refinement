#ifndef MESH_ACCURACY_UNEXPECTED_TRIANGLE_EXCEPTION_H
#define MESH_ACCURACY_UNEXPECTED_TRIANGLE_EXCEPTION_H

#include <stdexcept>

namespace meshac {

    class UnexpectedTriangleException : public std::runtime_error {
    public:
        UnexpectedTriangleException(std::string msg) : std::runtime_error(msg)
        { /*    */ }

        UnexpectedTriangleException(Point &a, Point &b, Point &c) : std::runtime_error("There is no triangle composed by " 
                        "("+ std::to_string(a[0]) + "; " + std::to_string(a[1]) + "; " + std::to_string(a[2]) + ")" 
                        "("+ std::to_string(b[0]) + "; " + std::to_string(b[1]) + "; " + std::to_string(b[2]) + ")" 
                        "("+ std::to_string(c[0]) + "; " + std::to_string(c[1]) + "; " + std::to_string(c[2]) + ").")
        { /*    */ }

        UnexpectedTriangleException() : std::runtime_error("The given points does not form a triangle present in the mesh given.")
        { /*    */ }

    };

} // namespace meshac

#endif // MESH_ACCURACY_UNEXPECTED_TRIANGLE_EXCEPTION_H
