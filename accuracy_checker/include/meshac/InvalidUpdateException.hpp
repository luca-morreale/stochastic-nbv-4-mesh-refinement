#ifndef MESH_ACCURACY_INVALID_UPDATE_EXCEPTION_H
#define MESH_ACCURACY_INVALID_UPDATE_EXCEPTION_H

#include <stdexcept>

namespace meshac {

    class InvalidUpdateException : public std::runtime_error {
    public:
        InvalidUpdateException(std::string msg) : std::runtime_error(msg)
        { /*    */ }

        InvalidUpdateException() : std::runtime_error("Invalid Updated performed.")
        { /*    */ }

    };

} // namespace meshac

#endif // MESH_ACCURACY_INVALID_UPDATE_EXCEPTION_H
