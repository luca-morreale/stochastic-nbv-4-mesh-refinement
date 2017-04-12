#ifndef MESH_ACCURACY_INVALID_JSON_FILE_EXCEPTION_H
#define MESH_ACCURACY_INVALID_JSON_FILE_EXCEPTION_H

#include <stdexcept>

namespace meshac {

    class InvalidJsonFileException : public std::runtime_error {
    public:
        InvalidJsonFileException(std::string msg) : std::runtime_error(msg)
        { /*    */ }

        InvalidJsonFileException() : std::runtime_error("Invalid JSON format.")
        { /*    */ }

    };

} // namespace meshac

#endif // MESH_ACCURACY_INVALID_JSON_FILE_EXCEPTION_H
